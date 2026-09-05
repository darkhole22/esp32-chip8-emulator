#include <stdio.h>
#include <assert.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_cpu.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

constexpr auto LCD_LOG_TAG = "LCD";
constexpr auto CHIP8_LOG_TAG = "Chip8";

constexpr gpio_num_t LCD_BACKLIGHT_PIN = GPIO_NUM_27;
constexpr gpio_num_t LCD_DC_PIN = GPIO_NUM_2;
constexpr gpio_num_t BUTTON_PIN = GPIO_NUM_0;

static uint32_t prng_state[4] = {0x1, 0x2, 0x3, 0x4};

FORCE_INLINE_ATTR uint32_t rotr(const uint32_t v, int n) {
    uint32_t res = (v >> n) | (v << (32 - n));
    // asm volatile("mov %0, %1, ror %2" : "=r"(res) : "r"(v), "r"(n));
    return res;
}

// Implementation from https://prng.di.unimi.it/xoshiro128starstar.c
static uint32_t prng_next() {
    const uint32_t result = rotr(prng_state[1] * 5, 32 - 7) * 9;

    const uint32_t t = prng_state[1] << 9;

    prng_state[2] ^= prng_state[0];
    prng_state[3] ^= prng_state[1];
    prng_state[1] ^= prng_state[2];
    prng_state[0] ^= prng_state[3];

    prng_state[2] ^= t;

    prng_state[3] = rotr(prng_state[3], 32 - 11);

    return result;
}

static uint32_t button_pressed = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    button_pressed = 1;
}

static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t* t) {
    intptr_t dc = (intptr_t)t->user;
    gpio_set_level(LCD_DC_PIN, dc);
}

static void spi_send_command_async(spi_device_handle_t handle, spi_transaction_t* t, uint8_t command) {
    t->flags = SPI_TRANS_USE_TXDATA;
    t->length = 8;
    t->tx_data[0] = command;
    t->user = (void*)0;
    esp_err_t err = spi_device_queue_trans(handle, t, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_send_data_async(spi_device_handle_t handle, spi_transaction_t* t, uint32_t len, uint8_t* data) {
    t->flags = 0;
    t->length = 8 * len;
    t->tx_buffer = data;
    t->user = (void*)1;
    esp_err_t err = spi_device_queue_trans(handle, t, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_send_data_async(spi_device_handle_t handle, spi_transaction_t* t, uint8_t data0, uint8_t data1,
                                uint8_t data2, uint8_t data3) {
    t->flags = SPI_TRANS_USE_TXDATA;
    t->length = 8 * 4;
    t->tx_data[0] = data0;
    t->tx_data[1] = data1;
    t->tx_data[2] = data2;
    t->tx_data[3] = data3;
    t->user = (void*)1;
    esp_err_t err = spi_device_queue_trans(handle, t, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_wait_trans(spi_device_handle_t handle, uint32_t trans_count) {
    spi_transaction_t* rtrans;
    esp_err_t ret;
    // Wait for all 6 transactions to be done and get back the results.
    for (uint32_t x = 0; x < trans_count; x++) {
        ret = spi_device_get_trans_result(handle, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
        // We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

enum class Touch_Value_Index {
    AXIS_X,
    AXIS_Y,
};

static uint16_t touch_read_value(spi_device_handle_t handle, Touch_Value_Index index) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 24;
    switch (index) {
    case Touch_Value_Index::AXIS_X:
        t.tx_data[0] = 0xD0;
        break;
    case Touch_Value_Index::AXIS_Y:
        t.tx_data[0] = 0x90;
        break;
    }
    esp_err_t err = spi_device_polling_transmit(handle, &t);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }

    uint16_t result = (((uint16_t)t.rx_data[1] & 0b01111111) << 4) | (((uint16_t)t.rx_data[2] & 0b11110000) >> 4);
    return result;
}

static void spi_send_command(spi_device_handle_t handle, uint8_t command, bool keep_active) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = command;
    t.user = (void*)0;
    if (keep_active) {
        t.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
    }
    esp_err_t err = spi_device_polling_transmit(handle, &t);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_send_data(spi_device_handle_t handle, uint32_t len, uint8_t data0, uint8_t data1, uint8_t data2,
                          uint8_t data3, bool keep_active) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8 * len;
    t.tx_data[0] = data0;
    t.tx_data[1] = data1;
    t.tx_data[2] = data2;
    t.tx_data[3] = data3;
    t.user = (void*)1;
    if (keep_active) {
        t.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
    }
    esp_err_t err = spi_device_polling_transmit(handle, &t);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_send_data(spi_device_handle_t handle, uint32_t len, const uint8_t* data) {
    spi_transaction_t t = {};
    t.flags = 0;
    t.length = 8 * len;
    t.tx_buffer = data;
    t.user = (void*)1;
    esp_err_t err = spi_device_polling_transmit(handle, &t);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to send command!");
        assert(false);
    }
}

static void spi_send_data(spi_device_handle_t handle, uint8_t data0) {
    spi_send_data(handle, 1, data0, 0, 0, 0, false);
}

static void spi_send_data(spi_device_handle_t handle, uint8_t data0, uint8_t data1) {
    spi_send_data(handle, 2, data0, data1, 0, 0, false);
}

DRAM_ATTR static const uint8_t lcd_porch_settings_cmd[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
DRAM_ATTR static const uint8_t lcd_pos_gamma_ctrl_cmd[] = {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37,
                                                           0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19};
DRAM_ATTR static const uint8_t lcd_neg_gamma_ctrl_cmd[] = {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D,
                                                           0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19};

constexpr uint16_t LCD_PARALLEL_LINES = 64;
constexpr uint16_t LCD_WIDTH = 240;
constexpr uint16_t LCD_HEIGHT = 320;

//  https://github.com/kripod/chip8-roms/blob/master/programs/Clock%20Program%20%5BBill%20Fisher%2C%201981%5D.ch8
const uint8_t chip8_rom[] = {
    0xf1, 0xa,  0xf2, 0xa,  0xf3, 0xa,  0xf4, 0xa,  0xf5, 0xa,  0xf6, 0xa,  0x0,  0xe0, 0x67, 0x1,  0x22, 0xce, 0x78,
    0x1,  0xf1, 0x29, 0xd7, 0x85, 0x67, 0xb,  0x22, 0xce, 0x78, 0x1,  0xf2, 0x29, 0xd7, 0x85, 0x67, 0x17, 0x22, 0xce,
    0x78, 0x1,  0xf3, 0x29, 0xd7, 0x85, 0x67, 0x21, 0x22, 0xce, 0x78, 0x1,  0xf4, 0x29, 0xd7, 0x85, 0x67, 0x2d, 0x22,
    0xce, 0x78, 0x1,  0xf5, 0x29, 0xd7, 0x85, 0x67, 0x37, 0x22, 0xce, 0x78, 0x1,  0xf6, 0x29, 0xd7, 0x85, 0xfd, 0xa,
    0x6d, 0x25, 0xfd, 0x15, 0x12, 0x60, 0x6d, 0x3b, 0xfd, 0x15, 0x22, 0xe8, 0x76, 0x1,  0x46, 0xa,  0x12, 0x6a, 0x22,
    0xe8, 0xfd, 0x7,  0x3d, 0x0,  0x12, 0x60, 0x2,  0xd8, 0x12, 0x52, 0x66, 0x0,  0x22, 0xe8, 0x22, 0xf0, 0x75, 0x1,
    0x45, 0x6,  0x12, 0x7a, 0x22, 0xf0, 0x12, 0x60, 0x65, 0x0,  0x22, 0xf0, 0x22, 0xf8, 0x74, 0x1,  0x44, 0xa,  0x12,
    0x8a, 0x22, 0xf8, 0x12, 0x60, 0x64, 0x0,  0x22, 0xf8, 0x23, 0x0,  0x73, 0x1,  0x43, 0x6,  0x12, 0x9a, 0x23, 0x0,
    0x12, 0x60, 0x63, 0x0,  0x23, 0x0,  0x23, 0x8,  0x72, 0x1,  0x42, 0x4,  0x12, 0xba, 0x42, 0xa,  0x12, 0xae, 0x23,
    0x8,  0x12, 0x60, 0x62, 0x0,  0x23, 0x8,  0x23, 0x10, 0x71, 0x1,  0x23, 0x10, 0x12, 0x60, 0x41, 0x2,  0x12, 0xc2,
    0x23, 0x8,  0x12, 0x60, 0x62, 0x0,  0x23, 0x8,  0x23, 0x10, 0x61, 0x0,  0x23, 0x10, 0x12, 0x60, 0x68, 0x7,  0xa2,
    0xe0, 0xd7, 0x87, 0x77, 0x1,  0x0,  0xee, 0xf8, 0xfa, 0xaf, 0x2f, 0x8f, 0x3a, 0xdb, 0xd4, 0xfc, 0xfc, 0xfc, 0xfc,
    0xfc, 0xfc, 0xfc, 0x0,  0x67, 0x38, 0xf6, 0x29, 0xd7, 0x85, 0x0,  0xee, 0x67, 0x2e, 0xf5, 0x29, 0xd7, 0x85, 0x0,
    0xee, 0x67, 0x22, 0xf4, 0x29, 0xd7, 0x85, 0x0,  0xee, 0x67, 0x18, 0xf3, 0x29, 0xd7, 0x85, 0x0,  0xee, 0x67, 0xc,
    0xf2, 0x29, 0xd7, 0x85, 0x0,  0xee, 0x67, 0x2,  0xf1, 0x29, 0xd7, 0x85, 0x0,  0xee,
};

DRAM_ATTR uint8_t chip8_ram[4096] = {
    0xF0, 0x90, 0x90, 0x90, 0xF0, // Character '0'
    0x20, 0x60, 0x20, 0x20, 0x70, // Character '1'
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // Character '2'
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // Character '3'
    0x90, 0x90, 0xF0, 0x10, 0x10, // Character '4'
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // Character '5'
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // Character '6'
    0xF0, 0x10, 0x20, 0x40, 0x40, // Character '7'
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // Character '8'
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // Character '9'
    0xF0, 0x90, 0xF0, 0x90, 0x90, // Character 'A'
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // Character 'B'
    0xF0, 0x80, 0x80, 0x80, 0xF0, // Character 'C'
    0xE0, 0x90, 0x90, 0x90, 0xE0, // Character 'D'
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // Character 'E'
    0xF0, 0x80, 0xF0, 0x80, 0x80, // Character 'F'
};

uint8_t chip8_delay_timer_register = 0;
uint8_t chip8_sound_timer_register = 0;
uint8_t chip8_button_pressed = 0x10;

static uint8_t chip8_screen[64 * 32];

static uint16_t frame_buffer[LCD_WIDTH * LCD_HEIGHT];

static spi_transaction_t send_data_transaction[6];

static void atomic_decrement_if_positive(uint8_t* val) {
    uint8_t old = __atomic_load_n(&chip8_delay_timer_register, __ATOMIC_ACQUIRE);
    while (old > 0 && !__atomic_compare_exchange_n(val, &old, old - 1, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    }
}

static TaskHandle_t chip8_task_handle = {};

static uint8_t touch_pressed = false;
static uint16_t touch_x = 0;
static uint16_t touch_y = 0;

static void lcd_draw_task(void* arg) {
    esp_err_t err;

    spi_bus_config_t spi_bus_config = {};
    spi_bus_config.mosi_io_num = GPIO_NUM_13;
    spi_bus_config.miso_io_num = GPIO_NUM_12;
    spi_bus_config.sclk_io_num = GPIO_NUM_14;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.data4_io_num = -1;
    spi_bus_config.data5_io_num = -1;
    spi_bus_config.data6_io_num = -1;
    spi_bus_config.data7_io_num = -1;
    spi_bus_config.data_io_default_level = 0;
    spi_bus_config.max_transfer_sz = LCD_PARALLEL_LINES * LCD_WIDTH * sizeof(uint32_t) + 8;
    spi_bus_config.dma_burst_size = 0;
    spi_bus_config.flags = 0;
    spi_bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1;
    spi_bus_config.intr_flags = 0;
    err = spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to initialize spi bus!");
        return;
    }

    spi_device_handle_t lcd = {};
    spi_device_interface_config_t lcd_interface_config = {};
    lcd_interface_config.mode = 0;
    lcd_interface_config.clock_speed_hz = 80 * 1000 * 1000;
    lcd_interface_config.spics_io_num = GPIO_NUM_15;
    lcd_interface_config.queue_size = 7;
    lcd_interface_config.pre_cb = lcd_spi_pre_transfer_callback;
    // lcd_interface_config.post_cb;
    err = spi_bus_add_device(SPI2_HOST, &lcd_interface_config, &lcd);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to add spi device (LCD)!");
        return;
    }
    spi_device_handle_t touch = {};
    spi_device_interface_config_t touch_interface_config = {};
    touch_interface_config.mode = 0;
    touch_interface_config.clock_speed_hz = 1 * 1000 * 1000;
    touch_interface_config.spics_io_num = GPIO_NUM_33;
    touch_interface_config.queue_size = 7;
    err = spi_bus_add_device(SPI2_HOST, &touch_interface_config, &touch);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_LOG_TAG, "Failed to add spi device (TOUCH)!");
        return;
    }

    {
        spi_device_acquire_bus(lcd, portMAX_DELAY);

        spi_device_release_bus(lcd);

        // Sleep out command
        spi_send_command(lcd, 0x11, false);
        vTaskDelay(120 / portTICK_PERIOD_MS);
        // Normal display mode command
        spi_send_command(lcd, 0x13, false);

        // Set color mode (RGB), and pixel address move (vertical display)
        spi_send_command(lcd, 0x36, false);
        spi_send_data(lcd, 0x00);

        // Ram control
        spi_send_command(lcd, 0xB0, false);
        spi_send_data(lcd, 0x00, 0xE0);

        // Set 16-bit per pixel
        spi_send_command(lcd, 0x3A, false);
        spi_send_data(lcd, 0x55);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // Porch settings
        spi_send_command(lcd, 0xB2, false);
        spi_send_data(lcd, sizeof(lcd_porch_settings_cmd), lcd_porch_settings_cmd);

        spi_send_command(lcd, 0x21, false);

        spi_send_command(lcd, 0xB7, false);
        spi_send_data(lcd, 0x55);

        spi_send_command(lcd, 0xBB, false);
        spi_send_data(lcd, 0x28);

        spi_send_command(lcd, 0xC0, false);
        spi_send_data(lcd, 0x2C);

        spi_send_command(lcd, 0xC2, false);
        spi_send_data(lcd, 0x01, 0xFF);

        spi_send_command(lcd, 0xC3, false);
        spi_send_data(lcd, 0x11);
        spi_send_command(lcd, 0xC4, false);
        spi_send_data(lcd, 0x20);

        spi_send_command(lcd, 0xC6, false);
        spi_send_data(lcd, 0x0F);

        spi_send_command(lcd, 0xD0, false);
        spi_send_data(lcd, 0xA4, 0xA1);

        spi_send_command(lcd, 0xE0, false);
        spi_send_data(lcd, sizeof(lcd_pos_gamma_ctrl_cmd), lcd_pos_gamma_ctrl_cmd);
        spi_send_command(lcd, 0xE1, false);
        spi_send_data(lcd, sizeof(lcd_neg_gamma_ctrl_cmd), lcd_neg_gamma_ctrl_cmd);

        spi_send_command(lcd, 0x11, false);
        vTaskDelay(120 / portTICK_PERIOD_MS);
        spi_send_command(lcd, 0x29, false);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    uint16_t* transfer_buffers[2];
    for (uint8_t i = 0; i < 2; ++i) {
        transfer_buffers[i] = (uint16_t*)spi_bus_dma_memory_alloc(
            SPI2_HOST, LCD_WIDTH * LCD_PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(transfer_buffers[i] != NULL);
    }

    int32_t transfer_buffer_index = 0;
    int32_t sending_buffer_index = -1;

    // TODO this has a resolution too low, maybe use some kind of busy wait (esp_cpu_get_cycle_count()),
    // configCPU_CLOCK_HZ
    static_assert(configTICK_RATE_HZ == 60, "The FreeRTOS frequency shold be set to 60Hz");
    TickType_t frequency = pdMS_TO_TICKS(1000) / 60;
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        BaseType_t was_delayed = xTaskDelayUntil(&last_wake_time, frequency);
        if (was_delayed) {
            ESP_LOGE(LCD_LOG_TAG, "Frame missed!");
            last_wake_time = xTaskGetTickCount();
        }

        atomic_decrement_if_positive(&chip8_delay_timer_register);
        atomic_decrement_if_positive(&chip8_sound_timer_register);

        {
            uint16_t x_raw = touch_read_value(touch, Touch_Value_Index::AXIS_X);
            uint16_t y_raw = touch_read_value(touch, Touch_Value_Index::AXIS_Y);

            if (x_raw >= 130 && x_raw <= 1930 && y_raw >= 210 && y_raw <= 1960) {
                touch_x = 239 - (((x_raw - 130) * 239) / (1920 - 130));
                touch_y = 319 - (((y_raw - 210) * 319) / (1960 - 210));
                if (!touch_pressed) {
                    int16_t x = (touch_x - 56) >> 5;
                    int16_t y = (touch_y - 160) >> 5;
                    if (x >= 0 && x < 4 && y >= 0 && y < 4) {
                        __atomic_store_n(&chip8_button_pressed, x + y * 4, __ATOMIC_RELEASE);
                        vTaskResume(chip8_task_handle);
                    }
                }
                touch_pressed = true;
            } else {
                if (touch_pressed) {
                    __atomic_store_n(&chip8_button_pressed, 0x10, __ATOMIC_RELEASE);
                }
                touch_pressed = false;
            }
        }

        {
            uint16_t* line = frame_buffer;
            for (uint16_t y = 0; y < LCD_HEIGHT; y++) {
                uint16_t* pixel = line;
                for (uint16_t x = 0; x < LCD_WIDTH; x++, ++pixel) {
                    *pixel = 0;
                }
                line += LCD_WIDTH;
            }

            line = frame_buffer + (23 * LCD_WIDTH);
            {
                uint16_t* pixel = line + 23;
                for (uint16_t x = 0; x < 194; x++) {
                    *pixel++ = 0b1000010000010000;
                }
                line += LCD_WIDTH;
            }
            for (uint16_t y = 0; y < 96; y++) {
                uint16_t* pixel = line + 23;
                *pixel++ = 0b1000010000010000;
                uint16_t chip8_y = y / 3;
                for (uint16_t x = 0; x < 192; x++) {
                    uint16_t chip8_x = x / 3;
                    *pixel++ = chip8_screen[chip8_x + chip8_y * 64] ? 0b0000011111100000 : 0b0000000000000000;
                }
                *pixel++ = 0b1000010000010000;
                line += LCD_WIDTH;
            }
            {
                uint16_t* pixel = line + 23;
                for (uint16_t x = 0; x < 194; x++) {
                    *pixel++ = 0b1000010000010000;
                }
            }

            uint16_t btn_x = chip8_button_pressed & 0b11;
            uint16_t btn_y = chip8_button_pressed >> 2;
            line = frame_buffer + (159 * LCD_WIDTH);
            for (uint16_t y = 31; y < 130 + 31; y++) {
                uint16_t* pixel = line + 55;
                uint16_t row = (y >> 5) - 1;
                uint16_t row_y = y & 0b11111;
                for (uint16_t x = 31; x < 130 + 31; x++, ++pixel) {
                    uint16_t row_x = x & 0b11111;
                    uint16_t col = (x >> 5) - 1;
                    if (row_y == 0 || row_y == 31 || row_x == 0 || row_x == 31) {
                        *pixel = 0b1000010000010000;
                    } else if (col == btn_x && row == btn_y) {
                        *pixel = 0b0000000000011111;
                    } else {
                        *pixel = 0b0000000000000000;
                    }
                }
                line += LCD_WIDTH;
            }
        }

        for (uint16_t n = 0; n < LCD_HEIGHT; n += LCD_PARALLEL_LINES) {
            uint16_t* transfer_buffer = transfer_buffers[transfer_buffer_index];

            uint16_t* line = transfer_buffer;
            uint16_t* src_line = frame_buffer + (n * LCD_WIDTH);
            for (uint16_t line_counter = 0; line_counter < LCD_PARALLEL_LINES; line_counter++) {
                uint16_t* pixel = line;
                uint16_t* src = src_line;
                for (uint16_t x = 0; x < LCD_WIDTH; x++, ++pixel) {
                    *pixel = __builtin_bswap16(*src++);
                }
                line += LCD_WIDTH;
                src_line += LCD_WIDTH;
            }

            if (sending_buffer_index != -1) {
                spi_wait_trans(lcd, 6);
            }
            sending_buffer_index = transfer_buffer_index;

            transfer_buffer_index++;
            if (transfer_buffer_index >= 2) {
                transfer_buffer_index = 0;
            }

            spi_send_command_async(lcd, &send_data_transaction[0], 0x2A);
            spi_send_data_async(lcd, &send_data_transaction[1], 0x00, 0x00, 0x00, 0xEF);

            uint16_t end_line = n + LCD_PARALLEL_LINES - 1;
            spi_send_command_async(lcd, &send_data_transaction[2], 0x2B);
            spi_send_data_async(lcd, &send_data_transaction[3], n >> 8, n & 0xFF, end_line >> 8, end_line & 0xFF);

            spi_send_command_async(lcd, &send_data_transaction[4], 0x2C);
            spi_send_data_async(lcd, &send_data_transaction[5], LCD_WIDTH * sizeof(uint16_t) * LCD_PARALLEL_LINES,
                                (uint8_t*)transfer_buffer);
        }
    }
}

static void lcd_backlight_task(void* arg) {
    uint32_t backlight_on = 1;
    // uint32_t before;
    // uint32_t after;
    // before = esp_cpu_get_cycle_count();
    // after = esp_cpu_get_cycle_count();
    gpio_set_level(LCD_BACKLIGHT_PIN, backlight_on);
    for (;;) {
        if (0 && button_pressed) {
            button_pressed = 0;
            backlight_on = !backlight_on;
        }

        if (backlight_on) {
            // gpio_set_level(LCD_BACKLIGHT_PIN, 0);
            // vTaskDelay(1);
            gpio_set_level(LCD_BACKLIGHT_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// uint16_t calData[5] = { 489, 3401, 318, 3501, 3 };
//   tft.setTouch(calData);

FORCE_INLINE_ATTR uint16_t chip8_load16(uint16_t addr) {
    uint16_t result = 0;
    if (addr <= 0xFFF) {
        result = (((uint16_t)chip8_ram[addr]) << 8) | ((uint16_t)chip8_ram[addr + 1]);
    }
    return result;
}

FORCE_INLINE_ATTR uint8_t chip8_load8(uint16_t addr) {
    uint8_t result = 0;
    if (addr <= 0xFFF) {
        result = chip8_ram[addr];
    }
    return result;
}

FORCE_INLINE_ATTR void chip8_store8(uint16_t addr, uint8_t value) {
    if (addr <= 0xFFF) {
        chip8_ram[addr] = value;
    }
}

static uint8_t chip8_button_map[16] = {
    0x1, 0x2, 0x3, 0xC, //
    0x4, 0x5, 0x6, 0xD, //
    0x7, 0x8, 0x9, 0xE, //
    0xA, 0x0, 0xB, 0xF, //
};

#define CHIP8_DEBUG 0
#if CHIP8_DEBUG
#define CHIP8_LOG(...) ESP_LOGI(CHIP8_LOG_TAG, __VA_ARGS__)
#else
#define CHIP8_LOG(...)
#endif

static void chip8_task(void* arg) {
    {
        // NOTE(Damiano): load rom
        uint16_t rom_len = sizeof(chip8_rom);
        for (uint16_t i = 0; i < rom_len; ++i) {
            chip8_ram[0x200 + i] = chip8_rom[i];
        }
    }

    {
        // NOTE(Damiano): Clear screen
        uint16_t* line = frame_buffer;
        for (uint16_t y = 0; y < LCD_HEIGHT; y++) {
            uint16_t* pixel = line;
            for (uint16_t x = 0; x < LCD_WIDTH; x++, ++pixel) {
                *pixel = 0;
            }
            line += LCD_WIDTH;
        }
    }

    uint8_t regs[16] = {};
    uint16_t reg_I = 0;
    uint16_t pc = 0x200;
    uint8_t sp = 0;
    uint16_t stack[16] = {};

#if CHIP8_DEBUG
    uint16_t running = false;
#endif
    for (;;) {
#if CHIP8_DEBUG
        if (button_pressed) {
            button_pressed = 0;
            running = true;
        }

        if (!running) {
            vTaskDelay(10);
            continue;
        }
        running = false;
#endif

#if CHIP8_DEBUG
        const uint16_t inst_count = 1;
#else
        const uint16_t inst_count = 1024;
#endif
        for (uint16_t frame_instruction_count = 0; frame_instruction_count < inst_count; ++frame_instruction_count) {
            uint16_t instruction = chip8_load16(pc);

            switch ((instruction >> 12) & 0xF) {
            case 0x0: {
                uint16_t low = instruction & 0xFF;
                if (low == 0xE0) {
                    // 00E0 - CLS
                    // TODO clear display
                    memset(chip8_screen, 0, 32 * 64);
                } else if (low == 0xEE) {
                    // 00EE - RET
                    assert(sp > 0 && sp <= 16);
                    CHIP8_LOG("RET");
                    sp--;
                    pc = stack[sp];
                } else {
                    // NOTE(Damiano): Ignore
                }
            } break;
            case 0x1: {
                // 1nnn - JP addr
                // NOTE(Damiano): Ignore the least significant bit to align the pc
                uint16_t addr = instruction & 0xFFE;
                CHIP8_LOG("JP 0x%04X", addr);
                pc = addr;
                continue;
            } break;
            case 0x2: {
                // 2nnn - CALL addr
                // NOTE(Damiano): Ignore the least significant bit to align the pc
                uint16_t addr = instruction & 0xFFE;
                assert(sp >= 0 && sp < 16);
                CHIP8_LOG("CALL 0x%04X", addr);
                stack[sp] = pc;
                sp++;
                pc = addr;
                continue;
            } break;
            case 0x3: {
                // 3xkk - SE Vx, byte
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                CHIP8_LOG("SE V%X(%u), %u", reg, regs[reg], value);
                if (regs[reg] == value) {
                    pc += 2;
                }
            } break;
            case 0x4: {
                // 4xkk - SNE Vx, byte
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                CHIP8_LOG("SNE V%X(%u), %u", reg, regs[reg], value);
                if (regs[reg] != value) {
                    pc += 2;
                }
            } break;
            case 0x5: {
                // 5xy0 - SE Vx, Vy
                assert((instruction & 0xF) == 0);
                uint8_t reg_l = (instruction >> 8) & 0xF;
                uint8_t reg_r = (instruction >> 4) & 0xF;
                CHIP8_LOG("SE V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                if (regs[reg_l] == regs[reg_r]) {
                    pc += 2;
                }
            } break;
            case 0x6: {
                // 6xkk - LD Vx, byte
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                CHIP8_LOG("LD V%X, %u", reg, value);
                regs[reg] = value;
            } break;
            case 0x7: {
                // 7xkk - ADD Vx, byte
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                CHIP8_LOG("ADD V%X(%u), %u", reg, regs[reg], value);
                regs[reg] += value;
            } break;
            case 0x8: {
                switch (instruction & 0xF) {
                case 0: {
                    // 8xy0 - LD Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("LD V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    regs[reg_l] = regs[reg_r];
                } break;
                case 1: {
                    // 8xy1 - OR Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("OR V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    regs[reg_l] |= regs[reg_r];
                } break;
                case 2: {
                    // 8xy2 - AND Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("AND V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    regs[reg_l] &= regs[reg_r];
                } break;
                case 3: {
                    // 8xy3 - XOR Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("XOR V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    regs[reg_l] ^= regs[reg_r];
                } break;
                case 4: {
                    // 8xy4 - ADD Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("ADD V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    // TODO(Damiano): save carry bit
                    regs[reg_l] += regs[reg_r];
                } break;
                case 5: {
                    // 8xy5 - SUB Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("SUB V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    // TODO(Damiano): save carry bit
                    regs[reg_l] -= regs[reg_r];
                } break;
                case 6: {
                    // 8xy6 - SHR Vx {, Vy}
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    CHIP8_LOG("SHR V%X(%u) {, Vy}", reg_l, regs[reg_l]);
                    // uint8_t reg_r = (instruction >> 4) & 0xF;
                    // TODO(Damiano): save carry bit
                    regs[reg_l] = regs[reg_l] >> 1;
                } break;
                case 7: {
                    // 8xy7 - SUBN Vx, Vy
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    uint8_t reg_r = (instruction >> 4) & 0xF;
                    CHIP8_LOG("SUBN V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                    // TODO(Damiano): save carry bit
                    regs[reg_l] = regs[reg_r] - regs[reg_l];
                } break;
                case 0xE: {
                    // 8xyE - SHL Vx {, Vy}
                    uint8_t reg_l = (instruction >> 8) & 0xF;
                    CHIP8_LOG("SHL V%X(%u) {, Vy}", reg_l, regs[reg_l]);
                    // uint8_t reg_r = (instruction >> 4) & 0xF;
                    // TODO(Damiano): save carry bit
                    regs[reg_l] = regs[reg_l] << 1;
                } break;
                default: {
                    assert(false);
                };
                }
            } break;
            case 0x9: {
                // 9xy0 - SNE Vx, Vy
                assert((instruction & 0xF) == 0);
                uint8_t reg_l = (instruction >> 8) & 0xF;
                uint8_t reg_r = (instruction >> 4) & 0xF;
                CHIP8_LOG("SNE V%X(%u), V%X(%u)", reg_l, regs[reg_l], reg_r, regs[reg_r]);
                if (regs[reg_l] != regs[reg_r]) {
                    pc += 2;
                }
            } break;
            case 0xA: {
                // Annn - LD I, addr
                uint16_t addr = instruction & 0xFFF;
                CHIP8_LOG("LD I, 0x%04X", addr);
                reg_I = addr;
            } break;
            case 0xB: {
                // Bnnn - JP V0, addr
                uint16_t addr = (instruction & 0xFFF) + regs[0];
                CHIP8_LOG("JP V0(%u), 0x%04X", regs[0], addr);
                // NOTE(Damiano): Ignore the least significant bit to align the pc
                pc = addr & 0xFFFE;
                continue;
            } break;
            case 0xC: {
                // Cxkk - RND Vx, byte
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                uint8_t rand_byte = prng_next() & 0xFF;
                CHIP8_LOG("RND V%x(%u), %u. (%u)", reg, regs[reg], value, rand_byte);
                regs[reg] = value & rand_byte;
            } break;
            case 0xD: {
                // Dxyn - DRW Vx, Vy, nibble
                uint8_t value = instruction & 0xF;
                uint8_t reg_r = (instruction >> 4) & 0xF;
                uint8_t reg_l = (instruction >> 8) & 0xF;

                CHIP8_LOG("DRW V%x(%u), V%x(%u), %u. I=0x%04x", reg_l, regs[reg_l], reg_r, regs[reg_r], value, reg_I);

                uint8_t y = regs[reg_r];
                for (uint16_t line_index = 0; line_index < value; ++line_index, ++y) {
                    uint8_t sprite_line = chip8_load8(reg_I + line_index);
                    uint8_t x = regs[reg_l];
                    while (y >= 32) {
                        y -= 32;
                    }
                    for (int16_t p = 7; p >= 0; --p, ++x) {
                        uint8_t pixel_value = (sprite_line >> p) & 1;
                        while (x >= 64) {
                            x -= 64;
                        }
                        chip8_screen[x + y * 64] ^= pixel_value;
                    }
                }
            } break;
            case 0xE: {
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                uint8_t button_value = __atomic_load_n(&chip8_button_pressed, __ATOMIC_ACQUIRE);
                if (value == 0x9E) {
                    // Ex9E - SKP Vx
                    CHIP8_LOG("SKP V%x(%u). (%u)", reg, regs[reg], chip8_button_map[button_value]);
                    if (button_value < 0x10 && regs[reg] == chip8_button_map[button_value]) {
                        pc += 2;
                    }
                } else if (value == 0xA1) {
                    // ExA1 - SKNP Vx
                    CHIP8_LOG("SKNP V%x(%u). (%u)", reg, regs[reg], chip8_button_map[button_value]);
                    if (button_value < 0x10 && regs[reg] != chip8_button_map[button_value]) {
                        pc += 2;
                    }
                } else {
                    // assert(false);
                }
            } break;
            case 0xF: {
                uint8_t value = instruction & 0xFF;
                uint8_t reg = (instruction >> 8) & 0xF;
                switch (value) {
                case 0x07: {
                    // Fx07 - LD Vx, DT
                    regs[reg] = __atomic_load_n(&chip8_delay_timer_register, __ATOMIC_ACQUIRE);
                    CHIP8_LOG("LD V%x, DT. (%u)", reg, regs[reg]);
                } break;
                case 0x0A: {
                    // Fx0A - LD Vx, K
                    uint8_t button_value = 0x10;
                    for (;;) {
                        vTaskSuspend(NULL);
                        button_value = __atomic_load_n(&chip8_button_pressed, __ATOMIC_ACQUIRE);
                        if (button_value < 0x10) {
                            break;
                        }
                        vTaskDelay(1);
                    }
                    regs[reg] = chip8_button_map[button_value];
                    CHIP8_LOG("LD V%X, K(%u)", reg, regs[reg]);
                } break;
                case 0x15: {
                    // Fx15 - LD DT, Vx
                    CHIP8_LOG("LD DT, V%x(%u)", reg, regs[reg]);
                    __atomic_store_n(&chip8_delay_timer_register, regs[reg], __ATOMIC_RELEASE);
                } break;
                case 0x18: {
                    // Fx18 - LD ST, Vx
                    CHIP8_LOG("LD ST, V%x(%u)", reg, regs[reg]);
                    __atomic_store_n(&chip8_sound_timer_register, regs[reg], __ATOMIC_RELEASE);
                } break;
                case 0x1E: {
                    // Fx1E - ADD I, Vx
                    CHIP8_LOG("ADD I(0x%04X), V%x(%u)", reg_I, reg, regs[reg]);
                    reg_I += regs[reg];
                } break;
                case 0x29: {
                    // Fx29 - LD F, Vx
                    assert(regs[reg] < 0x10);
                    reg_I = regs[reg] * 5;
                    CHIP8_LOG("LD F, V%x(%u). (0x%04X)", reg, regs[reg], reg_I);
                } break;
                case 0x33: {
                    // Fx33 - LD B, Vx
                    uint8_t value = regs[reg];
                    uint8_t units = value % 10;
                    value = value / 10;
                    uint8_t tens = value % 10;
                    value = value / 10;
                    uint8_t h = value;
                    chip8_store8(reg_I + 0, h);
                    chip8_store8(reg_I + 1, tens);
                    chip8_store8(reg_I + 2, units);
                    CHIP8_LOG("LD B, V%x(%u). (0x%04X)", reg, regs[reg], reg_I);
                } break;
                case 0x55: {
                    // Fx55 - LD [I], Vx
                    CHIP8_LOG("LD [I], V%x. (0x%04X)", reg, reg_I);
                    for (uint16_t i = 0; i <= reg; ++i) {
                        chip8_store8(reg_I + i, regs[i]);
                    }
                } break;
                case 0x65: {
                    // Fx65 - LD Vx, [I]
                    CHIP8_LOG("LD V%x, [I]. (0x%04X)", reg, reg_I);
                    for (uint16_t i = 0; i <= reg; ++i) {
                        regs[i] = chip8_load8(reg_I + i);
                    }
                } break;
                default: {
                    assert(false);
                } break;
                }
            } break;
            default: {
                assert(false);
            } break;
            }

            pc += 2;
        }

        vTaskDelay(1);
    }
}

extern "C" void app_main(void) {
    prng_state[0] = esp_random();
    prng_state[1] = esp_random();
    prng_state[2] = esp_random();
    prng_state[3] = esp_random();

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LCD_BACKLIGHT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LCD_DC_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    gpio_config(&io_conf);

    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*)0);

    xTaskCreatePinnedToCore(chip8_task, "chip8_task", 2048, NULL, 9, &chip8_task_handle, 0);
    xTaskCreatePinnedToCore(lcd_draw_task, "lcd_draw_task", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(lcd_backlight_task, "lcd_backlight_task", 2048, NULL, 10, NULL, 0);
}
