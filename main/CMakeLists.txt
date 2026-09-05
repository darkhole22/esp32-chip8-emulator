idf_component_register(SRCS "chip8.cpp"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_timer
                       INCLUDE_DIRS "")
