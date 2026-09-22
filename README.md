# Esp32 Chip8 Emulator

This is an implementation of a [chip8](https://en.wikipedia.org/wiki/CHIP-8) interpreter for [this](https://docs.freenove.com/projects/fnk0103/en/latest/fnk0103/codes/tutorial/Freenove_ESP32_Display.html) esp32 dev-kit.

## Notes

- The code is written using the espressif [esp-idf](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/index.html) development tools.
- The code is written in C++ (mainly for convenance eg. function overloading, constexpr etc), but it could be easily ported to C.
- This project is a personal exercise, so no library (outside the basic one from epressif) has been used.
- The display driver, and the touch driver were written by me. I did not yet validated them thoroughly.
- For now the display is fully updated every frame, this is not optimal considering that Chip8 only updates small portion of the scree at any time and that the rest of the UI is fairly static. 
  - This will probably not be fixed because I am interested in optimizing the display driver to render the full frame at 60Hz (the maximum allowed by the hardware).

## TODO
The project is not yet finished
- [ ] Add audio support
