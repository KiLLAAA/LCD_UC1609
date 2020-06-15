# UC1609C 19264-05 v3 Library
![Image of Yaktocat](https://github.com/KiLLAAA/UC1609/images/19264-05_v3.jpg)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

Written by Lukas Vyhnalek aka KiLLA.
BSD license, read license.txt for more information.
All text above must be included in any redistribution.

### FEATURES:
- it works with 19264-05 v3 display from AliExpress, be cautious about 3.3V and 5V versions
- compatible with [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - sketches using other common displays like OLED SSD1306 can be easily ported
- mirroring, inverse display and other features supported
- can work with none, single or multiple buffers

### LIMITATIONS:
- the backlight handling is left to user, for testing purposes it is enough to put 1k resistor from "VDD" pin to "A" pin and a wire from "GND" to "K" pin
- tested with old [1.1.5](https://github.com/adafruit/Adafruit-GFX-Library/tree/v1.1.5), may work up to https://github.com/adafruit/Adafruit-GFX-Library/tree/1.2.2, newer versions **are not tested**
- without any buffer the use is limited to display progmem bitmaps
- both the buffers and progmem bitmaps displayed directly have offset in Y axis aligned by 8 pixels, e.g. 0-8 -> 0, 8-16 -> 8 to avoid bit shifting whole bitmap
- the adafruitGFX screen canvas is set at compile time so it is always set to display dimensions, even if the advanced buffer is set smaller, but it does not limit the usage of anything except for text wrapping which is done at the screen edges presented by GfX, offscreen pixels are handled by both the GFX and `drawPixel()` in this library

### BUFFERS, BITMAPS AND GFX:
- the simple buffer is handled similar way as in the SSD1306 OLED library and contains whole screen region
- the advanced buffer structure carries more properties - the screen can be divided into more buffers with their own offsets!
- both types let almost whole memory free to let user decide the scope of the buffer!
- horizontally formatted bitmaps can be displayed via buffer and GFX's function `drawBitmap()`
- vertiacally formatted bitmaps can be displayed directly with function `displayBitmap()`
- this device uses "vertical" byte addressing, the library's functions `displayBitmap()` and `displayBuffer()` are using this format for maximum speed, buffered rotation takes too much cycles, it is better to have same data twice, or rotate to buffer only by need

###### TODO:
- [ ] make work with external SPI SRAM
- [ ] support STM32F1, esp8266
- [ ] test compatibility with more recent versions of GFX

###### OTHER:
- most of values were obtained from similar controller's UC1608 datasheet, however the default value of PM register is not still absolutely certain
