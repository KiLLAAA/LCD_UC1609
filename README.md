# UC1609C 19264-05 v3 Library
![Image of 19264-05 v3](https://raw.githubusercontent.com/KiLLAAA/LCD_UC1609/master/images/19264-05_v3.jpg)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

Written by Lukas Vyhnalek aka KiLLA.<br/>
BSD license, read license.txt for more information.<br/>
All text above must be included in any redistribution.<br/>

### FEATURES:
- works with 19264-05 v3 display from AliExpress, be cautious about 3.3V and 5V versions
- compatible with [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - sketches using other common displays like OLED SSD1306 can be easily ported
- mirroring, inverse display and other features supported
- can work with none, single or multiple buffers
- buffers can be placed in external SPI SRAM!

### SUPPORTED HARDWARE
| MCU | State |
| --- | --- |
| ATmega168 | ok |
| ATmega328 | ok |
| STM32F103C | TODO |
| ESP8266 | TODO |
| ESP32 | TODO |

| SRAM | Size | State |
| --- | --- | --- |
| 23LC256 | 32k | ok |
| 23LC512 | 64k | ok |
| 23LC1024 | 128k | TODO |
| IP12B256 | 32k | ok |
| IP12B512 | 64k | ok |
| ESP-PSRAM64H | 8192K | TODO |

### LIMITATIONS:
- the display chip select "CS" pin must be on PORTB on AVR - pins > D8
- the SRAM chip select pin must be on PORTD on AVR - pins < D8
- the backlight handling is left to user, for testing purposes it is enough to put 1k resistor from "VDD" pin to "A" pin and a wire from "GND" to "K" pin
- tested with old Adafruit GFX [1.1.5](https://github.com/adafruit/Adafruit-GFX-Library/tree/v1.1.5), may work up to [1.2.2](https://github.com/adafruit/Adafruit-GFX-Library/tree/1.2.2), newer versions **are not tested**
- any buffer can be directly sent to display memory
- without any buffer the use is limited to display progmem bitmaps
- both the buffers and progmem bitmaps displayed directly have offset in Y axis aligned by 8 pixels, e.g. 0-8 -> 0, 8-16 -> 8 to avoid bit shifting whole bitmap
- the Adafruit GFX screen canvas is set at compile time so it is always set to display dimensions, even if the advanced buffer is set smaller, but it does not limit the usage of anything except for text wrapping which is done at the screen edges presented by Adafruit GFX, offscreen pixels are handled by both the GFX and `drawPixel()` in this library

### BUFFERS, BITMAPS AND GFX:
- the simple buffer is handled similar way as in the SSD1306 OLED library and contains whole screen region - comment out line containing #define USE_ADVANCED_BUFFERING
- the advanced buffer structure carries more properties - the screen can be divided into more buffers with their own offsets!
- both types let almost whole memory free to let user decide the scope of the buffer!
- horizontally formatted bitmaps can be displayed via buffer and GFX's function `drawBitmap()`
- vertiacally formatted bitmaps can be displayed directly with function `displayBitmap()`
- this device uses "vertical" byte addressing, the library's functions `displayBitmap()` and `displayBuffer()` are using this format for maximum speed, buffered rotation takes too much cycles, it is better to have same data twice, or rotate to buffer only by need

###### TODO:
- [x] make work with external SPI SRAM
- [ ] support STM32F1, esp8266
- [ ] test compatibility with more recent versions of Adafruit GFX
- [ ] add more examples
- [ ] add basic connection scheme
- [ ] add in-depth docs

###### OTHER:
- most of values were obtained from similar controller's UC1608 datasheet
