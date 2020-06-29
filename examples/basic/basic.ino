#include <Adafruit_GFX.h>
#include "LCD_UC1609.h"

LCD_UC1609  display(10, 9, 8); // DC, RST, CS

void setup() {
  display.begin(); // initialize the LCD
  display.clearDisplay(0x55); // this version with single param writes directly to the display, 0x55 (B01010101) makes horizontal strips
}

void loop() {
  display.clearDisplay(0); // clear display memory
  // create buffer with vertically addressed 16x16 bitmap data
  uint8_t thug_life_v [] = {
    0x40, 0xD0, 0x4C, 0xC2, 0x42, 0xC1, 0xC1, 0x41, 0x41, 0xC1, 0x41, 0xC2, 0x42, 0xCC, 0xD8, 0x40,
    0x06, 0x19, 0x23, 0x42, 0x43, 0x8A, 0x91, 0x90, 0x90, 0x91, 0x8B, 0x42, 0x43, 0x22, 0x19, 0x06,
  };
  // draw the bitmap above directly into the display memory to a random position
  for (uint8_t m = 0; m < 64; m++) {
    display.displayBuffer(random(192), random(64), 16, 16, (uint8_t*)thug_life_v);
    delay(100);
  }
  delay(1000);
} // ----------------------> LOOP END
