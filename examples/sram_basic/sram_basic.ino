#include <Adafruit_GFX.h>
#include <LCD_UC1609.h>

// THIS EXAMPLE UTILIZES SRAM
// SRAM CHIP SELECT MUST BE CONNECTED ON ANY PIN < 8 ON AVR
LCD_UC1609  display(10, 9, 8, 7); // DC, RST, LCD_CS, SRAM_CS

// horizontally addressed 16x16 bitmap stored in PGM
const uint8_t thug_life_h [] PROGMEM = {
  0x07, 0xE0, 0x18, 0x18, 0x20, 0x04, 0x20, 0x06, 0x40, 0x02, 0x00, 0x00, 0xFF, 0xFF, 0x56, 0x56,
  0x6A, 0x6A, 0xBC, 0x3D, 0x80, 0x01, 0x44, 0x22, 0x43, 0xC2, 0x20, 0x04, 0x18, 0x18, 0x07, 0xE0
};

void setup() {
  display.begin(); // initialize the LCD
  display.clearDisplay(0); // clear display memory
}

void loop() {
  ////////////////////////////////////////////////////////////////
  AdvancedBuffer main_window;
  main_window.width = display.width(); // 192
  main_window.height = display.height(); // 64
  main_window.type = TYPE_SRAM_V; // use SRAM

  display.selectedBuffer = &main_window;
  //////////////////////////////// yes, it can be that simple!

  display.clearDisplay(); // clear selected buffer

  const uint8_t bitmap_count = 64; // how many bitmaps are drawn on screen at one time
  const uint8_t bitmap_side = 16; // side can be used for both width and height as the bitmap is a square

  for (uint8_t m = 0; m < bitmap_count; m++) {
    display.drawBitmap(random(main_window.width - bitmap_side), random(main_window.height - bitmap_side), thug_life_h, bitmap_side, bitmap_side, WHITE);
  }

  // draw uptime text
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(48, 0); // top, centered -> ( screen width - text width / 2 )
  long uptime = millis();
  char secs[3]; sprintf(secs, "%02d", (uptime / 1000L) % 60);
  char mins[3]; sprintf(mins, "%02d", (uptime / 60000L) % 60);
  char hours[3]; sprintf(hours, "%02d", (uptime / 3600000L) % 24);
  display.print(hours); display.print(F("."));  display.print(mins);  display.print(F(".")); display.print(secs);

  // display selected buffer content
  display.display();
} // ----------------------> LOOP END
