#include <Adafruit_GFX.h>
#include <LCD_UC1609.h>

#include "graphics.h"

// THIS EXAMPLE UTILIZES SRAM
// SRAM CHIP SELECT MUST BE CONNECTED ON ANY PIN < 8 ON AVR
LCD_UC1609  display(10, 9, 8, 7); // DC, RST, LCD_CS, SRAM_CS

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

  static long iteration = 0; // iteration counter

  // animated character options
  const uint8_t RUNNER_IDLE = 0; // example
  const uint8_t RUNNER_RUNNING = 1; // only running used
  uint8_t currentMove = RUNNER_RUNNING;

  // draw animated character
  const uint8_t bitmap_side = 16; // side can be used for both width and height as the bitmap is a square
  const uint8_t bitmap_y = 36; // y offset
  const uint8_t* currentBitmap = (const uint8_t*) pgm_read_ptr (&anim_runner[currentMove][(iteration >> 2) % RUNNER_FRAMES]); // RUNNER_FRAMES defined at graphics.h
  display.drawBitmap((iteration % (main_window.width + bitmap_side)) - bitmap_side, bitmap_y, currentBitmap, bitmap_side, bitmap_side, WHITE);

  // if the guy is behind tree -> erase him
  display.drawFastVLine(171, bitmap_y, bitmap_side, BLACK);
  display.drawFastVLine(172, bitmap_y, bitmap_side, BLACK);
  display.drawFastVLine(173, bitmap_y, bitmap_side, BLACK);
  display.drawFastVLine(174, bitmap_y, bitmap_side, BLACK);

  // draws background image with ground, tree and four static stars
  display.drawBitmap(0, 0, background_tree_192x64, 192, 64, WHITE);

  ////////////////////////////////////////////////////////////////
  // draw star sky
  const uint8_t STAR_COUNT = 8; // how many animated stars are drawn on screen at one time
  
  // stars array index naming
  const uint8_t STAR_X = 0;
  const uint8_t STAR_Y = 1;
  const uint8_t STAR_UPDATE_INTERVAL = 2;
  const uint8_t STAR_CURRENT_FRAME = 3;

  static uint8_t stars[STAR_COUNT][4];
  // init stars at first iteration
  if (iteration == 0) {
    for (uint8_t s = 0; s < STAR_COUNT; s++) {
      stars[s][STAR_X] = random(main_window.width); // x offset
      stars[s][STAR_Y] = random(24); // y offset
      stars[s][STAR_UPDATE_INTERVAL] = random(10) + 1; // interval -> how many frames to skip to advance the animation
      stars[s][STAR_CURRENT_FRAME] = random(STAR_FRAMES); // STAR_FRAMES defined at graphics.h
    }
  }
  
  // cycle trough array, advance animation and draw
  for (uint8_t s = 0; s < STAR_COUNT; s++) {
    if (iteration % stars[s][STAR_UPDATE_INTERVAL] == 0) stars[s][STAR_CURRENT_FRAME]++; // increase current frame
    // reinit after last frame
    if (stars[s][STAR_CURRENT_FRAME] == STAR_FRAMES) {
      stars[s][STAR_X] = random(main_window.width);
      stars[s][STAR_Y] = random(24);
      stars[s][STAR_UPDATE_INTERVAL] = random(10) + 1;
      stars[s][STAR_CURRENT_FRAME] = 0;
    }
    display.drawBitmap(stars[s][STAR_X], stars[s][STAR_Y], (const uint8_t*) pgm_read_ptr (&anim_star[stars[s][STAR_CURRENT_FRAME]]), 8, 8, WHITE);
  }

  ////////////////////////////////////////////////////////////////
  // draw fps value and label
  static long last_iteration = 0;
  static uint8_t fps;
  static long last_millis = 0;

  if (millis() - last_millis >= 1000) {
    fps = iteration - last_iteration;
    last_iteration = iteration;
    last_millis = millis();
  }
  display.setTextSize(1); // not required
  display.setTextColor(WHITE);
  display.setCursor(6, 56);
  display.print(fps);
  display.print(F("fps"));

  // display selected buffer content
  display.display();
  
  iteration++; // increase the counter
} // ----------------------> LOOP END
