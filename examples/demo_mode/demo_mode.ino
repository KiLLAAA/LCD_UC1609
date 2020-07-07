#include <Adafruit_GFX.h>
#include <LCD_UC1609.h>

LCD_UC1609  display(10, 9, 8); // DC, RST, CS

void setup() {
  display.begin(); // initialize the LCD
  display.clearDisplay(0x55); // this version with single param writes directly to the display, 0x55 (B01010101) makes horizontal strips
  delay(1000);
}

void loop() {
  static long frame = 0; // frame = iteration counter

  uint8_t  bitmapBuffer[512];  // 64x64 pixels take 512 bytes

  ////////////////////////////////////////////////////////////////
  // buffer helper structures

  AdvancedBuffer left_window;
  left_window.bitmap = (uint8_t*) &bitmapBuffer;
  left_window.width = 64;
  left_window.height = 64;
  left_window.x = 0; // offset on display

  AdvancedBuffer mid_window;
  mid_window.bitmap = (uint8_t*) &bitmapBuffer;
  mid_window.width = 64;
  mid_window.height = 64;
  mid_window.x = 64; // offset on display

  AdvancedBuffer right_window;
  right_window.bitmap = (uint8_t*) &bitmapBuffer;
  right_window.width = 64;
  right_window.height = 64;
  right_window.x = 128; // offset on display
  ////////////////////////////////////////////////////////////////

  display_demo_mode(&left_window, frame);

  display_scope(&mid_window, frame);

  if (frame % 10 == 0) {
    display_upscaled_data(&right_window, frame);
  }

  frame++; // increase the counter
} // --------------------------------> LOOP END

////////////////////////////////////////////////////////////////
// DEMO MODE
void display_demo_mode(AdvancedBuffer* target, long iteration) {
  display.selectedBuffer = target; // set target buffer object
  display.clearDisplay();
  // display.setTextSize(1); // not required until other func changes it
  display.setTextColor(WHITE);
  display.setCursor(6, 20 + ((get_sin(( iteration ) % 256)) / 10));
  display.print(F("DEMO MODE"));

  // static values to count fps
  static long last_iteration = 0;
  static uint8_t fps;
  static long last_millis = 0;

  if (millis() - last_millis >= 1000) {
    fps = iteration - last_iteration;
    last_iteration = iteration;
    last_millis = millis();
  }
  display.setCursor(6, 48); // print fps value and its label
  display.print(fps);
  display.print(F("fps"));

  // flip display inversion every n frames
  const uint16_t flip_at = 1000; // sets the number of iteration where display inversion gets flipped
  static boolean invert_state = 1; // 1 is set to simplify code, it is flipped at frame 0 to zero
  if (iteration % flip_at == 0) {
    invert_state = !invert_state; // flip
    display.invertDisplay (invert_state);
  }
  // draws text label when display is inverted
  if (invert_state && (iteration % 100 > 39)) { // blink on/off time 40%
    display.setCursor(6, 56);
    display.print(F("INVERSE!"));
  }
  // draws the loading bar representing iterations to flip the invert_state
  else if (!invert_state) {
    display.drawFastHLine(0, 58, (64.0 / flip_at) * (iteration % flip_at), WHITE);
  }
  display.display();
}

////////////////////////////////////////////////////////////////
// SINE SCOPE
void display_scope(AdvancedBuffer* target, long iteration) {
  display.selectedBuffer = target; // set target buffer object
  display.clearDisplay();
  display.drawFastHLine(0, 32 + (get_sin(( iteration << 1) % 256)) / 4, 64, WHITE);

  for (uint8_t m = 0; m < 64; m++) {
    display.drawPixel(m, 32 + (get_sin( ((iteration << 2 ) + (m << 2)) % 256)) / 10, WHITE);
    display.drawPixel(m, 32 + (get_sin( ((iteration << 2 ) + m) % 256)) / 10, WHITE);
  }
  display.display();
}

////////////////////////////////////////////////////////////////
// 8x8 to 64x64 FAST UPSCALE WITH NO BOUNDARY CHECKS
void display_upscaled_data(AdvancedBuffer* target, long iteration) {
  display.selectedBuffer = target; // set target buffer object
  display.clearDisplay();
  const uint8_t array_size = 8; // 8 bytes -> 8x8 pixels
  uint8_t randomArray[array_size]; // make array
  // get random values
  for (uint8_t i = 0; i < array_size; i++) {
    randomArray[i] = (random(256)); // 0-255
    //randomArray[i] = (iteration % 256); // TESTING 0-255
  }
  for (uint8_t line = 0; line < array_size; line++) {
    uint8_t bits = randomArray[line]; // read byte
    for (uint8_t xx = 0; xx < 8; xx++) { // for pixels in byte
      if ((bits >> xx) & 1) { // test pixel col
        // if bit is set -> memset 8 bytes to 0xff
        memset (display.selectedBuffer->bitmap + ((display.selectedBuffer->width * line) + (8 * xx)), 0xff, 8); // offset in target: (n-th bit * 8 Bytes) + (line * width Bytes per line in selectedBuffer bitmap)
      }
    }
  }
  display.display();
}

////////////////////////////////////////////////////////////////
// PRECOMPUTED SINE ARRAY AND GET FUNCTION
const int8_t sintable[256] PROGMEM = {0, 3, 6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 107, 109, 111, 112, 113, 115, 116, 117, 118, 120, 121, 122, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127, 127, 127, 127, 126, 126, 126, 125, 125, 124, 123, 122, 122, 121, 120, 118, 117, 116, 115, 113, 112, 111, 109, 107, 106, 104, 102, 100, 98, 96, 94, 92, 90, 88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 49, 46, 43, 40, 37, 34, 31, 28, 25, 22, 19, 16, 12, 9, 6, 3, 0, -3, -6, -9, -12, -16, -19, -22, -25, -28, -31, -34, -37, -40, -43, -46, -49, -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83, -85, -88, -90, -92, -94, -96, -98, -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116, -117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -122, -122, -121, -120, -118, -117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100, -98, -96, -94, -92, -90, -88, -85, -83, -81, -78, -76, -73, -71, -68, -65, -63, -60, -57, -54, -51, -49, -46, -43, -40, -37, -34, -31, -28, -25, -22, -19, -16, -12, -9, -6, -3};
int8_t get_sin(uint8_t angle) {
  return pgm_read_byte(&sintable[angle]);
}
