#ifndef _LCD_UC1609_H_
#define _LCD_UC1609_H_

//#include "display_print.h" // TODO:

#include "Adafruit_GFX.h"
#include <SPI.h>

////////////////////////////////////////////////////////////////
// Advanced buffer sructure
struct AdvancedBuffer
{
  uint8_t* bitmap; // pointer to buffer
  uint16_t width;  // bitmap x size
  uint16_t height; // bitmap y size
  int16_t x; // x offset at screen
  int16_t y; // y offset at scene or screen if not used to scene
  uint8_t type; // resolves both pixel order and place of data ext. SRAM or onchip
  uint32_t address; // address at external SRAM
}; // 14 Bytes per object (at least)
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Display Resolution
//
// AliExpress 19264-05 v3 LCD display
const uint8_t LCD_WIDTH = 192, LCD_HEIGHT = 64;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compatibility with OLEDs code
#define BLACK 0
#define WHITE 1
#define INVERSE 2

// COMMANDS compatible with UC1608, UC1609, (MAYBE UC1606, UC1611 PARTIALLY)
// READ OPERATIONS
#define UC1609_GET_STATUS B00000001 // TODO:(NOT TESTED) UC1608 datasheet page 10

// WRITE OPERATIONS
#define UC160X_SYS_RESET B11100010 // system reset -> 0x0e2 (5ms)
#define UC160X_SET_POWER_CONTROL B00101111 // power control, bit 1,2: PC2, PC1 - internal charge pump, bit 0: PC0: cap load
#define UC160X_SET_DISPLAY_ENABLE B10101110 // display enable.. 1 bit (lowest)
#define UC160X_SET_MR_TC B00100011 // set mux rate and temperature compensation bit 2: MR, bit 0,1: TC0, TC1
#define UC160X_SET_AC B10001000 // set RAM address control -> NOT USED
#define UC160X_SET_MAP_CONTROL B11000000 // set map control  bit 3: MY, bit 2: MX, bit 1 unset, bit 0: MSF

#define UC160X_SET_BR B11101000 // 2bit // set LCD bias ratio / Set BR -> BR1 BR0 / default 10b=12
#define UC160X_SET_GN_PM B10000001 // set gain and potentiometer(double-byte command) / default GN=3, PM=0
#define UC1608_DEFAULT_GN_PM B11000000 // default value for GN PM bits -> GN1 GN0 PM5 PM4 PM3 PM2 PM1 PM0 <-- UC1608 datasheet Page 9
#define UC1608_LOW_GN_PM B01000000 // <-- FOR TESTING
#define UC1606_DEFAULT_GN_PM B00010000 // <-- FOR TESTING
#define UC1608_U8_GN_PM B00010100 // default UGLIB <-- FOR TESTING

#define UC1609_SET_ALL_PIXEL_ON B10100100 // 1bit (lowest) // turn on all pixels / normal display (does not affect LCD memory)
#define UC1609_SET_INVERSE_DISPLAY B10100110 // 1bit (lowest) // set inverse display

#define UC1609_SET_COL_LSB B00000000 // + add lower 4 bits of data masked
#define UC1609_SET_COL_MSB B00010000 // + add higher 4 bits of data shifted
#define UC1609_SET_PAGE B10110000 // + add 3 bits -> 0-7 page
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LCD_UC1609 : public Adafruit_GFX {
  public:
    LCD_UC1609(uint8_t _dc, uint8_t _rst, uint8_t _cs);
    ~LCD_UC1609() {};

#ifndef __AVR__
#warning USING DIGITALWRITE
#endif

#ifdef USE_ADVANCED_BUFFERING
    AdvancedBuffer* selectedBuffer;
#else
    uint8_t* buffer;
    uint8_t bufferWidth;
    uint8_t bufferHeight;
#endif
    void begin();
    inline void uc1609_data (uint8_t d);
    void uc1609_command(uint8_t command, uint8_t value);

    void initDisplay();
    void displayOn(uint8_t i);

    void clearDisplay(void);
    void clearDisplay(uint8_t pixels);
    void clearDisplay(uint8_t pixels, uint8_t mdelay);

    void invertDisplay(uint8_t b);
    void allPixelOn(uint8_t b);
    void display(void);
    /*
        // TODO: HANDLE TEXT INTERNALLY
        // TODO: custom text class for other code pages
        friend class DisplayText;
        uint16_t textCursorX = 0;
        uint16_t textCursorY = 0;
      #if ARDUINO >= 100
        //using Print::write; // pull in write(str) and write(buf, size) from Print
        //virtual size_t write(const uint8_t *buffer, size_t size)override;
        //virtual size_t write(const uint8_t character)override;
      #else
        //virtual void write(const uint8_t *buffer, size_t size)override;
        //virtual void write(const uint8_t character)override;
      #endif
    */
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) override;

    void displayBuffer(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t* data);
    void displayBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t* data);

    // TODO: FIX overrides, functions removed because horizontal line written directly LCD to memory ovewrites 8 pixels tall rect
    //void drawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint16_t color);  //virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    //void drawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint16_t color);  //virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

  private:
    // x
    uint8_t dc;
    uint8_t cs;
    uint8_t rst;
    void select();
    void deselect();
    void hardwareReset();
};

#endif
