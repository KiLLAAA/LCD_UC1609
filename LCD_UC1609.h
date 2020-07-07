#ifndef _LCD_UC1609_H_
#define _LCD_UC1609_H_

//#include "display_print.h" // TODO:

#include "Adafruit_GFX.h"
#include <SPI.h>

////////////////////////////////////////////////////////////////
// OPTIONS
#define USE_ADVANCED_BUFFERING // <--------------------------------<<
// #define USE_SRAM_24BIT_ADDRESS // use for memories 128kB and up, comment out for 64kB and less devices!
// #define USE_PSRAM64_INIT // works without it -> tested, use in case of troubles
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// SRAM
#define SRAM_CMD_RDMR 0x05 // read mode register
#define SRAM_CMD_WRMR 0x01 // write mode register
#define SRAM_CMD_READ 0x03 // read data from memory array beginning at selected address
#define SRAM_CMD_WRITE 0x02 // write data to memory array beginning at selected address

#define SRAM_BYTE_MODE 0x00 //
#define SRAM_PAGE_MODE 0x80 //
#define SRAM_SEQUENTIAL_MODE 0x40 //
#define SRAM_PSRAM64_RESET_ENABLE 0x66 //
#define SRAM_PSRAM64_DEVICE_RESET 0x99 //

// SPISettings SRAM_settings(8000000, MSBFIRST, SPI_MODE0); // 
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Advanced buffer sructure
#define TYPE_INTERNAL_V B00000000
#define TYPE_SRAM_V B00010000

struct AdvancedBuffer
{
  uint8_t* bitmap; // pointer to buffer
  uint16_t width;  // bitmap x size
  uint16_t height; // bitmap y size
  int16_t x = 0; // x offset at screen
  int16_t y = 0; // y offset at scene or screen if not used to scene
  uint8_t type = TYPE_INTERNAL_V; // resolves both pixel order and place of data ext. SRAM or onchip
  uint32_t address = 0; // address at external SRAM
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

// COMMANDS compatible with UC1608, UC1609, (MAYBE UC1606, UC1611 LIMITED)
// Read Operations
#define UC1609_GET_STATUS B00000001 // TODO:(NOT TESTED) UC1608 datasheet page 10

// Write Operations
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
////////////////////////////////////////////////////////////////
#define LCD_ROTATION_NORMAL 4
#define LCD_ROTATION_FLIP 2
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LCD_UC1609 : public Adafruit_GFX {
  public:
    LCD_UC1609(int8_t _dc, int8_t _rst, int8_t _cs, int8_t _SRAM_cs);
    LCD_UC1609(int8_t _dc, int8_t _rst, int8_t _cs) : LCD_UC1609(_dc, _rst, _cs, -1) {};
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

    void initDisplay();
    void displayEnable(uint8_t b);

    void clearDisplay(void);
    void clearDisplay(uint8_t pixels);
    void clearDisplay(uint8_t pixels, uint8_t mdelay);

    void rotation(uint8_t b);
    void invertDisplay(uint8_t b); // override from Adafruit GFX
    void allPixelsOn(uint8_t b);
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

    void displayBuffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t* data);
    void displayBitmap(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* data);

    // TODO: add overrides, functions removed because horizontal line written directly LCD to memory overwrites 8 pixels tall rect
    //void drawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint16_t color);  //virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    //void drawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint16_t color);  //virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

    // SRAM
    void initSRAM(); // general init for all devices
    void initPSRAM(); // special init for PSRAM64H
    uint8_t SRAM_RDMR(); // read mode register
    void SRAM_WRMR(uint8_t mode); // write mode register
#ifdef USE_SRAM_24BIT_ADDRESS
    void SRAM_read(uint32_t addr, uint8_t* data, uint16_t n); // reads data to buffer
    void SRAM_write(uint32_t addr, uint8_t* data, uint16_t n) ; // writes n data to address
    void SRAM_fill(uint32_t addr, uint8_t data, uint16_t n); // similar to memset
    uint8_t SRAM_read_byte(uint32_t addr);  // similar to PGM_read_byte - reads a byte and returns it
    void SRAM_write_byte(uint32_t addr, uint8_t data); // writes a byte to address
#else
    void SRAM_read(uint16_t addr, uint8_t* data, uint16_t n); // reads data to buffer
    void SRAM_write(uint16_t addr, uint8_t* data, uint16_t n) ; // writes n data to address
    void SRAM_fill(uint16_t addr, uint8_t data, uint16_t n); // similar to memset
    uint8_t SRAM_read_byte(uint16_t addr);  // similar to PGM_read_byte - reads a byte and returns it
    void SRAM_write_byte(uint16_t addr, uint8_t data); // writes a byte to address
#endif
    void displaySRAMBuffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t address); // displays buffer content

  private:
    // LCD
    int8_t dc;
    int8_t cs;
    int8_t rst;
    void LCD_select();
    void LCD_deselect();
    void hardwareReset();
    inline void uc1609_data (uint8_t d);
    void uc1609_command(uint8_t command, uint8_t value);

    // SRAM
    int8_t SRAM_cs;
    void SRAM_select();
    void SRAM_deselect();
};

#endif
