/*
  version 1.0.0    June 2020

  "Revised BSD License",
  Copyright (c) 2020, Lukas Vyhnalek aka KiLLA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "LCD_UC1609.h"

////////////////////////////////////////////////////////////////
// Send a byte to the display
void LCD_UC1609::uc1609_data (uint8_t d) {
  (void)SPI.transfer(d);
}

// Send a command to the display
void LCD_UC1609::uc1609_command (uint8_t command, uint8_t value) {
#ifdef __AVR__
  PORTB &= ~(1 << dc);
  uc1609_data(command | value);
  PORTB |= (1 << dc);
#else
  digitalWrite(dc, LOW);
  uc1609_data(command | value);
  digitalWrite(dc, HIGH);
#endif
}

LCD_UC1609 :: LCD_UC1609(int8_t _dc, int8_t _rst, int8_t _cs, int8_t _SRAM_cs) : Adafruit_GFX(LCD_WIDTH, LCD_HEIGHT) {
  // check for define and set pin numbers
#ifdef __AVR__
  dc = _dc % 8;
  rst = _rst % 8;
  cs = _cs % 8;
  SRAM_cs = _SRAM_cs % 8;
#else
  dc = _dc;
  rst = _rst;
  cs = _cs;
  SRAM_cs = _SRAM_cs;
#endif
  // check for define and set buffer size
#ifndef USE_ADVANCED_BUFFERING
  bufferWidth = LCD_WIDTH;
  bufferHeight = LCD_HEIGHT;
#endif
}

// https://www.arduino.cc/en/reference/SPI
void LCD_UC1609::begin () {
#ifdef __AVR__
  DDRB = 1 << dc | 1 << cs | 1 << rst ; //
  if (SRAM_cs != -1) {
    DDRD = 1 << SRAM_cs;
  }
#else
  pinMode(dc, OUTPUT);
  pinMode(cs, OUTPUT);
  pinMode(rst, OUTPUT);
  if (SRAM_cs != -1) {
    pinMode (SRAM_cs, OUTPUT);
  }
#endif

  if (true) {
    SPI.begin();
#ifdef SPI_HAS_TRANSACTION
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // FAST
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // slow
#else
    SPI.setClockDivider (1); // FAST
    //SPI.setClockDivider (8); // slow
#endif
  }
  /* esp 8266 TESTING only!!! and maybe esp32... 22/5/2020 TODO: TESTING!
    // https://github.com/esp8266/Arduino/issues/2624
    SPI.setFrequency(32000000L);
    //SPI.setFrequency(24000000L);
    //SPI.setFrequency(16000000L);
    SPI.setBitOrder(MSBFIRST);
  */
  // DISPLAY POWER ON SEQUENCE
  initDisplay();

  // INIT SRAM
  SRAM_deselect(); // for intitial high to low transition
  SRAM_WRMR(SRAM_SEQUENTIAL_MODE); // sram<-------------------------------------------------------- sram
}

void LCD_UC1609::initDisplay() {
  // POWER ON SEQUENCE
#ifdef __AVR__
  PORTB |= (1 << dc);
#else
  digitalWrite(dc, HIGH);
#endif
  LCD_deselect();
  hardwareReset();

  LCD_select();
  //uc1609_command(UC160X_SYS_RESET, 0); // not required to do sys reset
  //delay(100);
  uc1609_command(UC160X_SET_MR_TC, 0); // set the multiplex ratio and temperature compensation
  uc1609_command(UC160X_SET_AC, B00000010); // set RAM address control
  uc1609_command(UC160X_SET_BR, 3); // set bias ratio to default
  uc1609_command(UC160X_SET_POWER_CONTROL, 0); // set power control
  delay(100);
  uc1609_command(UC160X_SET_GN_PM, 0); // set gain and potentiometer - double byte command
  uc1609_command(UC160X_SET_GN_PM, UC1608_DEFAULT_GN_PM); // set gain and potentiometer

  uc1609_command(UC160X_SET_DISPLAY_ENABLE, 1); // display enable
  uc1609_command(UC160X_SET_MAP_CONTROL, LCD_ROTATION_NORMAL); // set map control  bit 3: MY, bit 2: MX, bit 1 unset, bit 0: MSF
  LCD_deselect();
}

void LCD_UC1609::LCD_select () {
#ifdef __AVR__
  PORTB &= ~(1 << cs);
#else
  digitalWrite(cs, LOW);
#endif
}

void LCD_UC1609::LCD_deselect () {
#ifdef __AVR__
  PORTB |= (1 << cs);
#else
  digitalWrite(cs, HIGH);
#endif
}

void LCD_UC1609::hardwareReset () {
#ifdef __AVR__
  PORTB &= ~(1 << rst);
  delay(150);
  PORTB |= (1 << rst);
#else
  digitalWrite(rst, LOW);
  delay(150);
  digitalWrite(rst, HIGH);
#endif
}

void LCD_UC1609::displayEnable (uint8_t b) {
  LCD_select();
  uc1609_command(UC160X_SET_DISPLAY_ENABLE, b);
  LCD_deselect();
}

void LCD_UC1609::rotation(uint8_t b) {
  LCD_select();
  b = (b == 0) ? LCD_ROTATION_NORMAL : LCD_ROTATION_FLIP;
  uc1609_command(UC160X_SET_MAP_CONTROL, b);
  LCD_deselect();
}

void LCD_UC1609::invertDisplay (uint8_t b) {
  LCD_select();
  uc1609_command(UC1609_SET_INVERSE_DISPLAY, b);
  LCD_deselect();
}

void LCD_UC1609::allPixelsOn (uint8_t i) {
  LCD_select();
  uc1609_command(UC1609_SET_ALL_PIXEL_ON, i);
  LCD_deselect();
}

void LCD_UC1609::drawPixel(int16_t x, int16_t y, uint16_t color) {
  // do offscreen
  if ((x < 0) || (x >= LCD_WIDTH) || (y < 0) || (y >= LCD_HEIGHT)) {
    return;
  }
#ifdef USE_ADVANCED_BUFFERING
  uint16_t tc = (this->selectedBuffer->width * (y >> 3)) + x; //

  switch (this->selectedBuffer->type) {
    case TYPE_INTERNAL_V:
      switch (color)
      {
        case WHITE:   this->selectedBuffer->bitmap[tc] |= (1 << (y & 7)); break;
        case BLACK:   this->selectedBuffer->bitmap[tc] &= ~(1 << (y & 7)); break;
        case INVERSE: this->selectedBuffer->bitmap[tc] ^= (1 << (y & 7)); break;
      }
      break;
    case TYPE_SRAM_V:
      switch (color)
      {
          uint8_t data;
        case WHITE:    data = SRAM_read_byte(this->selectedBuffer->address + tc); data |= (1 << (y & 7)); SRAM_write_byte(this->selectedBuffer->address + tc, data); break;
        case BLACK:    data = SRAM_read_byte(this->selectedBuffer->address + tc); data &= ~(1 << (y & 7)); SRAM_write_byte(this->selectedBuffer->address + tc, data); break;
        case INVERSE:  data = SRAM_read_byte(this->selectedBuffer->address + tc); data ^= (1 << (y & 7)); SRAM_write_byte(this->selectedBuffer->address + tc, data); break;
      }
  }
#else
  uint16_t tc = (this->bufferWidth * (y >> 3)) + x; //
  switch (color)
  {
    case WHITE:   this->buffer[tc] |= (1 << (y & 7)); break;
    case BLACK:   this->buffer[tc] &= ~(1 << (y & 7)); break;
    case INVERSE: this->buffer[tc] ^= (1 << (y & 7)); break;
  }
#endif
}

void LCD_UC1609::display() {
  // compatibility!
  //uint16_t x = 0; uint16_t y = 0; uint16_t w = 192; uint16_t h = 64; // FAILSAFE - JUST FOR TESTING, the buffer must have same dimensions
#ifdef USE_ADVANCED_BUFFERING
  switch (this->selectedBuffer->type) {
    case TYPE_INTERNAL_V:
      displayBuffer( this->selectedBuffer->x, this->selectedBuffer->y, this->selectedBuffer->width, this->selectedBuffer->height, (uint8_t*) this->selectedBuffer->bitmap); break;
    case TYPE_SRAM_V:
      displaySRAMBuffer( this->selectedBuffer->x, this->selectedBuffer->y, this->selectedBuffer->width, this->selectedBuffer->height, this->selectedBuffer->address); break;
  }
#else
  uint16_t x = 0; uint16_t y = 0; uint16_t w = this->bufferWidth; uint16_t h = this->bufferHeight; // For simple buffer, x, y are zero
  displayBuffer( x,  y,  w,  h, (uint8_t*) this->buffer);
#endif
}

void LCD_UC1609::clearDisplay() {
  // CLEARS CURRENTLY SELECTED BUFFER! compatibility!
#ifdef USE_ADVANCED_BUFFERING
  switch (this->selectedBuffer->type) {
    case TYPE_INTERNAL_V:
      memset( this->selectedBuffer->bitmap, 0x00, (this->selectedBuffer->width * (this->selectedBuffer->height >> 3))  ); break;
    case TYPE_SRAM_V:
      SRAM_fill(this->selectedBuffer->address, 0x00, (this->selectedBuffer->width * (this->selectedBuffer->height >> 3))); break;
  }
#else
  memset( this->buffer, 0x00, (this->bufferWidth * (this->bufferHeight >> 3))  ); // avoid sizeof to keep dynamic!
#endif
}

void LCD_UC1609::clearDisplay(uint8_t pixels) {
  // NEW FUNCTION to overwrite the display memory with value pixels -> 0 for clear, 0xFF all pixels on, 0x55  -> strips, 0xaa -> strips INVERTED
  LCD_select();
  uint16_t bytesize = LCD_WIDTH * (LCD_HEIGHT >> 3); // width * height

  for (uint16_t tx = 0; tx < bytesize; tx++) {
    (void)SPI.transfer( pixels );
  }
  LCD_deselect();
}

void LCD_UC1609::clearDisplay(uint8_t pixels, uint8_t mdelay) {
  // NEW FUNCTION to overwrite the display memory, with delayMicroseconds between writes
  LCD_select();
  uint16_t bytesize = LCD_WIDTH * (LCD_HEIGHT >> 3); // width * height

  for (uint16_t tx = 0; tx < bytesize; tx++) {
    (void)SPI.transfer( pixels );
    delayMicroseconds(mdelay);
  }
  LCD_deselect();
}

void LCD_UC1609::displayBuffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t* data) {
  LCD_select();

  uint8_t tx, ty; // temp x, y
  uint16_t tc = 0; // mem offset

  uint8_t col = (x < 0) ? 0 : x;
  uint8_t page = (y < 0) ? 0 : y >> 3;

  for (ty = 0; ty < h; ty = ty + 8) {
    if (y + ty < 0 || y + ty >= LCD_HEIGHT) {
      continue;
    }
    uc1609_command(UC1609_SET_COL_LSB, (col & B00001111)); // column low nibble
    uc1609_command(UC1609_SET_COL_MSB, (col & B11110000) >> 4); // column high nibble
    uc1609_command(UC1609_SET_PAGE, page++); //  page adr

    for (tx = 0; tx < w; tx++) {
      if (x + tx < 0 || x + tx >= LCD_WIDTH) {
        continue;
      }
      tc = (w * (ty >> 3)) + tx; // get offset to read from vertically addressed bitmap
      (void)SPI.transfer( data[tc++] ); //
    }
  }
  LCD_deselect();
}

void LCD_UC1609::displayBitmap(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t* data) {
  LCD_select();

  uint8_t tx, ty; // temp x, y
  uint16_t tc = 0; // mem offset

  uint8_t col = (x < 0) ? 0 : x;
  uint8_t page = (y < 0) ? 0 : y >> 3;

  for (ty = 0; ty < h; ty = ty + 8) {
    if (y + ty < 0 || y + ty >= LCD_HEIGHT) {
      continue;
    }
    uc1609_command(UC1609_SET_COL_LSB, (col & B00001111)); // column low nibble
    uc1609_command(UC1609_SET_COL_MSB, (col & B11110000) >> 4); // column high nibble
    uc1609_command(UC1609_SET_PAGE, page++); //  page adr

    for (tx = 0; tx < w; tx++) {
      if (x + tx < 0 || x + tx >= LCD_WIDTH) {
        continue;
      }
      tc = (w * (ty >> 3)) + tx; // get offset to read from vertically addressed bitmap
      (void)SPI.transfer(  pgm_read_byte(&data[tc]) ); // send byte
    }
  }
  LCD_deselect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SRAM
void LCD_UC1609::displaySRAMBuffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t address) {
  uint8_t  xtraBuffer[w]; // internal memory buffer large as bitmap width to speed up transfer -> caching

  uint8_t tx, ty; // temp x, y
  uint16_t tc = 0; // mem offset

  uint8_t col = (x < 0) ? 0 : x;
  uint8_t page = (y < 0) ? 0 : y >> 3;

  for (ty = 0; ty < h; ty = ty + 8) {
    if (y + ty < 0 || y + ty >= LCD_HEIGHT) {
      continue;
    }
    tc = 0;
    SRAM_read(address + (w * (ty >> 3)), (uint8_t*)&xtraBuffer, w);

    LCD_select();
    uc1609_command(UC1609_SET_COL_LSB, (col & B00001111)); // column low nibble
    uc1609_command(UC1609_SET_COL_MSB, (col & B11110000) >> 4); // column high nibble
    uc1609_command(UC1609_SET_PAGE, page++); //  page adr

    for (tx = 0; tx < w; tx++) {
      if (x + tx < 0 || x + tx >= LCD_WIDTH) {
        continue;
      }
      //tc = (w * (ty >> 3)) + tx; // get offset to read from vertically addressed bitmap not required here until we write outside of screen
      (void)SPI.transfer(  xtraBuffer[tc++] ); // send byte
    }
    LCD_deselect();
  }
}

// SRAM chip select
void LCD_UC1609::SRAM_select() {
  // SPI.beginTransaction(SRAM_settings); // futureproof
  // SPI.setModule(2); // stm
#ifdef __AVR__
  PORTD &= ~(1 << SRAM_cs);
#else
  digitalWrite(SRAM_cs, LOW);
#endif
}

// SRAM chip deselect
void LCD_UC1609::SRAM_deselect() {
  // SPI.setModule(1); // stm
#ifdef __AVR__
  PORTD |= (1 << SRAM_cs);
#else
  digitalWrite(SRAM_cs, HIGH);
#endif
}

// SRAM read mode register
uint8_t LCD_UC1609::SRAM_RDMR() {
  byte rc;
  SRAM_select();
  SPI.transfer(SRAM_CMD_RDMR);
  rc = SPI.transfer(0xFF);
  SRAM_deselect();
  return rc;
}

// SRAM write mode register
void LCD_UC1609::SRAM_WRMR(uint8_t mode) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_WRMR);
  SPI.transfer(mode);
  SRAM_deselect();
}

// SRAM read - reads data to buffer
void LCD_UC1609::SRAM_read(uint16_t addr, uint8_t* data, uint16_t n) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_READ);
  SPI.transfer((uint8_t)(addr >> 8) & 0xFF); // A15-A08
  SPI.transfer((uint8_t)(addr) & 0xFF);       // A07-A00
  for (uint16_t i = 0; i < n; i++ ) {
    data[i] = SPI.transfer(0xFF);
  }
  SRAM_deselect();
}

// SRAM write - writes n data to address
void LCD_UC1609::SRAM_write(uint16_t addr, uint8_t* data, uint16_t n) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_WRITE);
  SPI.transfer((uint8_t)(addr >> 8) & 0xFF); // A15-A08
  SPI.transfer((uint8_t)(addr) & 0xFF);       // A07-A00
  for (uint16_t i = 0; i < n; i++) {
    SPI.transfer(data[i]);
  }
  SRAM_deselect();
}

// SRAM fill - similar to memset
void LCD_UC1609::SRAM_fill(uint16_t addr, uint8_t data, uint16_t n) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_WRITE);
  SPI.transfer((uint8_t)(addr >> 8) & 0xFF); // A15-A08
  SPI.transfer((uint8_t)(addr) & 0xFF);       // A07-A00
  for (uint16_t i = 0; i < n; i++) {
    SPI.transfer(data);
  }
  SRAM_deselect();
}

// SRAM read byte - similar to PGM_read_byte - reads a byte and returns it
uint8_t LCD_UC1609::SRAM_read_byte(uint16_t addr) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_READ);
  SPI.transfer((uint8_t)(addr >> 8) & 0xFF); // A15-A08
  SPI.transfer((uint8_t)(addr) & 0xFF);       // A07-A00
  uint8_t data;
  data = SPI.transfer(0x00);
  SRAM_deselect();
  return data;
}

// SRAM write - writes a byte to address
void LCD_UC1609::SRAM_write_byte(uint16_t addr, uint8_t data) {
  SRAM_select();
  SPI.transfer(SRAM_CMD_WRITE);
  SPI.transfer((uint8_t)(addr >> 8) & 0xFF); // A15-A08
  SPI.transfer((uint8_t)(addr) & 0xFF);       // A07-A00
  SPI.transfer(data);
  SRAM_deselect();
}
