/*!
* @file Adafruit_ILI9341.cpp
*
* @mainpage Adafruit ILI9341 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for Adafruit's ILI9341 driver for the
* Arduino platform.
*
* This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
*    http://www.adafruit.com/products/1651
*
* Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ILI9341
*    https://www.adafruit.com/product/2478
*
* 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ILI9341
*    https://www.adafruit.com/product/1770
*
* 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
*    https://www.adafruit.com/product/1770
*
* TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers
*    https://www.adafruit.com/product/3315
*
* These displays use SPI to communicate, 4 or 5 pins are required
* to interface (RST is optional).
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#ifdef ESP32
#include "ILI9341_2.h"
#include <limits.h>

// if using software spi this optimizes the code

#ifdef ESP32
#undef ILI9341_2_DIMMER
#define ILI9341_2_DIMMER
#undef ESP32_PWM_CHANNEL
#define ESP32_PWM_CHANNEL 1
#endif


const uint16_t ili9341_2_colors[]={ILI9341_2_BLACK,ILI9341_2_WHITE,ILI9341_2_RED,ILI9341_2_GREEN,ILI9341_2_BLUE,ILI9341_2_CYAN,ILI9341_2_MAGENTA,\
  ILI9341_2_YELLOW,ILI9341_2_NAVY,ILI9341_2_DARKGREEN,ILI9341_2_DARKCYAN,ILI9341_2_MAROON,ILI9341_2_PURPLE,ILI9341_2_OLIVE,\
ILI9341_2_LIGHTGREY,ILI9341_2_DARKGREY,ILI9341_2_ORANGE,ILI9341_2_GREENYELLOW,ILI9341_2_PINK};

uint16_t ILI9341_2::GetColorFromIndex(uint8_t index) {
  if (index>=sizeof(ili9341_2_colors)/2) index=0;
  return ili9341_2_colors[index];
}

static const uint8_t PROGMEM ili9341_2_initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_2_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_2_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_2_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_2_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_2_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_2_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_2_PIXFMT  , 1, 0x55,
  ILI9341_2_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_2_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_2_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_2_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_2_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_2_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_2_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

// Constructor when using software SPI.  All output pins are configurable.
ILI9341_2::ILI9341_2(int8_t cs, int8_t mosi, int8_t miso, int8_t sclk, int8_t res, int8_t dc, int8_t bp) : Renderer(ILI9341_2_TFTWIDTH, ILI9341_2_TFTHEIGHT) {
  _cs   = cs;
  _mosi  = mosi;
  _miso  = miso;
  _sclk = sclk;
  _res = res;
  _dc = dc;
  _bp = bp;
  _hwspi = 0;
}

void ILI9341_2::writedata(uint8_t d) {

}

void ILI9341_2::writecmd(uint8_t d) {

}

void ILI9341_2::init(uint16_t width, uint16_t height) {
  sspi2 = SPISettings(2500000, MSBFIRST, SPI_MODE3);
  spi2 = new SPIClass(HSPI); // VSPI
  spi2->begin(_sclk, _miso, _mosi, -1);

//  startWrite();

  uint8_t        cmd, x, numArgs;
  const uint8_t *addr = ili9341_2_initcmd;
  while((cmd = pgm_read_byte(addr++)) > 0) {
      writecmd(cmd);
      x = pgm_read_byte(addr++);
      numArgs = x & 0x7F;
      while(numArgs--) spi2->write(pgm_read_byte(addr++));
      if(x & 0x80) delay(120);
  }

//  endWrite();
}
#endif
