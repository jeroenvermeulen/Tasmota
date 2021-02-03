/*
  xdsp_15_MAX7219.ino - MAX7219 Dot matrix led display support for Tasmota

  Copyright (C) 2021  Jeroen Vermeulen and MajicDesigns

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SPI
#ifdef USE_DISPLAY
#ifdef USE_DISPLAY_MAX7219 // This driver eats 17.1 K flash // @TODO verify + make smaller?

#define XDSP_15                    15

#define MAX7219_BLACK   0x0000
#define MAX7219_WHITE   0xFFFF

// #define MTX_MAX_SCREEN_BUFFER      80

//#include <MD_Parola.h>
//#include <MD_MAX72xx.h>
#include <Max72xxPanel.h>
//#include <SPI.h>

// Define hardware type, size, and output pins:
//#define HARDWARE_TYPE              MD_MAX72XX::FC16_HW
//#define MAX_DEVICES                4

//MD_Parola *myDisplay;
Max72xxPanel *Max7219;

void Max7219_InitDriver(void)
{
  if (!Settings.display_model) {
    Settings.display_model = XDSP_15;
  }

  int numberOfHorizontalDisplays = 4;
  int numberOfVerticalDisplays = 1;

  if (PinUsed(GPIO_MAX7219_CS) && TasmotaGlobal.spi_enabled) {
      Max7219 = new Max72xxPanel(Pin(GPIO_MAX7219_CS), numberOfHorizontalDisplays, numberOfVerticalDisplays);
  }
  else if (PinUsed(GPIO_MAX7219_CS) && TasmotaGlobal.soft_spi_enabled) {
    //Max7219 = new Max72xxPanel(HARDWARE_TYPE, Pin(GPIO_SSPI_MOSI), Pin(GPIO_SSPI_SCLK), Pin(GPIO_MAX7219_CS), MAX_DEVICES); // @TODO test
  }
  else {
    return;
  }

  for(int i = 0; i < 4; i++) {
    Max7219->setPosition(i, i, 0);  // The i'th display is at <i, 0>
  }

  // the setRotation function is responsible for the orientation of displays
  for(int i = 0; i < 8; i++) {
    Max7219->setRotation(i, 1);     // rotate all displays 90 degrees
  }

  fg_color = MAX7219_WHITE;
  bg_color = MAX7219_BLACK;

  renderer = Max7219;
  renderer->setTextColor(fg_color, bg_color);

  #ifdef SHOW_SPLASH
  //for (int i=0; i<100; i++) {
    Max7219->scrollDrawText("MAX7219", 25);
  //}
//    renderer->clearDisplay();
//    renderer->DrawStringAt(10,0,"MAX", MAX7219_WHITE, 0);
//    renderer->print("MAX");
//    Max7219->write();
//    delay(1000);
//    renderer->clearDisplay();
//    renderer->DrawStringAt(8,0,"7219", MAX7219_WHITE, 0);
//    Max7219->write();
//    delay(1000);
  renderer->clearDisplay();
  Max7219->write();
  #endif

    //myDisplay->setTextAlignment(PA_LEFT);
  AddLog(LOG_LEVEL_INFO, PSTR("DSP: MAX7219"));

//      MatrixInitMode();
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdsp15(uint8_t function)
{
  // if (!I2cEnabled(XI2C_05)) { return false; } // TODO: check SPI

  bool result = false;

  if (FUNC_DISPLAY_INIT_DRIVER == function) {
    Max7219_InitDriver();
  }
  else if (XDSP_15 == Settings.display_model) {
    switch (function) {
      case FUNC_DISPLAY_MODEL:
        result = true;
        break;
      case FUNC_DISPLAY_INIT:
//        MatrixInit(dsp_init);
        break;
      case FUNC_DISPLAY_EVERY_50_MSECOND:
        Max7219->write();
        break;
      case FUNC_DISPLAY_POWER:
//        MatrixOnOff();
        break;
      case FUNC_DISPLAY_DRAW_STRING:
//        renderer->DrawStringAt(dsp_x, dsp_y, dsp_str, dsp_color, dsp_flag);
//        Max7219->write();
        break;
    }
  }
  return result;
}

#endif  // USE_DISPLAY_MAX7219
#endif  // USE_DISPLAY
#endif  // USE_I2C
