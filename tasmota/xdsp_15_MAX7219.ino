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

// #define MTX_MAX_SCREEN_BUFFER      80

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Define hardware type, size, and output pins:
#define HARDWARE_TYPE              MD_MAX72XX::FC16_HW
#define MAX_DEVICES                4

MD_Parola *myDisplay;

void Max7219_InitDriver(void)
{
    if (PinUsed(GPIO_MAX7219_CS) && TasmotaGlobal.spi_enabled) {
      myDisplay = new MD_Parola(HARDWARE_TYPE, Pin(GPIO_MAX7219_CS), MAX_DEVICES);
    }
    else if (PinUsed(GPIO_MAX7219_CS) && TasmotaGlobal.soft_spi_enabled) {
      myDisplay = new MD_Parola(HARDWARE_TYPE, Pin(GPIO_SSPI_MOSI), Pin(GPIO_SSPI_SCLK), Pin(GPIO_MAX7219_CS), MAX_DEVICES); // @TODO test
    }
    else {
      return;
    }

    // Intialize the object:
    myDisplay->begin();
    // Set the intensity (brightness) of the display (0-15):
    myDisplay->setIntensity(0);
    // Clear the display:
    myDisplay->displayClear();

    #ifdef SHOW_SPLASH
    myDisplay->setTextAlignment(PA_CENTER);
    myDisplay->print("MAX");
    delay(1000);
    myDisplay->print("7219");
    delay(1000);
    myDisplay->displayClear();
    #endif

    myDisplay->setTextAlignment(PA_LEFT);
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
//        MatrixRefresh();
        break;
      case FUNC_DISPLAY_POWER:
//        MatrixOnOff();
        break;
      case FUNC_DISPLAY_DRAW_STRING:
//        MatrixDrawStringAt(dsp_x, dsp_y, dsp_str, dsp_color, dsp_flag);
        break;
    }
  }
  return result;
}

#endif  // USE_DISPLAY_MAX7219
#endif  // USE_DISPLAY
#endif  // USE_I2C
