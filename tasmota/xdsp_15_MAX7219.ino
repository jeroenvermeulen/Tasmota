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
#define SOFTSPI

#define MAX7219_BLACK   0x0000
#define MAX7219_WHITE   0xFFFF

// #define MTX_MAX_SCREEN_BUFFER      80

#include <Max72xxPanel.h>

Max72xxPanel *Max7219;
int Max7219_scrollWait = 30;

void Max7219_InitDriver(void)
{
  if (!Settings.display_model) {
    Settings.display_model = XDSP_15;
  }

  int numberOfHorizontalDisplays = 4;
  int numberOfVerticalDisplays = 1;

#ifdef SOFTSPI
  if (TasmotaGlobal.soft_spi_enabled && PinUsed(GPIO_MAX7219_CS) && PinUsed(GPIO_SSPI_MOSI) && PinUsed(GPIO_SSPI_SCLK)) {
    Max7219 = new Max72xxPanel(Pin(GPIO_MAX7219_CS), Pin(GPIO_SSPI_MOSI), Pin(GPIO_SSPI_SCLK), numberOfHorizontalDisplays, numberOfVerticalDisplays);
  }
  else
#endif
  if (TasmotaGlobal.spi_enabled && PinUsed(GPIO_MAX7219_CS)) {
    Max7219 = new Max72xxPanel(Pin(GPIO_MAX7219_CS), numberOfHorizontalDisplays, numberOfVerticalDisplays);
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
  Max7219->scrollDrawText("MAX7219", Max7219_scrollWait);
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

void Max7219_Time(void) {
  char line[20];

//  renderer->clearDisplay();
  snprintf_P(line, sizeof(line), PSTR("%02d" D_MONTH_DAY_SEPARATOR "%02d" D_YEAR_MONTH_SEPARATOR "%04d   %02d" D_HOUR_MINUTE_SEPARATOR "%02d" D_MINUTE_SECOND_SEPARATOR "%02d"), 
             RtcTime.day_of_month, RtcTime.month, RtcTime.year, RtcTime.hour, RtcTime.minute, RtcTime.second);  // [01-02-2021  12:34:56]
  Max7219->scrollDrawText(line, Max7219_scrollWait); // @TODO use async scroll
}

void Max7219_PrintLog(bool withDateTime) {
}

void Max7219_Refresh(void)  // Every second
{
  if (Settings.display_mode) {  // Mode 0 is User text
    switch (Settings.display_mode) {
      case 1:  // Time
        Max7219_Time();
        break;
      case 2:  // Local
      case 4:  // Mqtt
        Max7219_PrintLog(false);
        break;
      case 3:  // Local + Time
      case 5:  // Mqtt + Time
        Max7219_PrintLog(true);
        break;
    }
  }
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
      case FUNC_DISPLAY_EVERY_50_MSECOND:
        Max7219->write();
        break;
#ifdef USE_DISPLAY_MODES1TO5
      case FUNC_DISPLAY_EVERY_SECOND:
        Max7219_Refresh();
        break;
#endif
    }
  }
  return result;
}

#endif  // USE_DISPLAY_MAX7219
#endif  // USE_DISPLAY
#endif  // USE_SPI
