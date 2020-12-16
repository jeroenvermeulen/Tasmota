/*
  xdrv_84_core2.ino - ESP32 m5stack core2 support for Tasmota

  Copyright (C) 2020  Gerhard Mutz and Theo Arends

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

#ifdef ESP32
#ifdef USE_M5STACK_CORE2

#include <AXP192.h>
#include <i2c_bus.h>
#include <soc/rtc.h>

#define XDRV_84          84

struct CORE2_globs {
  AXP192 Axp;
} core2_globs;


// cause SC card is needed by scripter
void CORE2_Module_Init(void) {

  // m5stack uses pin 38 not selectable in tasmota
  SPI.setFrequency(40000000);
  SPI.begin(18, 38, 23, -1);
  // establish power chip on wire1 SDA 21, SCL 22
  core2_globs.Axp.begin();

  I2cSetActiveFound(AXP_ADDR, "AXP192");

}


void CORE2_Init(void) {

}

void CORE2_audio_power(bool power) {
  core2_globs.Axp.SetSpkEnable(power);
}


void CORE2_loop(uint32_t flg) {
}

void CORE2_WebShow(uint32_t json) {
}

const char CORE2_Commands[] PROGMEM = "CORE2|"
  "LSLP";

void (* const CORE2_Command[])(void) PROGMEM = {
  &CORE2_LightSleep};

void CORE2_LightSleep(void) {
}

void core2_disp_pwr(uint8_t on) {
  core2_globs.Axp.SetDCDC3(on);
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv84(uint8_t function) {
  bool result = false;

  switch (function) {

    case FUNC_WEB_SENSOR:
#ifdef USE_WEBSERVER
      CORE2_WebShow(0);
#endif
      break;
    case FUNC_JSON_APPEND:
      CORE2_WebShow(1);
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(CORE2_Commands, CORE2_Command);
      break;
    case FUNC_MODULE_INIT:
      CORE2_Module_Init();
      break;
    case FUNC_INIT:
      CORE2_Init();
      break;
    case FUNC_LOOP:
      CORE2_loop(1);
      break;

  }
  return result;
}

#endif  // USE_M5STACK_CORE2
#endif  // ESP32
