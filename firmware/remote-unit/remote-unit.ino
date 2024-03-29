/* 
   Waterfloors Heating System (WHS)
   - remote unit module firmware (ATMega328p microcontroller)


V1.0 (2019-11-04)
initial version

V2.0 (2021-07-09)
Sketch uses 16826 bytes (54%) of program storage space. Maximum is 30720 bytes.
Global variables use 921 bytes (44%) of dynamic memory, leaving 1127 bytes for local variables. Maximum is 2048 bytes.

V2.1 (2021-11-13)
+changed completely ICONS (16x16 -> 32x32)
+rework BME280 class (compactify, fix initialization bug, apply all settings at once)
+rework OneWireHub class (removed unused codepaths, compactify, fix small bug, minimize utilized CPU and memory resources)
+change menu polling from 100 to 150ms
+more smooth processing button press (not change screens while holding button)
+added software RESET function
+changed way sleep mode is entered (fixed bug too long sleeping)
+changed where CRC is calculated on writescratchpad 1W cmd (fixed bug not ack write)

Sketch uses 17724 bytes (57%) of program storage space. Maximum is 30720 bytes.
Global variables use 540 bytes (26%) of dynamic memory, leaving 1508 bytes for local variables. Maximum is 2048 bytes.


   Written by dUkk (c) 2018-2021 Wholesome Software
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <U8x8lib.h>
#include "OneWireHub.h"
#include "OneWireItem.h"
#include "myBME280.h"

#define FIRMWARE_VERSION 2.1

#define encoderPinA 9  //PB1
#define encoderPinB 8  //PB0
#define encoderPinSW 10  //PB2

U8X8_SSD1306_128X64_NONAME_HW_I2C Display(U8X8_PIN_NONE); //0x3C byte addr
BME280 mySensor;  //0x76 byte addr
auto hub = OneWireHub(2); //in IDE this is 2 , on PCB this is pin 2
auto ourOWsensor = OneWireItem();

const uint8_t SymbolCurTemperature[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0E, 0x03,
0x03, 0x0E, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,
0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x38, 0x0C, 0xCF, 0xE0, 0xE0,
0xE0, 0xE0, 0xCF, 0x0C, 0x18, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x1C, 0x30, 0x31, 0x23, 0x67,
0x67, 0x63, 0x33, 0x30, 0x1C, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolCurHumidity[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8,
0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x20, 0x04, 0x06, 0xC7, 0x07, 0x07, 0x1F, 0xFF,
0x3F, 0x07, 0x07, 0xC3, 0xF2, 0xFC, 0xF8, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xFF, 0xFE, 0xF8, 0xF8, 0xF1, 0x78, 0x08, 0x04, 0xC0,
0xE0, 0x04, 0x03, 0x03, 0x61, 0x03, 0x03, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x07, 0x0F, 0x0D, 0x1C, 0x1C, 0x1E, 0x1F,
0x1F, 0x1F, 0x1C, 0x0C, 0x0C, 0x04, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  
};
const uint8_t SymbolTempSet[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34,
0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xC0, 0xF0, 0x18, 0x0C, 0x0C, 0x04, 0x06,
0x06, 0x04, 0x0C, 0x0C, 0x18, 0xF0, 0xC0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x71, 0xD8, 0x88, 0x00, 0x00, 0x00, 0xF8,
0xC8, 0x00, 0x00, 0x00, 0xF8, 0xD8, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x1F, 0x00, 0x00, 0x00,
0x11, 0x1F, 0x06, 0x00, 0x00, 0x11, 0x1F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolModeSet[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x48, 0x40,
0x48, 0x40, 0x48, 0x40, 0x48, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
0x00, 0x01, 0x09, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolTOffsetSet[] PROGMEM = {
0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xF4, 0x00, 0xE0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x04, 0x04, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x08, 0x00, 0x00, 0x40, 0x00, 0x80, 0xC0, 0xFF, 0x90, 0x1F,
0x40, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x80, 0xF8, 0xC0, 0x07, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x13, 0x07, 0x07, 0x03, 0x10,
0x08, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08, 0x13, 0x07, 0x07, 0x07, 0x10, 0x08, 0x00, 0x00, 0x00
};
const uint8_t SymbolBrightnessSet[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C,
0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x01, 0xC0, 0x60, 0x18, 0x0C, 0x0C, 0x06, 0x06,
0x06, 0x06, 0x0C, 0x0C, 0x18, 0x60, 0xC0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x07, 0x0C, 0x18, 0x20, 0x40, 0x40, 0x40,
0x40, 0x40, 0x40, 0x20, 0x10, 0x0C, 0x07, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C,
0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolAddressSet[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0xD0, 0x68, 0xB4, 0x54, 0x74, 0x24,
0x24, 0x74, 0x54, 0xB4, 0x68, 0xD0, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x80, 0x39, 0xE6, 0xD0, 0xA0, 0xA0, 0x40,
0x40, 0xA0, 0xA0, 0xD0, 0xE6, 0x39, 0x80, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0xC0, 0x80, 0x00, 0x02, 0x04, 0x13, 0x27, 0x9F, 0x3F,
0x3F, 0x9F, 0x27, 0x13, 0x04, 0x02, 0x00, 0x80, 0xC0, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x40, 0x50, 0x5A, 0x46, 0x41, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41,
0x41, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41, 0x46, 0x5A, 0x50, 0x40, 0x00, 0x00
};
const uint8_t SymbolModeOFF[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x08, 0x08, 0x08, 0x08, 0x18, 0x38, 0x78, 0xF8, 0xF8, 0x78,
0x38, 0x38, 0x78, 0x78, 0x38, 0x38, 0x38, 0x78, 0x38, 0x38, 0x30, 0xE0, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x0C, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x38, 0x38, 0x3F, 0x3C, 0x38,
0x39, 0x31, 0x38, 0x38, 0x38, 0x3C, 0x3E, 0x38, 0x38, 0x3C, 0x1F, 0x1F, 0x07, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolModeCALIBRATION[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x38,
0x38, 0x40, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF3, 0x1C, 0x08,
0x08, 0x1C, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x08, 0x08, 0x80, 0xF0, 0x3C, 0x0B, 0x09, 0x00, 0x10,
0x1C, 0x08, 0x08, 0x0B, 0x1E, 0xF8, 0xC8, 0x00, 0x08, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolModeAUTO_SINGLE[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x80, 0xE0, 0xF0, 0xF8, 0x38, 0x38, 0x98, 0x38, 0x78, 0x38, 0x38, 0xF8, 0x78, 0x78,
0xF8, 0xF8, 0xF8, 0x18, 0x18, 0x08, 0x08, 0x08, 0x00, 0x08, 0x18, 0x10, 0xE0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x0F, 0x1F, 0x1F, 0x3C, 0x39, 0x39, 0x38, 0x3C, 0x3C, 0x3E, 0x3C, 0x38, 0x3C,
0x3F, 0x3F, 0x3F, 0x38, 0x38, 0x20, 0x20, 0x20, 0x20, 0x30, 0x10, 0x08, 0x07, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t SymbolModeAUTO_MULTI[] PROGMEM = {
0x00, 0x00, 0x80, 0x20, 0x10, 0xE8, 0x28, 0x28, 0x98, 0x28, 0x28, 0x28, 0x28, 0xA8, 0x28, 0x28,
0xE8, 0x08, 0x08, 0x18, 0x08, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x10, 0xA0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x08, 0x10, 0x17, 0x24, 0x29, 0x29, 0x28, 0x24, 0x24, 0x2A, 0x24, 0x28, 0x28,
0x2C, 0x28, 0x2A, 0x28, 0x28, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x08, 0x05, 0x00, 0x00, 0x00,
0x00, 0x00, 0x80, 0x20, 0x10, 0xE8, 0x28, 0x28, 0x18, 0x28, 0x48, 0x28, 0x28, 0xA8, 0x28, 0x68,
0xE8, 0x08, 0x48, 0x08, 0x18, 0x08, 0x08, 0x08, 0x00, 0x08, 0x10, 0x10, 0x40, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x08, 0x10, 0x17, 0x28, 0x29, 0x29, 0x28, 0x24, 0x24, 0x2A, 0x24, 0x28, 0x28,
0x2D, 0x28, 0x20, 0x28, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x00, 0x05, 0x00, 0x00, 0x00
};
const uint8_t SymbolModeAUTOHYSTERESIS[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void DisplayScreenMenu();
void SaveSettings();
void GotoSleep();

uint8_t MenuTimeoutTicker = 0; //0 = off menu displaying, 1...149 enable and wait , 150 = turn off menu
uint8_t MenuScreenNum = 0; //0 = menu is off (display is off)
int8_t humidity_offset = 0, CurHumidity = 0;
uint8_t screenBrightness;
uint16_t PollPeriod = 2000;
unsigned long lastTicks;
uint8_t HeaterState = 0;
float HeaterManualTemp = 20.0f;
float CurTemperature = 0.0f;
byte rotaryEncState = 0;

ISR (WDT_vect)
{
  wdt_disable();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinSW, INPUT_PULLUP);

  //turn off ADC,SPI
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
  power_spi_disable();

  //setup display
  Display.setI2CAddress(0x3C * 2);
  Display.begin();
  Display.setFont(u8x8_font_inb21_2x4_n);

  //set vcom value (range 0-7) to lower brightness more
  u8x8_cad_StartTransfer(Display.getU8x8());
  u8x8_cad_SendCmd(Display.getU8x8(), 0x0db);
  u8x8_cad_SendArg(Display.getU8x8(), 0 << 4);
  u8x8_cad_EndTransfer(Display.getU8x8());

  //restore screen brightness
  EEPROM.get(0, screenBrightness);
  if (screenBrightness < 0 || screenBrightness > 10) screenBrightness = 5;
  //restore humidity correction coeff
  EEPROM.get(1, humidity_offset);
  if (humidity_offset < -20 || humidity_offset > 20) humidity_offset = 0;
  //restore ID of our sensor (1-9 dec)
  EEPROM.get(2, ourOWsensor.ID[1]);
  if(ourOWsensor.ID[1] < 0x31 || ourOWsensor.ID[1] > 0x39) ourOWsensor.ID[1] = 0x31;
  ourOWsensor.ID[7] = ourOWsensor.crc8(ourOWsensor.ID, 7);
  //restore temperature correction coeff
  EEPROM.get(5, mySensor.settings.tempCorrection);
  if (isnan(mySensor.settings.tempCorrection) || mySensor.settings.tempCorrection < -5.0f || mySensor.settings.tempCorrection > 5.0f) mySensor.settings.tempCorrection = 0.0f;

  Display.setContrast(map(screenBrightness, 0, 10, 3, 250));
  Display.setPowerSave(0);
  Display.setCursor(0, 0);

  if (mySensor.begin())
  {
    //show our address and version
    Display.print(FIRMWARE_VERSION);
    Display.setCursor(0, 4);
    Display.print(ourOWsensor.ID[1], HEX);
  }
  else
  {
    Display.print(-2);
    while (1) delay(100);
  }

  delay(2500);
  Display.setPowerSave(1);

  //start OneWire communication
  hub.attach(ourOWsensor);

  //FIRST command received from OW should be our configuration

  lastTicks = millis();
}

void loop()
{
  //Poll 1WIRE bus
  hub.poll();

  //grab current sensor readings
  if (ourOWsensor.needSensorReadings)
  {
    ourOWsensor.needSensorReadings=false; //clear flag
    
    //command from underlaying 1W bus - apply new settings grabbed from central unit
    //it should be FIRST or we got invalid data later
    if (ourOWsensor.needApplySettings)
    {
      ourOWsensor.needApplySettings = false; //clear flag
      //only write settings if CRC is OK
      if(ourOWsensor.scratchpad[8] == ourOWsensor.crc8(ourOWsensor.scratchpad, 8)) { 
          //transfer data from 1WIRE object to us
          //empty = float((ourOWsensor.scratchpad[1] << 8 | ourOWsensor.scratchpad[0])) / 100.0f;
          HeaterManualTemp = float(ourOWsensor.scratchpad[3] << 8 | ourOWsensor.scratchpad[2]) / 100.0f;
          humidity_offset = (int8_t)(ourOWsensor.scratchpad[4] & 255);
          HeaterState = ourOWsensor.scratchpad[5];
          //software RESET code
          if(HeaterState == 10) {
              wdt_disable();
              wdt_enable(WDTO_15MS);
              while (1) {}
          }
          mySensor.settings.tempCorrection =  float((int8_t)ourOWsensor.scratchpad[7] << 8 | (int8_t)(ourOWsensor.scratchpad[6] & 255)) / 100.0f;
          SaveSettings();
      }
    }
    
    PORTB |= _BV(PB5); //LED ON

    //get current sensor readings and pack it to our format
    CurTemperature = mySensor.readTempC(); //FIRST should be always TEMP (because of coeff reading for calculations)
    CurHumidity = int8_t(mySensor.readFloatHumidity());
    CurHumidity += humidity_offset;
    //transfer to 1WIRE object
    uint16_t val = CurTemperature * 1000.0f;
    ourOWsensor.scratchpad[0] = (byte)val;
    ourOWsensor.scratchpad[1] = (byte)(val >> 8);

    val = HeaterManualTemp * 100.0f;
    ourOWsensor.scratchpad[2] = (byte)val;
    ourOWsensor.scratchpad[3] = (byte)(val >> 8);

    ourOWsensor.scratchpad[4] = (byte)CurHumidity;
    ourOWsensor.scratchpad[5] = HeaterState;

    int16_t val2 = mySensor.settings.tempCorrection * 100.0f;
    ourOWsensor.scratchpad[6] = (byte)val2;
    ourOWsensor.scratchpad[7] = (byte)(val2 >> 8);

    //calc CRC
    ourOWsensor.scratchpad[8] = ourOWsensor.crc8(ourOWsensor.scratchpad, 8);

    PORTB &= ~ _BV(PB5); //LED OFF

    //if we CAN sleep and not displaying menu -> go to powerdown to save power...
    if(ourOWsensor.howmuchSleep != 0 && MenuScreenNum == 0) GotoSleep();
  }

  //
  unsigned long nowTicks = millis();
  if ((nowTicks - lastTicks) >= PollPeriod)
  {
    lastTicks = nowTicks;

    //determine is main button pressed?
    if(PINB & (1<<PB2))
    {
      //on first press -> turn on display,maximize refresh interval, display first menu screen
      if (MenuScreenNum == 0)
      {
        PollPeriod = 150;
        MenuScreenNum = 1;
        Display.setPowerSave(0);
        DisplayScreenMenu();
      }
      //if user *LONG* pressing initial button (after screen off) we don't want him to randomly increment current displayed screen
      //we want user always after power on land on first screen. so after user DEpressed button -> counter will start counting and after that we will process
      else if(MenuTimeoutTicker > 0) 
      {
        //we should change screens only when user pressed->released button (not holding it). Detect this by looking at timeout: if its counting -> then user is depressed it once
        if(MenuTimeoutTicker > 1) {
          //toggle between screens
          if (MenuScreenNum < 6) MenuScreenNum++;
          else {
            MenuScreenNum = 1;
            //request fresh sensor readings if our screen is first (current temp reading)
            ourOWsensor.needSensorReadings=true;
          }
        }
        //reset menu displaying counter
        MenuTimeoutTicker = 1;        
      }
    }
    //button is depressed -> count number of seconds and turn off screen
    else
    {
      //turn off display
      if (MenuTimeoutTicker == 150)
      {
        Display.setPowerSave(1);
        MenuTimeoutTicker = 0;
        MenuScreenNum = 0;
        PollPeriod = 2000;
        SaveSettings();
        //TODO: here we should go to sleep but we need to ensure timing is good (how much time we spend in menu relative to LAST sensor readings command?)
		    ourOWsensor.howmuchSleep = 0;
      }
      //display current values in realtime
      else
      {
        if (MenuScreenNum > 0)
        {
          DisplayScreenMenu();
          MenuTimeoutTicker++;
        }
      }
    }
    
  }


  //encoder processing section valid only if we not in screen off or current envinroment displaying modes
  if(MenuScreenNum > 1) 
  {
      uint8_t buttonsStates = PINB; //read at single command all our buttons states    
      rotaryEncState = (rotaryEncState << 2) | (buttonsStates & (1<<PB1)) << 1 | (buttonsStates & (1<<PB0));
      //value UP
      //if (rotaryEncState == 0b1101 || rotaryEncState == 0b0100 || rotaryEncState == 0b0010 || rotaryEncState == 0b1011)
      if (rotaryEncState == 0b0100)
      {
        //reset timeout on action while we not in OFF state
        MenuTimeoutTicker = 1;
    
        if (MenuScreenNum == 2)
        {
          if (HeaterManualTemp < 30.0) HeaterManualTemp += 0.1;
          else HeaterManualTemp = 30.0;
        }
        else if (MenuScreenNum == 3)
        {
          if (HeaterState < 4) HeaterState++;
          else HeaterState = 4;
        }
        else if (MenuScreenNum == 4)
        {
          if (mySensor.settings.tempCorrection < 5.0) mySensor.settings.tempCorrection += 0.05;
          else mySensor.settings.tempCorrection = 5.0;
        }
        else if (MenuScreenNum == 5)
        {
          if (screenBrightness < 10) screenBrightness++;
          else screenBrightness = 10;
        }
        else if (MenuScreenNum == 6)
        {
          if (ourOWsensor.ID[1] < 0x39) ourOWsensor.ID[1]++;
          else ourOWsensor.ID[1] = 0x39;
        }
      }
      //value DOWN
      //else if (rotaryEncState == 0b1110 || rotaryEncState == 0b0111 || rotaryEncState == 0b0001 || rotaryEncState == 0b1000)
      else if (rotaryEncState == 0b0001)
      {
        //reset timeout on action while we not in OFF state
        MenuTimeoutTicker = 1;

        if (MenuScreenNum == 2)
        {
          if (HeaterManualTemp > 5.0) HeaterManualTemp -= 0.1;
          else HeaterManualTemp = 5.0;
        }
        else if (MenuScreenNum == 3)
        {
          if (HeaterState > 0) HeaterState--;
          else HeaterState = 0;
        }
        else if (MenuScreenNum == 4)
        {
          if (mySensor.settings.tempCorrection > -5.0) mySensor.settings.tempCorrection -= 0.05;
          else mySensor.settings.tempCorrection = -5.0;
        }
        else if (MenuScreenNum == 5)
        {
          if (screenBrightness > 0) screenBrightness--;
          else screenBrightness = 0;
        }
        else if (MenuScreenNum == 6)
        {
          if (ourOWsensor.ID[1] > 0x31) ourOWsensor.ID[1]--;
          else ourOWsensor.ID[1] = 0x31;
        }
      }
  }
}

void DisplayScreenMenu()
{
  byte buffer[128];
  //do not redraw non changed information
  //we reset to 1 this counter on _any_ user-action
  if (MenuTimeoutTicker > 1) return;

  Display.clearDisplay();

  switch (MenuScreenNum)
  {
    //SCREEN1: display current sensor readings
    case 1:
      {
        memcpy_P(buffer, SymbolCurTemperature, 128);
        Display.drawTile(0, 0, 4, buffer);
        Display.drawTile(0, 1, 4, buffer + 32);
        Display.drawTile(0, 2, 4, buffer + 64);
        Display.drawTile(0, 3, 4, buffer + 96);
        memcpy_P(buffer, SymbolCurHumidity, 128);
        Display.drawTile(0, 4, 4, buffer);
        Display.drawTile(0, 5, 4, buffer + 32);
        Display.drawTile(0, 6, 4, buffer + 64);
        Display.drawTile(0, 7, 4, buffer + 96);
        Display.setInverseFont(0);
        Display.setCursor(5, 0);
        Display.print(CurTemperature, 1);
        Display.setCursor(5, 4);
        Display.print(CurHumidity);
      }
      break;
    //SCREEN2: set new room temperature
    case 2:
      {
        memcpy_P(buffer, SymbolTempSet, 128);
        Display.drawTile(0, 2, 4, buffer);
        Display.drawTile(0, 3, 4, buffer + 32);
        Display.drawTile(0, 4, 4, buffer + 64);
        Display.drawTile(0, 5, 4, buffer + 96);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(HeaterManualTemp, 1);
      }
      break;
    //SCREEN3: set heater operation mode
    case 3:
      {
        memcpy_P(buffer, SymbolModeSet, 128);
        Display.drawTile(0, 2, 4, buffer);
        Display.drawTile(0, 3, 4, buffer + 32);
        Display.drawTile(0, 4, 4, buffer + 64);
        Display.drawTile(0, 5, 4, buffer + 96);
        switch (HeaterState)
        {
          case 0:
            memcpy_P(buffer, SymbolModeOFF, 128);
            break;
          case 1:
            memcpy_P(buffer, SymbolModeCALIBRATION, 128);
            break;
          case 2:
            memcpy_P(buffer, SymbolModeAUTO_SINGLE, 128);
            break;
          case 3:
            memcpy_P(buffer, SymbolModeAUTO_MULTI, 128);
            break;
          case 4:
            memcpy_P(buffer, SymbolModeAUTOHYSTERESIS, 128);
            break;
        }
        Display.drawTile(5, 2, 4, buffer);
        Display.drawTile(5, 3, 4, buffer + 32);
        Display.drawTile(5, 4, 4, buffer + 64);
        Display.drawTile(5, 5, 4, buffer + 96);        
      }
      break;
    //SCREEN4: temp compensation
    case 4:
      {
        memcpy_P(buffer, SymbolTOffsetSet, 128);
        Display.drawTile(0, 2, 4, buffer);
        Display.drawTile(0, 3, 4, buffer + 32);
        Display.drawTile(0, 4, 4, buffer + 64);
        Display.drawTile(0, 5, 4, buffer + 96);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(mySensor.settings.tempCorrection);
      }
      break;
    //SCREEN5: display brightness
    case 5:
      {
        memcpy_P(buffer, SymbolBrightnessSet, 128);
        Display.drawTile(0, 2, 4, buffer);
        Display.drawTile(0, 3, 4, buffer + 32);
        Display.drawTile(0, 4, 4, buffer + 64);
        Display.drawTile(0, 5, 4, buffer + 96);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(screenBrightness);
        Display.setContrast(map(screenBrightness, 0, 10, 3, 250));
      }
      break;
    //SCREEN6: panel ID (0x31..0x39)
    case 6:
      {
        memcpy_P(buffer, SymbolAddressSet, 128);
        Display.drawTile(0, 2, 4, buffer);
        Display.drawTile(0, 3, 4, buffer + 32);
        Display.drawTile(0, 4, 4, buffer + 64);
        Display.drawTile(0, 5, 4, buffer + 96);
        Display.setCursor(5, 0);
        Display.setInverseFont(1);
        Display.print(ourOWsensor.ID[1], HEX);
        byte curAddr;
        EEPROM.get(2, curAddr);
        Display.setCursor(5, 4);
        Display.setInverseFont(0);
        Display.print(curAddr, HEX);
      }
      break;
  }
}

void SaveSettings()
{
  EEPROM.put(0, screenBrightness);
  EEPROM.put(1, humidity_offset);
  EEPROM.put(2, ourOWsensor.ID[1]);
  EEPROM.put(5, mySensor.settings.tempCorrection);
}

void GotoSleep()
{
   while(ourOWsensor.howmuchSleep != 0)
   {
    ourOWsensor.howmuchSleep--;
    //prepare us to sleep for 2seconds by WatchDog, signal interrupt on complete
    cli(); 
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    WDTCSR =  _BV(WDP0) | _BV(WDP1) | _BV(WDP2) | _BV(WDIE);
    sei();
    //now put to sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    //check is MAIN button pressed?
    if(PINB & (1<<PB2))
    {
      ourOWsensor.howmuchSleep = 0; //turn off sleep mode completely
      PollPeriod = 100;
      break;
    }    
   }
}
