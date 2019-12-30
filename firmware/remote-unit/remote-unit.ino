/* 
   remote unit module firmware for water floors heating system

   Written by dUkk (c) 2018-2019
*/

#include <EEPROM.h>
#include "OneWireHub.h"
#include "heatproto.h"
#include <U8x8lib.h>
#include "SFBME280_clone.h"

#define FIRMWARE_VERSION 1.0
#define ADDRESSBYTE 0x14

#define encoderPinA 9
#define encoderPinB 8
#define encoderPinSW 10

U8X8_SSD1306_128X64_NONAME_HW_I2C Display(U8X8_PIN_NONE); //0x3C byte addr
BME280 mySensor;  //0x76 byte addr
auto hub = OneWireHub(2); //in IDE this is 2 , on PCB this is pin 2
auto ourOWsensor = HeaterOW(0x60, ADDRESSBYTE, 0xDE, 0xAD, 0xBE, 0xEF, 0x00);

const uint8_t SymbolCurTemperature [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x03, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x6b, 0x5c, 0xdc, 0x7b, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t SymbolCurHumidity [32] = {0x00, 0x00, 0x00, 0x80, 0x60, 0xb0, 0x38, 0xfc, 0x7c, 0xb8, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x3e, 0x26, 0x79, 0x7e, 0x31, 0x25, 0x13, 0x0f, 0x00, 0x00, 0x00};
const uint8_t SymbolTempSet [32] = {0x00, 0x80, 0x80, 0x18, 0x90, 0xc0, 0x60, 0x26, 0x26, 0x60, 0xc0, 0x90, 0x18, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x04, 0x7e, 0x30, 0x0e, 0x7a, 0x00, 0x6e, 0x30, 0x00, 0x01, 0x01, 0x00};
const uint8_t SymbolModeSet [32] = {0x00, 0x00, 0x00, 0x06, 0x0e, 0x0e, 0x2e, 0xfe, 0xfe, 0xfe, 0xfc, 0x98, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x3f, 0x2f, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t SymbolTOffsetSet [32] = {0x00, 0x06, 0x06, 0xfe, 0x00, 0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0xfe, 0x02, 0xfe, 0x00, 0x00, 0x00, 0x02, 0x03, 0x03, 0x38, 0x7f, 0x7f, 0x7f, 0x3c, 0x00, 0x3c, 0x7f, 0x7e, 0x7f, 0x3c, 0x00};
const uint8_t SymbolBrightnessSet [32] = {0x00, 0x80, 0x88, 0x18, 0x90, 0xe0, 0x60, 0x3e, 0x3e, 0x60, 0xe0, 0x90, 0x18, 0x88, 0x80, 0x00, 0x01, 0x01, 0x01, 0x38, 0x1b, 0x07, 0x0c, 0xec, 0xec, 0x0c, 0x07, 0x1b, 0x38, 0x01, 0x01, 0x01};
const uint8_t SymbolModeOFF1 [16] = {0xff, 0x7f, 0x3f, 0x9f, 0x9f, 0x9f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x3f, 0x7f, 0xff};
const uint8_t SymbolModeOFF2 [16] = {0xff, 0xfc, 0xf9, 0xfb, 0xf3, 0xf9, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xfc, 0xff};
const uint8_t SymbolModeON1 [16] = {0xff, 0x3f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x9f, 0xdf, 0x9f, 0x1f, 0x7f, 0xff};
const uint8_t SymbolModeON2 [16] = {0xff, 0xfc, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf9, 0xfb, 0xfb, 0xf8, 0xfc, 0xff};
const uint8_t SymbolModeAUTO1 [16] = {0x7f, 0x3f, 0x1f, 0x9f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x9f, 0x1f, 0x1f, 0x1f, 0x1f, 0x3f, 0xff};
const uint8_t SymbolModeAUTO2 [16] = {0xfc, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xff};
const uint8_t SymbolModeCALIBRATION [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x6c, 0x6c, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x3e, 0x1f, 0x07, 0x06, 0x06, 0x03, 0x1f, 0x3e, 0x06, 0x02, 0x00, 0x00};

void DisplayScreenMenu();
void SaveSettings();

uint8_t MenuTimeoutTicker = 0; //0 = off menu displaying, 1...149 enable and wait , 150 = turn off menu
uint8_t MenuScreenNum = 0; //0 = menu is off (display is off)
int8_t humidity_offset = 0, CurHumidity = 0;
uint8_t screenBrightness;
uint16_t PollPeriod = 2000;
unsigned long lastTicks = 0;
uint8_t HeaterState = 0;
float HeaterManualTemp = 20.0f;
float CurTemperature = 0.0f;
byte stateLast = 0;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // Setup the buttons
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinSW, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH);

  //setup display
  Display.setI2CAddress(0x3C * 2);
  Display.begin();
  Display.setFont(u8x8_font_inb21_2x4_n);

  u8x8_cad_StartTransfer(Display.getU8x8());
  u8x8_cad_SendCmd(Display.getU8x8(), 0x0db);
  u8x8_cad_SendArg(Display.getU8x8(), 0 << 4);
  u8x8_cad_EndTransfer(Display.getU8x8());

  EEPROM.get(0, screenBrightness);
  if (screenBrightness < 0 || screenBrightness > 10) screenBrightness = 5;
  EEPROM.get(1, humidity_offset);
  if (humidity_offset < -20 || humidity_offset > 20) humidity_offset = 0;
  EEPROM.get(5, mySensor.settings.tempCorrection);
  if (mySensor.settings.tempCorrection < -5.0f || mySensor.settings.tempCorrection > 5.0f) mySensor.settings.tempCorrection = 0.0f;

  Display.setContrast(map(screenBrightness, 0, 10, 0, 250));
  Display.setPowerSave(0);
  Display.setCursor(0, 0);

  mySensor.settings.tStandby = 5; // every 1000ms
  mySensor.settings.filter = 2; //
  mySensor.settings.tempOverSample = 2;
  mySensor.settings.pressOverSample = 0;
  mySensor.settings.humidOverSample = 2;
  if (mySensor.begin() != 0x60)
  {
    Display.print(-2);
    while (1) delay(100);
  }
  //show our address and version
  else
  {
    Display.setCursor(0, 0);
    Display.print(FIRMWARE_VERSION);
    Display.setCursor(0, 4);
    Display.print(ADDRESSBYTE, HEX);
  }

  delay(2500);
  Display.setPowerSave(1);

  stateLast = (byte)digitalRead(encoderPinA) << 1 | (byte)digitalRead(encoderPinB);
  stateLast = stateLast & 0xF;

  // Setup OneWire
  hub.attach(ourOWsensor);
}

void loop()
{
  //Poll 1WIRE bus
  hub.poll();

  //first command from underlaying 1W bus - apply new settings grabbed from central unit
  //it should be FIRST or we got invalid data later
  if (ourOWsensor.NeedToRefreshSettings)
  {
    ourOWsensor.NeedToRefreshSettings = false;

    //transfer data from 1WIRE object to us
    HeaterManualTemp = float(ourOWsensor.scratchpad[3] << 8 | ourOWsensor.scratchpad[2]) / 100.0f;
    humidity_offset = (int8_t)(ourOWsensor.scratchpad[4] & 255);
    HeaterState = ourOWsensor.scratchpad[5];
    mySensor.settings.tempCorrection =  float((int8_t)ourOWsensor.scratchpad[7] << 8 | (int8_t)(ourOWsensor.scratchpad[6] & 255)) / 100.0f;
    SaveSettings();
  }

  //second command from underlaying 1W bus - grab current sensor readings
  if (ourOWsensor.NeedToPollSensor)
  {
    ourOWsensor.NeedToPollSensor = false;
    digitalWrite(LED_BUILTIN, HIGH);

    //get current sensor readings and pack it to our format
    CurTemperature = mySensor.readTempC(); //FIRST should be allways TEMP (because of coeff reading for calculations)
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
    ourOWsensor.updateCRC();

    digitalWrite(LED_BUILTIN, LOW);
  }

  //
  unsigned long nowTicks = millis();
  if ((nowTicks - lastTicks) >= PollPeriod)
  {
    lastTicks = nowTicks;

    //determine is main button pressed?
    if (digitalRead(encoderPinSW) == HIGH)
    {
      //reset menu timeout couter
      MenuTimeoutTicker = 1;
      //on first press -> turn on display,maximize refresh interval, display first menu screen
      if (MenuScreenNum == 0)
      {
        PollPeriod = 150;
        MenuScreenNum = 1;
        Display.setPowerSave(0);
        DisplayScreenMenu();
      }
      //toggle between screens
      if (MenuScreenNum < 5) MenuScreenNum++;
      else MenuScreenNum = 1;
      //request fresh sensor readings each time user pushed button
      ourOWsensor.NeedToPollSensor = true;
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
      }
      //display current values in realtime
      else
      {
        if (MenuTimeoutTicker > 0)
        {
          DisplayScreenMenu();
          MenuTimeoutTicker++;
        }
      }
    }
  }

  byte state = stateLast;
  state = state << 2 | (byte)digitalRead(encoderPinA) << 1 | (byte)digitalRead(encoderPinB);
  state = state & 0xF;
  stateLast = state;
  //value UP
  if (state == 0b0010) //full steps
  {
    //reset timeout on action while we not in OFF state
    if (MenuScreenNum > 0)
    {
      MenuTimeoutTicker = 1;
    }

    if (MenuScreenNum == 2)
    {
      if (HeaterManualTemp < 30.0) HeaterManualTemp += 0.1;
      else HeaterManualTemp = 30.0;
    }
    else if (MenuScreenNum == 3)
    {
      if (HeaterState < 3) HeaterState++;
      else HeaterState = 3;
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
  }
  //value DOWN
  else if (state == 0b0001) //full steps
  {
    //reset timeout on action while we not in OFF state
    if (MenuScreenNum > 0)
    {
      MenuTimeoutTicker = 1;
    }

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
  }
}

void DisplayScreenMenu()
{
  //do not redraw non changed information
  //we reset to 1 this counter on _any_ user-action
  if (MenuTimeoutTicker > 1) return;

  Display.clearDisplay();

  switch (MenuScreenNum)
  {
    //SCREEN1: display current sensor readings
    case 1:
      {
        Display.drawTile(0, 1, 2, SymbolCurTemperature);  //secondline
        Display.drawTile(0, 2, 2, SymbolCurTemperature + 16);  //secondline
        Display.drawTile(0, 5, 2, SymbolCurHumidity);
        Display.drawTile(0, 6, 2, SymbolCurHumidity + 16);
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
        Display.drawTile(0, 3, 2, SymbolTempSet);
        Display.drawTile(0, 4, 2, SymbolTempSet + 16);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(HeaterManualTemp, 1);
      }
      break;
    //SCREEN3: set heater operation mode
    case 3:
      {
        Display.drawTile(0, 3, 2, SymbolModeSet);
        Display.drawTile(0, 4, 2, SymbolModeSet + 16);

        switch (HeaterState)
        {
          case 0:
            Display.drawTile(5, 3, 2, SymbolModeOFF1);
            Display.drawTile(5, 4, 2, SymbolModeOFF2);
            break;
          case 1:
            Display.drawTile(5, 3, 2, SymbolModeON1);
            Display.drawTile(5, 4, 2, SymbolModeON2);
            break;
          case 2:
            Display.drawTile(5, 3, 2, SymbolModeAUTO1);
            Display.drawTile(5, 4, 2, SymbolModeAUTO2);
            break;
          case 3:
            Display.drawTile(5, 3, 2, SymbolModeCALIBRATION);
            Display.drawTile(5, 4, 2, SymbolModeCALIBRATION + 16);
            break;
        }
      }
      break;
    //SCREEN4: temp compensation
    case 4:
      {
        Display.drawTile(0, 3, 2, SymbolTOffsetSet);
        Display.drawTile(0, 4, 2, SymbolTOffsetSet + 16);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(mySensor.settings.tempCorrection);
      }
      break;
    //SCREEN5: display brightness
    case 5:
      {
        Display.drawTile(0, 3, 2, SymbolBrightnessSet);
        Display.drawTile(0, 4, 2, SymbolBrightnessSet + 16);
        Display.setCursor(5, 2);
        Display.setInverseFont(1);
        Display.print(screenBrightness);
        Display.setContrast(map(screenBrightness, 0, 10, 0, 250));
      }
      break;
  }
}

void SaveSettings()
{
  EEPROM.put(0, screenBrightness);
  EEPROM.put(1, humidity_offset);
  EEPROM.put(5, mySensor.settings.tempCorrection);
}
