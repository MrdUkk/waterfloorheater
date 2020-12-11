/* 
   Waterfloors Heating System (WHS)
   - remote unit module firmware (ATMega328p microcontroller)

   Written by dUkk (c) 2018-2020 Wholesome Software
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <U8x8lib.h>
#include "OneWireHub.h"
#include "OneWireItem.h"
#include "SFBME280_clone.h"

#define FIRMWARE_VERSION 2.0

#define encoderPinA 9  //PB1
#define encoderPinB 8  //PB0
#define encoderPinSW 10  //PB2

U8X8_SSD1306_128X64_NONAME_HW_I2C Display(U8X8_PIN_NONE); //0x3C byte addr
BME280 mySensor;  //0x76 byte addr
auto hub = OneWireHub(2); //in IDE this is 2 , on PCB this is pin 2
auto ourOWsensor = OneWireItem();

const uint8_t SymbolCurTemperature [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x03, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x6b, 0x5c, 0xdc, 0x7b, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t SymbolCurHumidity [32] = {0x00, 0x00, 0x00, 0x80, 0x60, 0xb0, 0x38, 0xfc, 0x7c, 0xb8, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x3e, 0x26, 0x79, 0x7e, 0x31, 0x25, 0x13, 0x0f, 0x00, 0x00, 0x00};
const uint8_t SymbolTempSet [32] = {0x00, 0x80, 0x80, 0x18, 0x90, 0xc0, 0x60, 0x26, 0x26, 0x60, 0xc0, 0x90, 0x18, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x04, 0x7e, 0x30, 0x0e, 0x7a, 0x00, 0x6e, 0x30, 0x00, 0x01, 0x01, 0x00};
const uint8_t SymbolModeSet [32] = {0x00, 0x00, 0x00, 0x06, 0x0e, 0x0e, 0x2e, 0xfe, 0xfe, 0xfe, 0xfc, 0x98, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x3f, 0x2f, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t SymbolTOffsetSet [32] = {0x00, 0x06, 0x06, 0xfe, 0x00, 0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0xfe, 0x02, 0xfe, 0x00, 0x00, 0x00, 0x02, 0x03, 0x03, 0x38, 0x7f, 0x7f, 0x7f, 0x3c, 0x00, 0x3c, 0x7f, 0x7e, 0x7f, 0x3c, 0x00};
const uint8_t SymbolBrightnessSet [32] = {0x00, 0x80, 0x88, 0x18, 0x90, 0xe0, 0x60, 0x3e, 0x3e, 0x60, 0xe0, 0x90, 0x18, 0x88, 0x80, 0x00, 0x01, 0x01, 0x01, 0x38, 0x1b, 0x07, 0x0c, 0xec, 0xec, 0x0c, 0x07, 0x1b, 0x38, 0x01, 0x01, 0x01};
const uint8_t SymbolModeOFF [32] = {0xff, 0x7f, 0x3f, 0x9f, 0x9f, 0x9f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x3f, 0x7f, 0xff, 0xff, 0xfc, 0xf9, 0xfb, 0xf3, 0xf9, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xfc, 0xff};
const uint8_t SymbolModeCALIBRATION [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x6c, 0x6c, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x3e, 0x1f, 0x07, 0x06, 0x06, 0x03, 0x1f, 0x3e, 0x06, 0x02, 0x00, 0x00};
const uint8_t SymbolModeAUTO_SINGLE [32] = {0x00, 0x00, 0xC0, 0x40, 0x00, 0x40, 0x40, 0x40, 0xC0, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x04, 0x06, 0x06, 0x06, 0x07, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t SymbolModeAUTO_MULTI [32] = {0x00, 0x00, 0x24, 0x44, 0x44, 0x44, 0x64, 0x44, 0x54, 0x44, 0x40, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x24, 0x04, 0x00, 0x24, 0x24, 0x24, 0x24, 0x20, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00};
const uint8_t SymbolAddressSet[32] = {0x00, 0x00, 0x00, 0x00, 0x30, 0xB4, 0x44, 0x84, 0x84, 0x44, 0xB4, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x02, 0x04, 0x04, 0x02, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00};

void DisplayScreenMenu();
void SaveSettings();
void GotoSleep();

uint8_t MenuTimeoutTicker = 0; //0 = off menu displaying, 1...149 enable and wait , 150 = turn off menu
uint8_t MenuScreenNum = 0; //0 = menu is off (display is off)
int8_t humidity_offset = 0, CurHumidity = 0;
uint8_t screenBrightness;
uint16_t PollPeriod = 2000;
unsigned long lastTicks = 0;
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

  set_sleep_mode(SLEEP_MODE_STANDBY);
  
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
  if (mySensor.settings.tempCorrection < -5.0f || mySensor.settings.tempCorrection > 5.0f || isnan(mySensor.settings.tempCorrection)) mySensor.settings.tempCorrection = 0.0f;

  Display.setContrast(map(screenBrightness, 0, 10, 3, 250));
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
    Display.print(FIRMWARE_VERSION);
    Display.setCursor(0, 4);
    Display.print(ourOWsensor.ID[1], HEX);
  }

  delay(2500);
  Display.setPowerSave(1);

  //start OneWire communication
  hub.attach(ourOWsensor);

  //FIRST command received from OW should be our configuration
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
      
      //transfer data from 1WIRE object to us
      //empty = float((ourOWsensor.scratchpad[1] << 8 | ourOWsensor.scratchpad[0])) / 100.0f;
      HeaterManualTemp = float(ourOWsensor.scratchpad[3] << 8 | ourOWsensor.scratchpad[2]) / 100.0f;
      humidity_offset = (int8_t)(ourOWsensor.scratchpad[4] & 255);
      HeaterState = ourOWsensor.scratchpad[5];
      mySensor.settings.tempCorrection =  float((int8_t)ourOWsensor.scratchpad[7] << 8 | (int8_t)(ourOWsensor.scratchpad[6] & 255)) / 100.0f;
      SaveSettings();
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
        PollPeriod = 100;
        MenuScreenNum = 1;
        Display.setPowerSave(0);
        DisplayScreenMenu();
      }
      //if user *LONG* pressing initial button (after screen off) we don't want him to randomly increment current displayed screen
      //we want user always after power on land on first screen. so after user DEpressed button -> counter will start counting and after that we will process
      else if(MenuTimeoutTicker > 0) 
      {
        //reset menu displaying counter
        MenuTimeoutTicker = 1;
        //toggle between screens
        if (MenuScreenNum < 6) MenuScreenNum++;
        else MenuScreenNum = 1;
        //request fresh sensor readings each time user pushed button
        ourOWsensor.needSensorReadings=true;
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
        //display is off here. sleep
        else {
              GotoSleep();
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
            Display.drawTile(5, 3, 2, SymbolModeOFF);
            Display.drawTile(5, 4, 2, SymbolModeOFF + 16);
            break;
          case 1:
            Display.drawTile(5, 3, 2, SymbolModeCALIBRATION);
            Display.drawTile(5, 4, 2, SymbolModeCALIBRATION + 16);
            break;
          case 2:
            Display.drawTile(5, 3, 2, SymbolModeAUTO_SINGLE);
            Display.drawTile(5, 4, 2, SymbolModeAUTO_SINGLE + 16);
            break;
          case 3:
            Display.drawTile(5, 3, 2, SymbolModeAUTO_MULTI);
            Display.drawTile(5, 4, 2, SymbolModeAUTO_MULTI + 16);
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
        Display.setContrast(map(screenBrightness, 0, 10, 3, 250));
      }
      break;
    //SCREEN6: panel ID (0x31..0x39)
    case 6:
      {
        Display.drawTile(0, 3, 2, SymbolAddressSet);
        Display.drawTile(0, 4, 2, SymbolAddressSet + 16);
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
    //put us to sleep for 2seconds
    wdt_enable(WDTO_2S);
    WDTCSR |= (1 << WDIE);
    sleep_mode();
    //check is MAIN button pressed?
    if(PINB & (1<<PB2))
    {
      ourOWsensor.howmuchSleep = 0; //turn off sleep mode completely
      break;
    }    
   }
}
