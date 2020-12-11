/* 
   Waterfloors Heating System (WHS)
   - main module firmware (ESP32 microcontroller)

   Written by dUkk (c) 2018-2020 Wholesome Software
*/

#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <rom/rtc.h>
#include <driver/rtc_io.h>
#include <driver/adc.h>
#include <RtcDS3231.h>
#include <DS2482.h>
#include "OWRemoteDevices.h"
#include "PIDLib.h"

#define DEBUG 1

#define RTCTS_OFFSET 946684800 //we get time from server at base year 1970, but here we counting from 2000

#define NO_ADDR_ASSIGNED 255

#define HEATER_MODE_OFF 0
#define HEATER_MODE_PIDTUNE 1
#define HEATER_MODE_PID_SINGLEZONE 2
#define HEATER_MODE_PID_MULTIZONE 3

#define SSID_NAME "bssid"
#define SSID_PASSWORD "wifipassword"
#define HTTP_HOST "serverhost.name"
#define OUR_HOSTNAME "wfh"

#define RUNCYCLE_FREQ 20 //panels and sensors data grab cycle and PID compute cycle will run at this interval (in seconds)
#define VALVE_PWM_CYCLES 6 //TRIAC opening or closing valve window. VALVE_PWM_CYCLES*RUNCYCLE_FREQ (in seconds)

RtcDS3231<TwoWire> Rtc(Wire);
//1wire switch instance  0 = 0x18 address
DS2482 DS2482(0, &Wire);
//slaves
OWRemoteDevices OneWireBus(&DS2482);

RTC_DATA_ATTR uint32_t nextTimeConnect, NextRunCycleTS;
bool first_time;

struct HEATER_PANEL {
  //PID coeff
  double Kp;
  double Ki;
  double Kd;
  //manual corrections of humidity and temperature
  float toffset;
  int8_t hoffset;
  //1Wire addresses for panel and associated termometr (mounted on pipe)
  int32_t PanelAddress;
  int32_t TermalSensorAddress;
  //current channel operation mode
  uint8_t CurrentMode;   //(0= OFF, 1= PID calibration, 2= single temperature every hour, 3= three time ranges distinct temperatures
  //for 3-timezones mode (individual zone ranges and temperature)
  int32_t TZoneStart[3];
  int32_t TZoneEnd[3];
  //user setted temperature. 0..2 is for multi timezone
  double TZoneDesiredTemp[3];
  //for fixed everytime tempepature
  double desiredTemp;
};

struct {
  //for PID
  double input;
  double output;
  //forward pipe flow temperature sensor value
  float ValveTemp;
  //room humidity from remote unit
  uint8_t humidity;
  //number of
  uint32_t OnTimeCount;
  uint16_t PanelErrorsOnBus;
  uint16_t SensorErrorsOnBus;
  uint32_t PanellastOnline;
  uint32_t SensorlastOnline;
  int8_t NumberOfCycles;
  //GPIO pin number of ESP32
  uint8_t Pin;
  //this holds pointer (to HEATER_PANEL[n].) to not recalculate N-times current selected temperature
  double *ptrToCurSettemp; 
  //flag to send autotune calibration data to server
  bool PIDTuneReady;
  //flag to push current config to remote panel
  bool NeedPushConfig;
} RunningValues[4];

struct MY_SETTINGS {
  uint16_t crc16;
  int32_t ConnInterval;
  wifi_power_t txpower;
  char ssid[16];
  char password[32];
  HEATER_PANEL HeaterConfig[4];
} settings;

struct rtcData_WIFI {
  uint16_t crc16;   // 2 bytes
  uint8_t channel;  // 1 byte,   3 in total
  uint8_t bssid[6]; // 6 bytes,  9 in total
  uint8_t padding;  // 1 byte,  10 in total
};

RTC_DATA_ATTR rtcData_WIFI rtcData;

PID PIDHeaterMachinery[4];

unsigned int CRC16(unsigned char *buf, int len);
String extractParam(String& authReq, const String& param, const char delimit);
bool is_between(const int32_t value, const int32_t rangestart, const int32_t rangeend);
uint8_t GetDeviceIndex(const int32_t HWAddress);
double *GetSetTempForZone(uint8_t PanelIdx, uint32_t CurTime);
bool PushToControlPanel(uint8_t PanelIdx, uint8_t deviceIdx);
void ScanOWBus();
void saveSettings();
void loadSettings();
bool InitiateWIFI();
void ShutdownWIFI();
float getVBatt();

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  delay(10);
  Serial.printf("our settings size array %d\n", sizeof(settings));
#endif
  EEPROM.begin(sizeof(settings));
  Wire.begin(SDA, SCL);

  pinMode(26, OUTPUT); //GPIO26 = valve1
  pinMode(25, OUTPUT); //GPIO25 = valve2
  pinMode(33, OUTPUT);//pinMode(33, OUTPUT); //GPIO33 = valve3
  pinMode(32, OUTPUT);//pinMode(32, OUTPUT); //GPIO32 = valve4
  pinMode(35, OUTPUT); //GPIO35 = Pump relay off
  pinMode(34, OUTPUT); //GPIO34 = Pump relay on

  digitalWrite(26, LOW); //Initial state
  digitalWrite(25, LOW); //Initial state
  digitalWrite(33, LOW); //Initial state
  digitalWrite(32, LOW); //Initial state
  digitalWrite(35, LOW); //Initial state
  digitalWrite(34, LOW); //Initial state
 
  loadSettings();
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_OFF);
  btStop();
  
  if (!Rtc.IsDateTimeValid()) 
  {
#ifdef DEBUG
    Serial.print("RTC invalid TS, resetting it\n");
#endif    
      RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
      Rtc.SetDateTime(compiled);
  }

  if (!Rtc.GetIsRunning())
  {
#ifdef DEBUG
    Serial.println("RTC not running, starting\n");
#endif
    Rtc.SetIsRunning(true);
  }
#ifdef DEBUG
  else
    Serial.print("RTC already running\n");
#endif
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

#ifdef DEBUG
  Serial.printf("1W switch reset %d\n", DS2482.reset());
#else
  DS2482.reset();
#endif
  //DS2482.configure(DS2482_CONFIG_APU);

  //wait until all remote control panels is booted up
  //while we are waiting -> illuminate valve control LEDs for one second
  digitalWrite(26, HIGH);
  delay(1000);
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);
  delay(1000);
  digitalWrite(25, LOW);
  digitalWrite(33, HIGH);
  delay(1000);
  digitalWrite(33, LOW);
  digitalWrite(32, HIGH);
  delay(1000);
  digitalWrite(32, LOW);

  ScanOWBus();

  //check is all panels is online?
  for (uint8_t i = 0; i < OneWireBus.getDeviceCount(); i++)
  {
    //if panel is online -> light up its LED
    uint8_t idx = GetDeviceIndex(OneWireBus.getDeviceHWAddress(i));
    if(idx == NO_ADDR_ASSIGNED) continue;
    if (settings.HeaterConfig[idx].PanelAddress != 0 &&
        settings.HeaterConfig[idx].PanelAddress == OneWireBus.getDeviceHWAddress(i))
      digitalWrite(RunningValues[idx].Pin, HIGH);
  }
  delay(5000);

  nextTimeConnect = 0;
  NextRunCycleTS = 0;
  first_time = true;
}

void loop()
{
  RtcDateTime dt = Rtc.GetDateTime();

  if (dt.TotalSeconds() >= NextRunCycleTS)
  {
    char datestring[20];
    snprintf_P(datestring, 20, PSTR("%02u/%02u/%04u %02u:%02u:%02u\r\n"),  dt.Month(), dt.Day(), dt.Year(), dt.Hour(), dt.Minute(), dt.Second());
    Serial.print(datestring);

    //sensors and panels data accqusion may consume more than RUNCYCLE_FREQ seconds. so we must calculate next real delay to call Run cycle monotonically
    NextRunCycleTS = dt.TotalSeconds() + RUNCYCLE_FREQ;
#ifdef DEBUG
    Serial.printf("NextRunCycleTS=%d\r\n", NextRunCycleTS);
#endif

    uint32_t SecondsSinceZZ = (dt.Hour() * 60 * 60) + (dt.Minute() * 60);

    //search again if no devices found
    if (OneWireBus.getDeviceCount() == 0)
    {
      ScanOWBus();
      delay(750); //need because sensors took some time to prepare readings
    }

    //consume data from remote sensors and panels
    ControlPanelData data;
    float tempC;
    for (uint8_t i = 0; i < OneWireBus.getDeviceCount(); i++)
    {
      //map this sensor to our internal idx
      uint8_t idx = GetDeviceIndex(OneWireBus.getDeviceHWAddress(i));
#ifdef DEBUG
      Serial.printf("our panel ID at %d for slave %d\r\n", idx, i);
#endif
      if (idx == NO_ADDR_ASSIGNED) continue; //skip if not found

      if (OneWireBus.getDeviceType(i) == OURMODEL)
      {
        double *ptr = GetSetTempForZone(idx, SecondsSinceZZ);
        //compare OLD (current) settemp with potentially new (by time)
        if(*RunningValues[idx].ptrToCurSettemp != *ptr)
        {
#ifdef DEBUG
          Serial.printf("comparing OLD (cur) settemp with next one DIFFERENT -> force panel pushing config\r\n");
#endif
          //update pointer and set flag
          RunningValues[idx].ptrToCurSettemp = ptr;
          RunningValues[idx].NeedPushConfig=true;
        }
        //check is we need to refresh its settings?
        if(RunningValues[idx].NeedPushConfig)
        {
            if(PushToControlPanel(idx, i))
            {
#ifdef DEBUG
              Serial.printf("CONFIG PUSH success\r\n");
#endif
              RunningValues[idx].NeedPushConfig=false;
            }
#ifdef DEBUG            
            else {
              Serial.printf("CONFIG PUSH ERROR!\r\n");
            }
#endif            
            //delay next operation because panel needs some time to do work (write config values and retrieve sensor readings)
            delay(150);
        }
        //next is obtain values
        if (OneWireBus.GetPanelData(i, &data))
        {
#ifdef DEBUG
          Serial.printf(" slave #%d (addr: %d), returned CurrentTemp:%f, SetTemp:%f, Humidity:%d, Mode:%d, TOffset:%f\r\n", i, OneWireBus.getDeviceHWAddress(i), data.CurTemp, data.ReqTemp, data.CurHumidity, data.CurMode, data.toffset);
#endif
          *RunningValues[idx].ptrToCurSettemp = data.ReqTemp;
          settings.HeaterConfig[idx].toffset = data.toffset;
          //settings.HeaterConfig[idx].hoffset
          //if remote mode is changed
          if (data.CurMode != settings.HeaterConfig[idx].CurrentMode)
          {
#ifdef DEBUG
            Serial.printf("panel changed MODE via panel\r\n");
#endif
            settings.HeaterConfig[idx].CurrentMode = data.CurMode;
            //for PID or AUTOTUNE -> initialize it
            if (settings.HeaterConfig[idx].CurrentMode == HEATER_MODE_PIDTUNE)
              PIDHeaterMachinery[idx].ATCancel();
            else if (settings.HeaterConfig[idx].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
            {
              PIDHeaterMachinery[idx].Reset();
              PIDHeaterMachinery[idx].SetMode(AUTOMATIC);
            }
          }
          RunningValues[idx].input = data.CurTemp;
          RunningValues[idx].humidity = data.CurHumidity;
          RunningValues[idx].PanellastOnline = dt.TotalSeconds();
          RunningValues[idx].PanelErrorsOnBus = OneWireBus.getDeviceErrorsCount(i);
#ifdef DEBUG
          Serial.printf(" slave panel #%d (addr: %d), requesting next measurement. status=%d\r\n", i, OneWireBus.getDeviceHWAddress(i), OneWireBus.requestDataMetering(i));
#else
          OneWireBus.requestDataMetering(i); //this will request next measurement and put device to sleep 
#endif
        }
        else
        {
          RunningValues[idx].PanelErrorsOnBus = OneWireBus.getDeviceErrorsCount(i);
#ifdef DEBUG
          Serial.printf(" slave panel #%d (addr: %d), not responding! Errors counter=%d\r\n", i, OneWireBus.getDeviceHWAddress(i), RunningValues[idx].PanelErrorsOnBus);
#endif
        }
      }
      else
      {
        if (OneWireBus.getDSTemperature(i, &tempC))
        {
#ifdef DEBUG
          Serial.printf(" slave #%d SensorTemp=%f\r\n", i, tempC);
#endif
          RunningValues[idx].ValveTemp = tempC;
          RunningValues[idx].SensorlastOnline = dt.TotalSeconds();
          RunningValues[idx].SensorErrorsOnBus = OneWireBus.getDeviceErrorsCount(i);
#ifdef DEBUG
          Serial.printf(" slave sensor #%d requesting next measurement! status=%d\r\n", i, OneWireBus.requestDataMetering(i));
#else
          OneWireBus.requestDataMetering(i);
#endif
        }
        else
        {
          RunningValues[idx].SensorErrorsOnBus = OneWireBus.getDeviceErrorsCount(i);
#ifdef DEBUG
          Serial.printf(" slave sensor #%d (addr: %d), not responding! Errors counter=%d\r\n", i, OneWireBus.getDeviceHWAddress(i), RunningValues[idx].SensorErrorsOnBus);
#endif
        }
      }
    }

    //now spinup PID cycles
    for (uint8_t idx = 0; idx < 4; idx++)
    {
      if (settings.HeaterConfig[idx].CurrentMode == HEATER_MODE_OFF)
      {
        digitalWrite(RunningValues[idx].Pin, LOW);
#ifdef DEBUG
        Serial.printf("HEATER%d MANUAL OFF on GPIO %d\r\n", idx, RunningValues[idx].Pin);
#endif
      }
      //PID loop calibration loop mode
      else if (settings.HeaterConfig[idx].CurrentMode == HEATER_MODE_PIDTUNE)
      {
          if (PIDHeaterMachinery[idx].ATCompute())
          {
#ifdef DEBUG
          Serial.printf("HEATER%d Autotune DONE\r\n", idx);
#endif
            //was done, get P I D values
            RunningValues[idx].PIDTuneReady = true;
            settings.HeaterConfig[idx].CurrentMode = HEATER_MODE_OFF;
            //flag: in NEXT polling cycle we should push new settings to panel
            RunningValues[idx].NeedPushConfig = true;
          }

          if ((RunningValues[idx].output * VALVE_PWM_CYCLES) / 100 >= RunningValues[idx].NumberOfCycles)
          {
#ifdef DEBUG
            Serial.printf("PIDCal HEATER%d PID.input=%f PID.output=%f Cycles=%d ON\r\n", idx, RunningValues[idx].input, RunningValues[idx].output, RunningValues[idx].NumberOfCycles);
#endif
            digitalWrite(RunningValues[idx].Pin, HIGH);
            RunningValues[idx].OnTimeCount += RUNCYCLE_FREQ;
          }
          else
          {
#ifdef DEBUG
            Serial.printf("PIDCal HEATER%d PID.input=%f PID.output=%f Cycles=%d OFF\r\n", idx, RunningValues[idx].input, RunningValues[idx].output, RunningValues[idx].NumberOfCycles);
#endif
            digitalWrite(RunningValues[idx].Pin, LOW);
          }
          RunningValues[idx].NumberOfCycles++;
          if(RunningValues[idx].NumberOfCycles > VALVE_PWM_CYCLES) RunningValues[idx].NumberOfCycles=1;
      }
      //PID heating two modes
      else if (settings.HeaterConfig[idx].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
      {
          PIDHeaterMachinery[idx].Compute(RunningValues[idx].ptrToCurSettemp);

          if ((RunningValues[idx].output * VALVE_PWM_CYCLES) / 100 >= RunningValues[idx].NumberOfCycles)
          {
#ifdef DEBUG
            Serial.printf("HEATER%d PID.input=%f PID.output=%f SetPoint=%f Cycles=%d ON\r\n", idx, RunningValues[idx].input, RunningValues[idx].output, *RunningValues[idx].ptrToCurSettemp, RunningValues[idx].NumberOfCycles);
#endif
            digitalWrite(RunningValues[idx].Pin, HIGH);
            RunningValues[idx].OnTimeCount += RUNCYCLE_FREQ;
          }
          else
          {
#ifdef DEBUG
            Serial.printf("HEATER%d PID.input=%f PID.output=%f SetPoint=%f Cycles=%d OFF\r\n", idx, RunningValues[idx].input, RunningValues[idx].output, *RunningValues[idx].ptrToCurSettemp, RunningValues[idx].NumberOfCycles);
#endif
            digitalWrite(RunningValues[idx].Pin, LOW);
          }
          RunningValues[idx].NumberOfCycles++;
          if(RunningValues[idx].NumberOfCycles > VALVE_PWM_CYCLES) RunningValues[idx].NumberOfCycles=1;
      }    
    }
  }

  //check is we need to connect with server?
  if (dt.TotalSeconds() >= nextTimeConnect)
  {
#ifdef DEBUG
    Serial.printf("curts=%d , %d seconds elapsed, time to connect to server (next %d)\r\n", dt.TotalSeconds(), settings.ConnInterval, nextTimeConnect);
#endif
    if (InitiateWIFI())
    {
      HTTPClient http;
      String payload, value;
      payload.concat(F("/senshandle.php?hid="));
      payload.concat(WiFi.getHostname());
      if (first_time)
      {
        payload += "&boot=";
        payload.concat((int)rtc_get_reset_reason(0));
      }
      http.begin(F(HTTP_HOST), 80, payload);
      http.addHeader("Content-Type", "application/octet-stream");
      //serialize data to XML
      value = "<settings vbatt=\"";
      value.concat(getVBatt());
      value += "\" itemp=\"";
      value.concat(Rtc.GetTemperature().AsFloatDegC());
      value += "\">";
      for (uint8_t i = 0; i < 4; i++)
      {
        value += "<heater id=\"";
        value.concat(i);
        value += "\" paneladdr=\"";
        value.concat(settings.HeaterConfig[i].PanelAddress);
        value += "\" tempsensaddr=\"";
        value.concat(settings.HeaterConfig[i].TermalSensorAddress);
        value += "\" mode=\"";
        value.concat(settings.HeaterConfig[i].CurrentMode);
        value += "\" settemp=\"";
        value.concat(*RunningValues[i].ptrToCurSettemp);
        value += "\" curtemp=\"";
        value.concat(RunningValues[i].input);
        value += "\" toff=\"";
        value.concat(settings.HeaterConfig[i].toffset);
        value += "\" hoff=\"";
        value.concat(settings.HeaterConfig[i].hoffset);
        value += "\" valvetemp=\"";
        value.concat(RunningValues[i].ValveTemp);
        value += "\" humidity=\"";
        value.concat(RunningValues[i].humidity);
        value += "\" sensorerrors=\"";
        value.concat(RunningValues[i].SensorErrorsOnBus);
        value += "\" panelerrors=\"";
        value.concat(RunningValues[i].PanelErrorsOnBus);
        value += "\" oncounter=\"";
        value.concat(RunningValues[i].OnTimeCount);
        value += "\" slastseen=\"";
        if (RunningValues[i].SensorlastOnline == 0) value.concat(0);
        else value.concat(RunningValues[i].SensorlastOnline + RTCTS_OFFSET);
        value += "\" plastseen=\"";
        if (RunningValues[i].PanellastOnline == 0) value.concat(0);
        else value.concat(RunningValues[i].PanellastOnline + RTCTS_OFFSET);
        if (RunningValues[i].PIDTuneReady)
        {
          RunningValues[i].PIDTuneReady = false;
          value += "\" atkp=\"";
          value.concat(PIDHeaterMachinery[i].ATGetKp());
          value += "\" atki=\"";
          value.concat(PIDHeaterMachinery[i].ATGetKi());
          value += "\" atkd=\"";
          value.concat(PIDHeaterMachinery[i].ATGetKd());
        }
        value += "\" />";
        
        RunningValues[i].OnTimeCount = 0;
      }
      value += "</settings>";
      int httpCode = http.POST(value);
#ifdef DEBUG
      Serial.printf("HTTP response %d\r\n", httpCode);
#endif
      if (httpCode == HTTP_CODE_OK)
      {
        if (first_time) first_time = false;
        payload = http.getString();
        value = extractParam(payload, "TIMESTAMP=", '&');
        if (value.length() > 0)
        {
          RtcDateTime TSsync(value.toInt());
          TSsync -= RTCTS_OFFSET;
          RtcDateTime dt = Rtc.GetDateTime();
#ifdef DEBUG
          Serial.printf("OUR TimeStamp %d server %d\r\n", dt.TotalSeconds(), TSsync.TotalSeconds());
#endif
          //if difference is too big - sync RTC
          if (abs(TSsync.TotalSeconds() - dt.TotalSeconds()) > 10)
          {
            uint32_t elapsed = NextRunCycleTS - dt.TotalSeconds();
            Rtc.SetDateTime(TSsync);
            //need restart our RUN cycle but take care of elapsed already time
            NextRunCycleTS = TSsync.TotalSeconds() + (RUNCYCLE_FREQ - elapsed);
#ifdef DEBUG
            Serial.printf("our clock is skew %d seconds from server timestamp! - RESYNCing (RUNCycle elapsed %d)\r\n", abs(TSsync.TotalSeconds() - dt.TotalSeconds()), elapsed);
#endif
          }
        }
        value = extractParam(payload, "SSID=", '&');
        if (value.length() > 0)
        {
          strcpy(settings.ssid, value.c_str());
        }
        value = extractParam(payload, "PASSWD=", '&');
        if (value.length() > 0)
        {
          strcpy(settings.password, value.c_str());
        }
        value = extractParam(payload, "CONINT=", '&');
        if (value.length() > 0)
        {
          settings.ConnInterval = value.toInt();
          if (settings.ConnInterval < 10) settings.ConnInterval = 10;
          else if (settings.ConnInterval > 10000) settings.ConnInterval = 10000;
        }
        value = extractParam(payload, "TXPWR=", '&');
        if (value.length() > 0)
        {
          switch (value.toInt())
          {
            case 0:
              settings.txpower = WIFI_POWER_MINUS_1dBm;
              break;
            case 1:
              settings.txpower = WIFI_POWER_2dBm;
              break;
            case 2:
              settings.txpower = WIFI_POWER_5dBm;
              break;
            case 3:
              settings.txpower = WIFI_POWER_7dBm;
              break;
            case 4:
              settings.txpower = WIFI_POWER_8_5dBm;
              break;
            case 5:
              settings.txpower = WIFI_POWER_11dBm;
              break;
            case 6:
              settings.txpower = WIFI_POWER_13dBm;
              break;
            case 7:
              settings.txpower = WIFI_POWER_15dBm;
              break;
            case 8:
              settings.txpower = WIFI_POWER_17dBm;
              break;
            case 9:
              settings.txpower = WIFI_POWER_18_5dBm;
              break;
            case 10:
              settings.txpower = WIFI_POWER_19dBm;
              break;
            case 11:
              settings.txpower = WIFI_POWER_19_5dBm;
              break;
          }
        }
        value = extractParam(payload, "SCANBUS=", '&');
        if (value.length() > 0)
        {
#ifdef DEBUG
          Serial.printf("OW bus scan\r\n");
#endif
          DS2482.reset();
          ScanOWBus();
        }
        value = extractParam(payload, "CONFSAVE=", '&');
        if (value.length() > 0)
        {
#ifdef DEBUG
          Serial.printf("eeprom write\r\n");
#endif
          saveSettings();
        }
        value = extractParam(payload, "REBOOT=", '&');
        if (value.length() > 0)
        {
#ifdef DEBUG
          Serial.printf("mcu reset\r\n");
#endif
          ESP.restart();
        }
        //==============================PANEL 1 configuration ===============================
        value = extractParam(payload, "H1MODE=", '&');
        if (value.length() > 0)
        {
          uint8_t prevmode = settings.HeaterConfig[0].CurrentMode;
          settings.HeaterConfig[0].CurrentMode = value.toInt();
          //check is running mode changed
          if (settings.HeaterConfig[0].CurrentMode != prevmode)
          {
            if (settings.HeaterConfig[0].CurrentMode == HEATER_MODE_PIDTUNE)
              PIDHeaterMachinery[0].ATCancel();
            else if (settings.HeaterConfig[0].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
            {
              PIDHeaterMachinery[0].Reset();
              PIDHeaterMachinery[0].SetMode(AUTOMATIC);
            }
            RunningValues[0].NeedPushConfig = true;
#ifdef DEBUG
            Serial.printf("HEATER1 mode changed\r\n");
#endif
          }
        }
        value = extractParam(payload, "H1T1S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneStart[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[0]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H1T1E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneEnd[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[0]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H1T2S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneStart[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[1]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H1T2E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneEnd[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[1]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H1T3S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneStart[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[2]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H1T3E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneEnd[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 TimeZone[2]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H1TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].desiredTemp = value.toFloat();
          RunningValues[0].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER1 temp set %f\r\n", settings.HeaterConfig[0].desiredTemp);
#endif
        }
        value = extractParam(payload, "H1T1TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneDesiredTemp[0] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER1 temp set for timezone [0]\r\n");
#endif
        }                
        value = extractParam(payload, "H1T2TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneDesiredTemp[1] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER1 temp set for timezone [1]\r\n");
#endif
        }                
        value = extractParam(payload, "H1T3TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TZoneDesiredTemp[2] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER1 temp set for timezone [2]\r\n");
#endif
        }
        value = extractParam(payload, "H1CPADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].PanelAddress = value.toInt();
          RunningValues[0].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER1 CP addr set %d\r\n", settings.HeaterConfig[0].PanelAddress);
#endif
        }
        value = extractParam(payload, "H1TSADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].TermalSensorAddress = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER1 temp sensor addr set %d\r\n", settings.HeaterConfig[0].TermalSensorAddress);
#endif
        }
        value = extractParam(payload, "H1TOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].toffset = value.toFloat();
          RunningValues[0].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER1 temp offset set %f\r\n", settings.HeaterConfig[0].toffset);
#endif
        }
        value = extractParam(payload, "H1HOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].hoffset = value.toInt();
          RunningValues[0].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER1 humidity offset set %d\r\n", settings.HeaterConfig[0].hoffset);
#endif
        }
        value = extractParam(payload, "H1KP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].Kp = value.toFloat();
          PIDHeaterMachinery[0].SetTunings(settings.HeaterConfig[0].Kp, settings.HeaterConfig[0].Ki, settings.HeaterConfig[0].Kd);
#ifdef DEBUG
          Serial.printf("HEATER1 KP set %f\r\n", settings.HeaterConfig[0].Kp);
#endif
        }
        value = extractParam(payload, "H1KI=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].Ki = value.toFloat();
          PIDHeaterMachinery[0].SetTunings(settings.HeaterConfig[0].Kp, settings.HeaterConfig[0].Ki, settings.HeaterConfig[0].Kd);
#ifdef DEBUG
          Serial.printf("HEATER1 KI set %f\r\n", settings.HeaterConfig[0].Ki);
#endif
        }
        value = extractParam(payload, "H1KD=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[0].Kd = value.toFloat();
          PIDHeaterMachinery[0].SetTunings(settings.HeaterConfig[0].Kp, settings.HeaterConfig[0].Ki, settings.HeaterConfig[0].Kd);
#ifdef DEBUG
          Serial.printf("HEATER1 KD set %f\r\n", settings.HeaterConfig[0].Kd);
#endif
        }
        //==============================PANEL 2 configuration ===============================
        value = extractParam(payload, "H2MODE=", '&');
        if (value.length() > 0)
        {
          uint8_t prevmode = settings.HeaterConfig[1].CurrentMode;
          settings.HeaterConfig[1].CurrentMode = value.toInt();
          //check is running mode changed
          if (settings.HeaterConfig[1].CurrentMode != prevmode)
          {
            if (settings.HeaterConfig[1].CurrentMode == HEATER_MODE_PIDTUNE)
              PIDHeaterMachinery[1].ATCancel();
            else if (settings.HeaterConfig[1].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
            {
              PIDHeaterMachinery[1].Reset();
              PIDHeaterMachinery[1].SetMode(AUTOMATIC);
            }
            RunningValues[1].NeedPushConfig = true;
#ifdef DEBUG
            Serial.printf("HEATER2 mode changed\r\n");
#endif
          }
        }
        value = extractParam(payload, "H2T1S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneStart[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[0]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H2T1E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneEnd[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[0]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H2T2S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneStart[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[1]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H2T2E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneEnd[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[1]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H2T3S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneStart[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[2]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H2T3E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneEnd[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 TimeZone[2]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H2TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].desiredTemp = value.toFloat();
          RunningValues[1].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER2 temp set %f\r\n", settings.HeaterConfig[1].desiredTemp);
#endif
        }
        value = extractParam(payload, "H2T1TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneDesiredTemp[0] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER2 temp set for timezone [0]\r\n");
#endif
        }                
        value = extractParam(payload, "H2T2TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneDesiredTemp[1] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER2 temp set for timezone [1]\r\n");
#endif
        }                
        value = extractParam(payload, "H2T3TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TZoneDesiredTemp[2] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER2 temp set for timezone [2]\r\n");
#endif
        }
        value = extractParam(payload, "H2CPADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].PanelAddress = value.toInt();
          RunningValues[1].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER2 CP addr set %d\r\n", settings.HeaterConfig[1].PanelAddress);
#endif
        }
        value = extractParam(payload, "H2TSADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].TermalSensorAddress = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER2 temp sensor addr set %d\r\n", settings.HeaterConfig[1].TermalSensorAddress);
#endif
        }
        value = extractParam(payload, "H2TOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].toffset = value.toFloat();
          RunningValues[1].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER2 temp offset set %f\r\n", settings.HeaterConfig[1].toffset);
#endif
        }
        value = extractParam(payload, "H2HOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].hoffset = value.toInt();
          RunningValues[1].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER2 humidity offset set %d\r\n", settings.HeaterConfig[1].hoffset);
#endif
        }
        value = extractParam(payload, "H2KP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].Kp = value.toFloat();
          PIDHeaterMachinery[1].SetTunings(settings.HeaterConfig[1].Kp, settings.HeaterConfig[1].Ki, settings.HeaterConfig[1].Kd);
#ifdef DEBUG
          Serial.printf("HEATER2 KP set %f\r\n", settings.HeaterConfig[1].Kp);
#endif
        }
        value = extractParam(payload, "H2KI=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].Ki = value.toFloat();
          PIDHeaterMachinery[1].SetTunings(settings.HeaterConfig[1].Kp, settings.HeaterConfig[1].Ki, settings.HeaterConfig[1].Kd);
#ifdef DEBUG
          Serial.printf("HEATER2 KI set %f\r\n", settings.HeaterConfig[1].Ki);
#endif
        }
        value = extractParam(payload, "H2KD=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[1].Kd = value.toFloat();
          PIDHeaterMachinery[1].SetTunings(settings.HeaterConfig[1].Kp, settings.HeaterConfig[1].Ki, settings.HeaterConfig[1].Kd);
#ifdef DEBUG
          Serial.printf("HEATER2 KD set %f\r\n", settings.HeaterConfig[1].Kd);
#endif
        }
        //==============================PANEL 3 configuration ===============================
        value = extractParam(payload, "H3MODE=", '&');
        if (value.length() > 0)
        {
          uint8_t prevmode = settings.HeaterConfig[2].CurrentMode;
          settings.HeaterConfig[2].CurrentMode = value.toInt();
          //check is running mode changed
          if (settings.HeaterConfig[2].CurrentMode != prevmode)
          {
            if (settings.HeaterConfig[2].CurrentMode == HEATER_MODE_PIDTUNE)
              PIDHeaterMachinery[2].ATCancel();
            else if (settings.HeaterConfig[2].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
            {
              PIDHeaterMachinery[2].Reset();
              PIDHeaterMachinery[2].SetMode(AUTOMATIC);
            }
            RunningValues[2].NeedPushConfig = true;
#ifdef DEBUG
            Serial.printf("HEATER3 mode changed\r\n");
#endif
          }
        }
        value = extractParam(payload, "H3T1S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneStart[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[0]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H3T1E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneEnd[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[0]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H3T2S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneStart[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[1]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H3T2E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneEnd[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[1]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H3T3S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneStart[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[2]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H3T3E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneEnd[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 TimeZone[2]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H3TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].desiredTemp = value.toFloat();
          RunningValues[2].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER3 temp set %f\r\n", settings.HeaterConfig[2].desiredTemp);
#endif
        }
        value = extractParam(payload, "H3T1TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneDesiredTemp[0] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER3 temp set for timezone [0]\r\n");
#endif
        }                
        value = extractParam(payload, "H3T2TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneDesiredTemp[1] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER3 temp set for timezone [1]\r\n");
#endif
        }                
        value = extractParam(payload, "H3T3TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TZoneDesiredTemp[2] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER3 temp set for timezone [2]\r\n");
#endif
        }
        value = extractParam(payload, "H3CPADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].PanelAddress = value.toInt();
          RunningValues[2].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER3 CP addr set %d\r\n", settings.HeaterConfig[2].PanelAddress);
#endif
        }
        value = extractParam(payload, "H3TSADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].TermalSensorAddress = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER3 temp sensor addr set %d\r\n", settings.HeaterConfig[2].TermalSensorAddress);
#endif
        }
        value = extractParam(payload, "H3TOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].toffset = value.toFloat();
          RunningValues[2].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER3 temp offset set %f\r\n", settings.HeaterConfig[2].toffset);
#endif
        }
        value = extractParam(payload, "H3HOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].hoffset = value.toInt();
          RunningValues[2].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER3 humidity offset set %d\r\n", settings.HeaterConfig[2].hoffset);
#endif
        }
        value = extractParam(payload, "H3KP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].Kp = value.toFloat();
          PIDHeaterMachinery[2].SetTunings(settings.HeaterConfig[2].Kp, settings.HeaterConfig[2].Ki, settings.HeaterConfig[2].Kd);
#ifdef DEBUG
          Serial.printf("HEATER3 KP set %f\r\n", settings.HeaterConfig[2].Kp);
#endif
        }
        value = extractParam(payload, "H3KI=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].Ki = value.toFloat();
          PIDHeaterMachinery[2].SetTunings(settings.HeaterConfig[2].Kp, settings.HeaterConfig[2].Ki, settings.HeaterConfig[2].Kd);
#ifdef DEBUG
          Serial.printf("HEATER3 KI set %f\r\n", settings.HeaterConfig[2].Ki);
#endif
        }
        value = extractParam(payload, "H3KD=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[2].Kd = value.toFloat();
          PIDHeaterMachinery[2].SetTunings(settings.HeaterConfig[2].Kp, settings.HeaterConfig[2].Ki, settings.HeaterConfig[2].Kd);
#ifdef DEBUG
          Serial.printf("HEATER3 KD set %f\r\n", settings.HeaterConfig[2].Kd);
#endif
        }
        //==============================PANEL 4 configuration ===============================
        value = extractParam(payload, "H4MODE=", '&');
        if (value.length() > 0)
        {
          uint8_t prevmode = settings.HeaterConfig[3].CurrentMode;
          settings.HeaterConfig[3].CurrentMode = value.toInt();
          //check is running mode changed
          if (settings.HeaterConfig[3].CurrentMode != prevmode)
          {
            if (settings.HeaterConfig[3].CurrentMode == HEATER_MODE_PIDTUNE)
              PIDHeaterMachinery[3].ATCancel();
            else if (settings.HeaterConfig[3].CurrentMode >= HEATER_MODE_PID_SINGLEZONE)
            {
              PIDHeaterMachinery[3].Reset();
              PIDHeaterMachinery[3].SetMode(AUTOMATIC);
            }
            RunningValues[3].NeedPushConfig = true;
#ifdef DEBUG
            Serial.printf("HEATER4 mode changed\r\n");
#endif
          }
        }
        value = extractParam(payload, "H4T1S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneStart[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[0]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H4T1E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneEnd[0] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[0]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H4T2S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneStart[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[1]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H4T2E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneEnd[1] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[1]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H4T3S=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneStart[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[2]Start changed\r\n");
#endif
        }
        value = extractParam(payload, "H4T3E=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneEnd[2] = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 TimeZone[2]End changed\r\n");
#endif
        }
        value = extractParam(payload, "H4TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].desiredTemp = value.toFloat();
          RunningValues[3].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER4 temp set %f\r\n", settings.HeaterConfig[3].desiredTemp);
#endif
        }
        value = extractParam(payload, "H4T1TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneDesiredTemp[0] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER4 temp set for timezone [0]\r\n");
#endif
        }                
        value = extractParam(payload, "H4T2TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneDesiredTemp[1] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER4 temp set for timezone [1]\r\n");
#endif
        }                
        value = extractParam(payload, "H4T3TEMP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TZoneDesiredTemp[2] = value.toFloat();
#ifdef DEBUG
          Serial.printf("HEATER4 temp set for timezone [2]\r\n");
#endif
        }
        value = extractParam(payload, "H4CPADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].PanelAddress = value.toInt();
          RunningValues[3].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER4 CP addr set %d\r\n", settings.HeaterConfig[3].PanelAddress);
#endif
        }
        value = extractParam(payload, "H4TSADDR=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].TermalSensorAddress = value.toInt();
#ifdef DEBUG
          Serial.printf("HEATER4 temp sensor addr set %d\r\n", settings.HeaterConfig[3].TermalSensorAddress);
#endif
        }
        value = extractParam(payload, "H4TOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].toffset = value.toFloat();
          RunningValues[3].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER4 temp offset set %f\r\n", settings.HeaterConfig[3].toffset);
#endif
        }
        value = extractParam(payload, "H4HOFFSET=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].hoffset = value.toInt();
          RunningValues[3].NeedPushConfig = true;
#ifdef DEBUG
          Serial.printf("HEATER4 humidity offset set %d\r\n", settings.HeaterConfig[3].hoffset);
#endif
        }
        value = extractParam(payload, "H4KP=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].Kp = value.toFloat();
          PIDHeaterMachinery[3].SetTunings(settings.HeaterConfig[3].Kp, settings.HeaterConfig[3].Ki, settings.HeaterConfig[3].Kd);
#ifdef DEBUG
          Serial.printf("HEATER4 KP set %f\r\n", settings.HeaterConfig[3].Kp);
#endif
        }
        value = extractParam(payload, "H4KI=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].Ki = value.toFloat();
          PIDHeaterMachinery[3].SetTunings(settings.HeaterConfig[3].Kp, settings.HeaterConfig[3].Ki, settings.HeaterConfig[3].Kd);
#ifdef DEBUG
          Serial.printf("HEATER4 KI set %f\r\n", settings.HeaterConfig[3].Ki);
#endif
        }
        value = extractParam(payload, "H4KD=", '&');
        if (value.length() > 0)
        {
          settings.HeaterConfig[3].Kd = value.toFloat();
          PIDHeaterMachinery[3].SetTunings(settings.HeaterConfig[3].Kp, settings.HeaterConfig[3].Ki, settings.HeaterConfig[3].Kd);
#ifdef DEBUG
          Serial.printf("HEATER4 KD set %f\r\n", settings.HeaterConfig[3].Kd);
#endif
        }
      }
      //set next connect timestamp
      dt = Rtc.GetDateTime();
      nextTimeConnect = dt.TotalSeconds() + settings.ConnInterval;
    }
    //missed connect let's retry fastly
    else
    {
#ifdef DEBUG
      Serial.printf("missing WIFI connection, will retry 10seconds later\r\n");
#endif
      dt = Rtc.GetDateTime();
      nextTimeConnect = dt.TotalSeconds() + 10;
    }
    ShutdownWIFI();
  }

  //determine how much seconds we allowed to powerdown MCU?
  //either WIFI or RUN cycles come first
  int32_t seconds = 0;
#ifdef DEBUG
  Serial.printf("nextTimeConnect=%d  NextRunCycleTS=%d\r\n", nextTimeConnect, NextRunCycleTS);
#endif
  if (nextTimeConnect > NextRunCycleTS)
  {
    seconds = NextRunCycleTS - dt.TotalSeconds();
  }
  else
  {
    seconds = nextTimeConnect - dt.TotalSeconds();
  }

  if (seconds > 0)
  {
#ifdef DEBUG
    Serial.printf("allowed powerdown for %d seconds\r\n", seconds);
#endif
    esp_sleep_enable_timer_wakeup(seconds * 1000000);

#ifdef DEBUG
    for (int a = 0; a < 4; a++) Serial.printf("[%d] = %d\r\n", a, digitalRead(RunningValues[a].Pin));
#endif    
    if (digitalRead(GPIO_NUM_26))
      gpio_hold_en(GPIO_NUM_26);
    if (digitalRead(GPIO_NUM_25))
      gpio_hold_en(GPIO_NUM_25);
    if (digitalRead(GPIO_NUM_33))
      gpio_hold_en(GPIO_NUM_33);
    if (digitalRead(GPIO_NUM_32))
      gpio_hold_en(GPIO_NUM_32);

    gpio_deep_sleep_hold_en();
    WiFi.mode(WIFI_OFF);

    esp_light_sleep_start();
#ifdef DEBUG
    Serial.println("waked from powerdown GPIO states:\r\n");
    for (int a = 0; a < 4; a++) Serial.printf("[%d] = %d\r\n", a, digitalRead(RunningValues[a].Pin));
#endif

    gpio_hold_dis(GPIO_NUM_26);
    gpio_hold_dis(GPIO_NUM_25);
    gpio_hold_dis(GPIO_NUM_33);
    gpio_hold_dis(GPIO_NUM_32);

#ifdef DEBUG
    Serial.println("after gpio_hold_dis\r\n");
    for (int a = 0; a < 4; a++) Serial.printf("[%d] = %d\r\n", a, digitalRead(RunningValues[a].Pin));
#endif
  }
}


double *GetSetTempForZone(uint8_t PanelIdx, uint32_t CurTime)
{
    if(settings.HeaterConfig[PanelIdx].CurrentMode == HEATER_MODE_PID_MULTIZONE)
    {
        for(uint8_t a=0;a<3;a++)
        {
            if(is_between(CurTime, settings.HeaterConfig[PanelIdx].TZoneStart[a], settings.HeaterConfig[PanelIdx].TZoneEnd[a])) 
            {
#ifdef DEBUG
                  Serial.printf("HEATER%d matched timezone from %d to %d  SetPoint=%f\r\n", PanelIdx, settings.HeaterConfig[PanelIdx].TZoneStart[a], settings.HeaterConfig[PanelIdx].TZoneEnd[a], settings.HeaterConfig[PanelIdx].TZoneDesiredTemp[a]);
#endif
                  return &(settings.HeaterConfig[PanelIdx].TZoneDesiredTemp[a]);
            }
        }        
    }

#ifdef DEBUG
    Serial.printf("HEATER%d default SetPoint=%f\r\n", PanelIdx, settings.HeaterConfig[PanelIdx].desiredTemp);
#endif

    return &(settings.HeaterConfig[PanelIdx].desiredTemp);
}

uint8_t GetDeviceIndex(const int32_t HWAddress)
{
  for (uint8_t idx = 0; idx < 4; idx++)
  {
    if (settings.HeaterConfig[idx].PanelAddress == HWAddress ||
        settings.HeaterConfig[idx].TermalSensorAddress == HWAddress) return idx;
  }

  return NO_ADDR_ASSIGNED;
}

bool PushToControlPanel(uint8_t PanelIdx, uint8_t deviceIdx)
{
  ControlPanelData newdata;

#ifdef DEBUG
  Serial.printf("control panel %d need to refresh its settings. writing at idx %d\r\n", PanelIdx, deviceIdx );
#endif

  newdata.ReqTemp = *RunningValues[PanelIdx].ptrToCurSettemp;
  newdata.toffset = settings.HeaterConfig[PanelIdx].toffset;
  newdata.CurHumidity = settings.HeaterConfig[PanelIdx].hoffset;
  newdata.CurMode = settings.HeaterConfig[PanelIdx].CurrentMode;
  return OneWireBus.SetPanelData(deviceIdx, &newdata);
}

void ScanOWBus()
{
#ifdef DEBUG
  Serial.print("initiating 1Wire bus scan\n");
#endif
  OneWireBus.ScanAll();
#ifdef DEBUG
  Serial.printf("done. devices count: %d\n", OneWireBus.getDeviceCount());
#endif
  //map devices to our array
  uint8_t PanelIdx = 0, TSIdx = 0;
  for (uint8_t i = 0; i < OneWireBus.getDeviceCount(); i++)
  {
    if (OneWireBus.getDeviceType(i) == OURMODEL)
    {
      //skip silently more than four devices
      if (PanelIdx == 4) continue;
      //this will save address only once
      if (settings.HeaterConfig[PanelIdx].PanelAddress == 0)
      {
#ifdef DEBUG
        Serial.printf("slave #%d (our idx=%d) is panel type, panel address is zero, saving it\r\n", i, PanelIdx);
#endif
        settings.HeaterConfig[PanelIdx].PanelAddress = OneWireBus.getDeviceHWAddress(i);
      }

#ifdef DEBUG
      if(PushToControlPanel(PanelIdx, i))
      {
        Serial.printf("PUSH slave #%d (our idx=%d) success\r\n", i, PanelIdx);
      }
      else
      {
        Serial.printf("PUSH slave #%d (our idx=%d) error!\r\n", i, PanelIdx);
      }
#else
      PushToControlPanel(PanelIdx, i);
#endif	  
      PanelIdx++;
    }
    else
    {
      //skip silently more than four devices
      if (TSIdx == 4) continue;
      //this will save address only once
      if (settings.HeaterConfig[TSIdx].TermalSensorAddress == 0)
      {
#ifdef DEBUG
        Serial.printf("slave #%d is thermalsensor type, address is zero at %d\r\n", i, TSIdx);
#endif
        settings.HeaterConfig[TSIdx].TermalSensorAddress = OneWireBus.getDeviceHWAddress(i);
      }
      //setup thermosensor (resolution and request first measurement)
      OneWireBus.setResolution(i, 12);
      OneWireBus.requestDataMetering(i);
      TSIdx++;
    }
  }
#ifdef DEBUG
  Serial.printf("OWScan end. Panels: %d, TermalSensors: %d\r\n", PanelIdx, TSIdx);
#endif
}

bool InitiateWIFI()
{
  WiFi.mode(WIFI_STA);
  delay(50);
  WiFi.setTxPower(settings.txpower);

  // Try to read WiFi settings from RTC memory
  bool rtcValid = false;
  // Calculate the CRC of what we just read from RTC memory, but skip the first 2 bytes as that's the checksum itself.
  if (rtcData.crc16 == CRC16( ((uint8_t*)&rtcData) + 2, sizeof( rtcData ) - 2 ))
  {
#ifdef DEBUG
    Serial.printf("WIFI begin using RTC old data\r\n");
#endif
    WiFi.begin(settings.ssid, settings.password, rtcData.channel, rtcData.bssid, true);
  }
  else
  {
#ifdef DEBUG
    Serial.printf("WIFI begin using full scan\r\n");
#endif
    WiFi.begin(settings.ssid, settings.password);
  }
  WiFi.setHostname(OUR_HOSTNAME);

  //loop waiting for connection
  int retries = 0;
  while ( WiFi.status() != WL_CONNECTED )
  {
    retries++;
    if ( retries == 100 )
    {
#ifdef DEBUG
      Serial.printf("WIFI connect after 5sec not set\r\n");
#endif
      WiFi.disconnect();
      delay(10);
      WiFi.begin(settings.ssid, settings.password);
    }
    else if ( retries == 300 )
    {
#ifdef DEBUG
      Serial.printf("WIFI connect after 15sec not set\r\n");
#endif
      return false;
    }
    delay(50);
  }

#ifdef DEBUG
  Serial.println("wifi connected, IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("BSSID: %s\n", WiFi.BSSIDstr().c_str());
#endif
  //connected!
  // Write current connection info back to RTC
  rtcData.padding = 0;
  rtcData.channel = WiFi.channel();
  memcpy(rtcData.bssid, WiFi.BSSID(), 6); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.crc16 = CRC16( ((uint8_t*)&rtcData) + 2, sizeof( rtcData ) - 2 );

  return true;
}

void ShutdownWIFI()
{
#ifdef DEBUG
  Serial.printf("WIFI disconnect\r\n");
#endif
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

void loadSettings()
{
  byte* p = (byte*)(void*)&settings;
  for (int i = 0; i < sizeof(settings); i++)
  {
    *p++ = EEPROM.read(i);
  }

  // Check CRC
  if (settings.crc16 == CRC16( ((uint8_t*)&settings) + 2, sizeof( settings ) - 2 ))
  {
#ifdef DEBUG
    Serial.println("settings CRC ok, loading");
#endif
  }
  else
  {
    for (uint8_t a = 0; a < 4; a++)
    {
      settings.HeaterConfig[a].desiredTemp = 16.0;
      settings.HeaterConfig[a].toffset = 0.0;
      settings.HeaterConfig[a].hoffset = 0;
      settings.HeaterConfig[a].Kp = 6.0;
      settings.HeaterConfig[a].Ki = 3.0;
      settings.HeaterConfig[a].Kd = 2.0;
      settings.HeaterConfig[a].PanelAddress = 0;
      settings.HeaterConfig[a].TermalSensorAddress = 0;
      settings.HeaterConfig[a].CurrentMode = HEATER_MODE_OFF;
      for (uint8_t b = 0; b < 3; b++)
      {
          settings.HeaterConfig[a].TZoneStart[b] = 25200;
          settings.HeaterConfig[a].TZoneEnd[b] = 82800;
          settings.HeaterConfig[a].TZoneDesiredTemp[b] = 16.0;
      }
    }
    settings.ConnInterval = 60;
    settings.txpower = WIFI_POWER_2dBm;
    strcpy_P(settings.ssid, PSTR(SSID_NAME));
    strcpy_P(settings.password, PSTR(SSID_PASSWORD));
#ifdef DEBUG
    Serial.println("settings CRC invalid. loading defaults");
#endif
  }

  //common running counters initialize
  for (int a = 0; a < 4; a++)
  {
    RunningValues[a].input = 0.0;
    RunningValues[a].output = 0.0;
    RunningValues[a].ValveTemp = 0.0;
    RunningValues[a].humidity = 0;
    RunningValues[a].PanellastOnline = 0;
    RunningValues[a].SensorlastOnline = 0;
    RunningValues[a].PanelErrorsOnBus = 0;
    RunningValues[a].SensorErrorsOnBus = 0;
    RunningValues[a].OnTimeCount = 0;
    RunningValues[a].NumberOfCycles = 1;
    RunningValues[a].PIDTuneReady = false;
    RunningValues[a].NeedPushConfig = false;
    RunningValues[a].ptrToCurSettemp = &(settings.HeaterConfig[a].desiredTemp); //default to single

    //PID initialize
    PIDHeaterMachinery[a].Initialize(&(RunningValues[a].input), &(RunningValues[a].output), settings.HeaterConfig[a].Kp, settings.HeaterConfig[a].Ki, settings.HeaterConfig[a].Kd, P_ON_M);
    PIDHeaterMachinery[a].SetOutputLimits(0, 100);  // Give output as power percentage
    PIDHeaterMachinery[a].SetMode(AUTOMATIC);
    PIDHeaterMachinery[a].SetSampleTime(RUNCYCLE_FREQ * 1000);
    PIDHeaterMachinery[a].ATInitialize(&(RunningValues[a].input), &(RunningValues[a].output));
    PIDHeaterMachinery[a].ATSetLookbackSec(RUNCYCLE_FREQ);
    PIDHeaterMachinery[a].ATSetNoiseBand(0.5);
    PIDHeaterMachinery[a].ATSetOutputStep(50);
    PIDHeaterMachinery[a].ATSetControlType(1);
  }
  //assign GPIOs pins
  RunningValues[0].Pin = 26;
  RunningValues[1].Pin = 25;
  RunningValues[2].Pin = 33;
  RunningValues[3].Pin = 32;

  rtcData.crc16 = 0xBEEF;
}

void saveSettings()
{
  settings.crc16 = CRC16( ((uint8_t*)&settings) + 2, sizeof( settings ) - 2 );

  const byte* p = (const byte*)(const void*)&settings;

  for (int i = 0; i < sizeof(settings); i++)
  {
    EEPROM.write(i, *p++);
  }
  EEPROM.commit();
}

bool is_between(const int32_t value, const int32_t rangestart, const int32_t rangeend)
{
  if (rangestart <= rangeend)
  {
    return (rangestart <= value) && (rangeend > value);
  }
  //cross midnight
  else
  {
    return (rangestart <= value) || (rangeend > value);
  }
}

unsigned int CRC16(unsigned char *buf, int len)
{
  unsigned int crc = 0x0000;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }

  return crc;
}

String extractParam(String & authReq, const String & param, const char delimit)
{
  int _begin = authReq.indexOf(param);
  if (_begin == -1)
    return "";
  return authReq.substring(_begin + param.length(), authReq.indexOf(delimit, _begin + param.length()));
}

float getVBatt()
{
  int32_t Raw=0;
  float voltage;

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_0db);
  
  //sample average
  for(int i = 0; i < 100; i++) Raw += adc1_get_voltage(ADC1_CHANNEL_0);
  voltage = Raw / 100.0f;

#ifdef DEBUG
    Serial.printf("ADC1 readed %f\r\n", voltage);
#endif
  voltage = (voltage * 1.1f) / 4096.0f;
  voltage = voltage / (180000.0f/(180000.0f + 360000.0f));  //(R2/(R2+R1)
  voltage = roundf(voltage * 100) / 100;
#ifdef DEBUG
    Serial.printf("ADC1 flattened %f\r\n", voltage);
#endif
  
  return voltage;
}
