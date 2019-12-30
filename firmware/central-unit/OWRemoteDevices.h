/*
   This class is based on
   DS2482 library for Arduino Copyright (C) 2009-2010 Paeae Technologies

   slightly changed to work with only 2 device types: Remote Control Panel and DS18b20 thermosensor
   Modifications (c) 2018-2019 dUkk
*/

#ifndef OWRemoteDevices_h
#define OWRemoteDevices_h
#include <inttypes.h>
#include <DS2482.h>

struct ControlPanelData 
{
  float CurTemp;
  float ReqTemp;
  int8_t CurHumidity;
  uint8_t CurMode;
  float toffset;
};
    
#define MAX_OW_SLAVES 10

// Model IDs
#define OURMODEL     0x60
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy to EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power

//SCRATCHPAD locations for DS18
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

//DS18 Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

typedef uint8_t DeviceAddress[8];

struct SLAVE_DEVICE
{
  DeviceAddress Address;
  uint8_t SWPort;
  uint16_t ErrorsCnt;
  bool parasitePwr;
};

class OWRemoteDevices
{
public:

    OWRemoteDevices();
    OWRemoteDevices(DS2482*);

    // initialise bus
    void ScanAll(void);

    // returns the number of devices found on the bus
    uint8_t getDeviceCount(void);

    uint16_t getDeviceErrorsCount(const uint8_t deviceIndex);

    //returns device ID (first 4 bytes of 1wire device ID)
    int32_t getDeviceHWAddress(const uint8_t deviceIndex);

    uint8_t getDeviceType(const uint8_t deviceIndex);

    // attempt to determine if the device at the given address is connected to the bus
    // also allows for updating the read scratchpad
    bool isConnected(const uint8_t, uint8_t*);

    bool setResolution(const uint8_t DeviceIndex, uint8_t newResolution);

    // sends command for one device to perform data accuision
    bool requestDataMetering(const uint8_t deviceIndex);
  
    // Get temperature for device index (slow)
    bool getDSTemperature(const uint8_t deviceIndex, float *tempC);

    bool GetPanelData(const uint8_t deviceIndex, ControlPanelData *dataptr);
    void SetPanelData(const uint8_t deviceIndex, ControlPanelData *dataptr);

private:
    typedef uint8_t ScratchPad[9];

    SLAVE_DEVICE SlaveDevices[MAX_OW_SLAVES];
    // count of devices on the bus present since last scan
    uint8_t TotalSlavesCnt;

    // Take a pointer to one wire instance
    DS2482* _wire;

    // returns true if address is valid
    bool validAddress(const uint8_t*);

    // read device's power requirements
    bool readPowerSupply(const uint8_t*);

    uint8_t getResolution(const uint8_t);

    // read device's scratchpad
    bool readScratchPad(const uint8_t, uint8_t*);

    // write device's scratchpad
    bool writeScratchPad(const uint8_t DeviceIndex, const uint8_t* scratchPad);
    
    int16_t calculateTemperature(const uint8_t* deviceAddress, uint8_t* scratchPad);

};
#endif
