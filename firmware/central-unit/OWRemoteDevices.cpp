/*
   This class is based on
   DS2482 library for Arduino Copyright (C) 2009-2010 Paeae Technologies

   slightly changed to work with only 2 device types: Remote Control Panel and DS18b20 thermosensor
   Modifications (c) 2018-2019 dUkk
*/
#include "Arduino.h"
#include "OWRemoteDevices.h"

OWRemoteDevices::OWRemoteDevices() {}
OWRemoteDevices::OWRemoteDevices(MYDS2482* _DS2482)

{
  _wire = _DS2482;
  TotalSlavesCnt = 0;
}

// initialise the bus starting all slaves enumeration
void OWRemoteDevices::ScanAll(void)
{
  TotalSlavesCnt = 0; // Reset the number of devices when we enumerate wire devices

  for (uint8_t channel = 0; channel < 8; channel++)
  {
    _wire->selectChannel(channel);

    _wire->wireResetSearch();

    while (_wire->wireSearch(SlaveDevices[TotalSlavesCnt].Address))
    {
      if (validAddress(SlaveDevices[TotalSlavesCnt].Address))
      {
        SlaveDevices[TotalSlavesCnt].ErrorsCnt = 0;
        SlaveDevices[TotalSlavesCnt].SWPort = channel;
        if (readPowerSupply(SlaveDevices[TotalSlavesCnt].Address))
          SlaveDevices[TotalSlavesCnt].parasitePwr = true;
        else
          SlaveDevices[TotalSlavesCnt].parasitePwr = false;
        //don't go beyond array
        if (TotalSlavesCnt < MAX_OW_SLAVES) TotalSlavesCnt++;
        else break;
      }
    }
  }
}

// returns the number of devices found on the bus
uint8_t OWRemoteDevices::getDeviceCount(void)
{
  return TotalSlavesCnt;
}

uint16_t OWRemoteDevices::getDeviceErrorsCount(const uint8_t deviceIndex)
{
  if (deviceIndex > TotalSlavesCnt) return 0;

  return SlaveDevices[deviceIndex].ErrorsCnt;
}

uint8_t OWRemoteDevices::getDeviceType(const uint8_t deviceIndex)
{
  if (deviceIndex > TotalSlavesCnt) return 0;
  return SlaveDevices[deviceIndex].Address[0];
}

int32_t OWRemoteDevices::getDeviceHWAddress(const uint8_t deviceIndex)
{
  if (deviceIndex > TotalSlavesCnt) return 0;

  //return first 4 bytes (enough to distiguish between devices on bus)
  return int32_t((unsigned char)(SlaveDevices[deviceIndex].Address[0]) << 24 |
                 (unsigned char)(SlaveDevices[deviceIndex].Address[1]) << 16 |
                 (unsigned char)(SlaveDevices[deviceIndex].Address[2]) << 8 |
                 (unsigned char)(SlaveDevices[deviceIndex].Address[3]));
}

// returns true if address is valid
bool OWRemoteDevices::validAddress(const uint8_t* deviceAddress)
{
  return (_wire->crc8(deviceAddress, 7) == deviceAddress[7]);
}

// attempt to determine if the device at the given address is connected to the bus
// also allows for updating the read scratchpad
bool OWRemoteDevices::isConnected(const uint8_t DeviceIndex, uint8_t* scratchPad)
{
  if (!readScratchPad(DeviceIndex, scratchPad)) return false;

  if (SlaveDevices[DeviceIndex].Address[0] == OURMODEL)
    return (_wire->crc8(scratchPad, 8) == scratchPad[8]);
  else
    return (_wire->crc8(scratchPad, SCRATCHPAD_CRC) == scratchPad[SCRATCHPAD_CRC]);
}

bool OWRemoteDevices::readScratchPad(const uint8_t DeviceIndex, uint8_t* scratchPad)
{
  uint8_t readsize;

  if (DeviceIndex > TotalSlavesCnt) return false;

  if (SlaveDevices[DeviceIndex].Address[0] == OURMODEL) readsize = 9;
  else readsize = 9;

  if (!_wire->selectChannel(SlaveDevices[DeviceIndex].SWPort)) return false;

  // send the reset command and fail fast
  int b = _wire->wireReset();
  if (b == 0) {
    SlaveDevices[DeviceIndex].ErrorsCnt++;
    return false;
  }

  _wire->wireSelect(SlaveDevices[DeviceIndex].Address);
  _wire->wireWriteByte(READSCRATCH);

  // Read struct packet in a simple loop
  for (uint8_t i = 0; i < readsize; i++)
  {
    scratchPad[i] = _wire->wireReadByte();
  }

  b = _wire->wireReset();
  return (b == 1);
}

bool OWRemoteDevices::writeScratchPad(const uint8_t DeviceIndex, const uint8_t* scratchPad)
{
  if (DeviceIndex > TotalSlavesCnt) return false;

  if (!_wire->selectChannel(SlaveDevices[DeviceIndex].SWPort)) return false;

  if (_wire->wireReset() == 0)
  {
    SlaveDevices[DeviceIndex].ErrorsCnt++;
    return false;
  }

  _wire->wireSelect(SlaveDevices[DeviceIndex].Address);
  _wire->wireWriteByte(WRITESCRATCH);

  if (SlaveDevices[DeviceIndex].Address[0] == OURMODEL)
  {
    for (uint8_t i = 0; i < 9; i++) _wire->wireWriteByte(scratchPad[i]);
  }
  else
  {
    _wire->wireWriteByte(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
    _wire->wireWriteByte(scratchPad[LOW_ALARM_TEMP]); // low alarm temp

    // DS1820 and DS18S20 have no configuration register
    if (SlaveDevices[DeviceIndex].Address[0] != DS18S20MODEL) _wire->wireWriteByte(scratchPad[CONFIGURATION]);

    _wire->wireReset();

    // save the newly written values to eeprom
    _wire->wireSelect(SlaveDevices[DeviceIndex].Address);
    //_wire->wireWriteByte(COPYSCRATCH, parasitePwr);
    _wire->wireWriteByte(COPYSCRATCH);
    delay(20);  // <--- added 20ms delay to allow 10ms long EEPROM write operation (as specified by datasheet)
  }

  //if (parasitePwr) delay(10); // 10ms delay
  if (_wire->wireReset() == 0)
  {
    SlaveDevices[DeviceIndex].ErrorsCnt++;
    return false;
  }

  return true;
}

bool OWRemoteDevices::readPowerSupply(const uint8_t* deviceAddress)
{

  bool ret = false;
  _wire->wireReset();
  _wire->wireSelect(deviceAddress);
  _wire->wireWriteByte(READPOWERSUPPLY);
  if (_wire->wireReadBit() == 0) ret = true;
  _wire->wireReset();
  return ret;

}

// set resolution of a device to 9, 10, 11, or 12 bits
// if new resolution is out of range, 9 bits is used.
bool OWRemoteDevices::setResolution(const uint8_t DeviceIndex, uint8_t newResolution)
{
  if (SlaveDevices[DeviceIndex].Address[0] == DS18S20MODEL) return false;
  if (SlaveDevices[DeviceIndex].Address[0] == OURMODEL) return false;

  // ensure same behavior as setResolution(uint8_t newResolution)
  newResolution = constrain(newResolution, 9, 12);

  // return when stored value == new value
  if (getResolution(DeviceIndex) == newResolution) return true;

  ScratchPad scratchPad;
  if (isConnected(DeviceIndex, scratchPad))
  {
    // DS1820 and DS18S20 have no resolution configuration register
    switch (newResolution) {
      case 12:
        scratchPad[CONFIGURATION] = TEMP_12_BIT;
        break;
      case 11:
        scratchPad[CONFIGURATION] = TEMP_11_BIT;
        break;
      case 10:
        scratchPad[CONFIGURATION] = TEMP_10_BIT;
        break;
      case 9:
      default:
        scratchPad[CONFIGURATION] = TEMP_9_BIT;
        break;
    }
    writeScratchPad(DeviceIndex, scratchPad);
    return true;  // new value set
  }

  return false;

}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t OWRemoteDevices::getResolution(const uint8_t DeviceIndex)
{
  // DS1820 and DS18S20 have no resolution configuration register
  if (SlaveDevices[DeviceIndex].Address[0] == DS18S20MODEL) return 12;
  if (SlaveDevices[DeviceIndex].Address[0] == OURMODEL) return 0;

  ScratchPad scratchPad;
  if (!isConnected(DeviceIndex, scratchPad)) return 0;

  switch (scratchPad[CONFIGURATION])
  {
    case TEMP_12_BIT:
      return 12;

    case TEMP_11_BIT:
      return 11;

    case TEMP_10_BIT:
      return 10;

    case TEMP_9_BIT:
    default:
      return 9;
  }
}

// sends command for one device to perform data accuision
bool OWRemoteDevices::requestDataMetering(const uint8_t deviceIndex)
{
  if (deviceIndex > TotalSlavesCnt) return false;

  if (_wire->wireReset() == 0)
  {
    SlaveDevices[deviceIndex].ErrorsCnt++;
    return false;
  }

  _wire->wireSelect(SlaveDevices[deviceIndex].Address);
  //_wire->wireWriteByte(STARTCONVO, parasitePwr);
  _wire->wireWriteByte(STARTCONVO);

  return true;
}

// Fetch temperature for device index
bool OWRemoteDevices::getDSTemperature(const uint8_t deviceIndex, float *tempC)
{
  ScratchPad scratchPad;

  if (deviceIndex > TotalSlavesCnt) return false;

  if (SlaveDevices[deviceIndex].Address[0] == OURMODEL) return false;

  if (!isConnected(deviceIndex, scratchPad)) return false;

  //DS18 sensors
  *tempC = (float)calculateTemperature(SlaveDevices[deviceIndex].Address, scratchPad) * 0.0078125; // C = RAW/128

  return true;
}

bool OWRemoteDevices::GetPanelData(uint8_t deviceIndex, ControlPanelData *dataptr)
{
  ScratchPad scratchPad;

  if (deviceIndex > TotalSlavesCnt) return false;

  if (SlaveDevices[deviceIndex].Address[0] != OURMODEL) return false;

  if (!isConnected(deviceIndex, scratchPad)) return false;

  //from our control panel
  dataptr->CurTemp = float((scratchPad[1] << 8 | scratchPad[0])) / 1000.0;
  dataptr->ReqTemp = float((scratchPad[3] << 8 | scratchPad[2])) / 100.0;
  dataptr->CurHumidity = (int8_t)scratchPad[4];
  dataptr->CurMode = scratchPad[5];
  dataptr->toffset =  float((int8_t)scratchPad[7] << 8 | (int8_t)(scratchPad[6] & 255)) / 100.0;
  return true;
}

bool OWRemoteDevices::SetPanelData(uint8_t deviceIndex, ControlPanelData *dataptr)
{
  ScratchPad scratchPad;
  
  uint16_t val = dataptr->ReqTemp * 100.0f;
  scratchPad[0] = 0;
  scratchPad[1] = 0;
  scratchPad[2] = (byte)val;
  scratchPad[3] = (byte)(val >> 8);
  scratchPad[4] = (byte)(dataptr->CurHumidity & 255); //humidity offset -100+100
  scratchPad[5] = (byte)dataptr->CurMode;
  int16_t val2 = dataptr->toffset * 100.0f;  //temperature offset -5.0+5.0
  scratchPad[6] = (byte)(val2 & 255);
  scratchPad[7] = (byte)(val2 >> 8);
  scratchPad[8] = _wire->crc8(scratchPad, 8);
  if(writeScratchPad(deviceIndex, scratchPad)) return true;
  
  return false;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t OWRemoteDevices::calculateTemperature(const uint8_t* deviceAddress, uint8_t* scratchPad)
{

  int16_t fpTemperature =
    (((int16_t) scratchPad[TEMP_MSB]) << 11) |
    (((int16_t) scratchPad[TEMP_LSB]) << 3);

  /*
    DS1820 and DS18S20 have a 9-bit temperature register.

    Resolutions greater than 9-bit can be calculated using the data from
    the temperature, and COUNT REMAIN and COUNT PER °C registers in the
    scratchpad.  The resolution of the calculation depends on the model.

    While the COUNT PER °C register is hard-wired to 16 (10h) in a
    DS18S20, it changes with temperature in DS1820.

    After reading the scratchpad, the TEMP_READ value is obtained by
    truncating the 0.5°C bit (bit 0) from the temperature data. The
    extended resolution temperature can then be calculated using the
    following equation:

                                  COUNT_PER_C - COUNT_REMAIN
    TEMPERATURE = TEMP_READ - 0.25 + --------------------------
                                          COUNT_PER_C

    Hagai Shatz simplified this to integer arithmetic for a 12 bits
    value for a DS18S20, and James Cameron added legacy DS1820 support.

    See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
  */

  if (deviceAddress[0] == DS18S20MODEL) {
    fpTemperature = ((fpTemperature & 0xfff0) << 3) - 16 +
                    (
                      ((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) /
                      scratchPad[COUNT_PER_C]
                    );
  }

  return fpTemperature;
}
