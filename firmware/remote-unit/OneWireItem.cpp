/*
 This class is based on wonderful library OneWireHub version=2.2.1
 written by authors: Ingmar Splitt, orgua, MarkusLange, Shagrat2

 It is base to work with 1W bus and emulate slave device (our remote unit panel)
 Modifications (c) 2018-2020 dUkk 
*/
#include "OneWireItem.h"

OneWireItem::OneWireItem()
{
    ID[0] = 0x60;
    ID[1] = 0x31;
    ID[2] = 0x02;
    ID[3] = 0xBB;
    ID[4] = 0xCC;
    ID[5] = 0xDD;
    ID[6] = 0xEE;
	
    needApplySettings = false;
    needSensorReadings = true;
    howmuchSleep = 0;
    
    for (int a = 0; a < 9; a++) scratchpad[a] = 0x00;
}

void OneWireItem::sendID(OneWireHub * const hub) const {
    hub->send(ID, 8);
}

uint8_t OneWireItem::crc8(const uint8_t data[], const uint8_t data_size, const uint8_t crc_init)
{
    uint8_t crc = crc_init;

    for (uint8_t index = 0; index < data_size; ++index)
    {
#if defined(__AVR__)
        crc = _crc_ibutton_update(crc, data[index]);
#else
        uint8_t inByte = data[index];
        for (uint8_t bitPosition = 0; bitPosition < 8; ++bitPosition)
        {
            const uint8_t mix = (crc ^ inByte) & static_cast<uint8_t>(0x01);
            crc >>= 1;
            if (mix != 0) crc ^= 0x8C;
            inByte >>= 1;
        }
#endif
    }
    return crc;
}

void OneWireItem::duty(OneWireHub * const hub)
{
  uint8_t cmd;
  if (hub->recv(&cmd, 1)) return;

  switch (cmd)
  {
    case 0x44: //tell main cycle that we need prepare new sensor readings
	    {
		    needSensorReadings = true;
		    howmuchSleep = 8; //we can safely gone to sleep for next 8*2 seconds (total 16sec, our hard defined measurement cycle is 20sec)
	    }
      break;
      
    case 0xBE: // READ our settings and sensor data TO central unit
      {
        hub->send(scratchpad, 9);
        hub->_error = Error::RESET_IN_PROGRESS;
      }
      break;

    case 0x4E: // WRITE our settings FROM central unit to us
      {
        hub->recv(&scratchpad[0], 9);
        hub->_error = Error::RESET_IN_PROGRESS;
        needApplySettings = true; needSensorReadings = true;
      }
      break;

    case 0xB4: // READ POWER SUPPLY
      hub->sendBit(1); // 1: say i am external powered, 0: uses parasite power, 1 is passive, so omit it ...
      break;
  }
}
