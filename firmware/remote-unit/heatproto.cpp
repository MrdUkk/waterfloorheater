/*
 This class is based on wonderful library OneWireHub version=2.2.1
 written by authors: Ingmar Splitt, orgua, MarkusLange, Shagrat2


 It is base to work with 1W bus and emulate slave device (our remote unit panel)
 Modifications (c) 2018-2019 dUkk 
*/
#include "heatproto.h"


HeaterOW::HeaterOW(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7) : OneWireItem(ID1, ID2, ID3, ID4, ID5, ID6, ID7)
{
  NeedToPollSensor = true;
  NeedToRefreshSettings = false;
  for (int a = 0; a < 9; a++) scratchpad[0] = 0x00;
  updateCRC();
}

void HeaterOW::updateCRC()
{
  scratchpad[8] = crc8(scratchpad, 8);
}

void HeaterOW::duty(OneWireHub * const hub)
{
  uint8_t cmd;
  if (hub->recv(&cmd, 1)) return;

  switch (cmd)
  {
    case 0x44: //tell main cycle that we need prepare new sensor readings
      NeedToPollSensor=true;
      break;
      
    case 0xBE: // READ our settings and sensor data TO central unit
      hub->send(scratchpad, 9);
      break;

    case 0x4E: // WRITE settings FROM central unit
      hub->recv(&scratchpad[0], 9);
      if(scratchpad[8] == crc8(scratchpad, 8)) NeedToRefreshSettings=true;
      break;

    case 0xB4: // READ POWER SUPPLY
      hub->sendBit(1); // 1: say i am external powered, 0: uses parasite power, 1 is passive, so omit it ...
      break;

    default:
      hub->raiseSlaveError(cmd);
  }
}
