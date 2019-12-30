/*
 This class is based on wonderful library OneWireHub version=2.2.1
 written by authors: Ingmar Splitt, orgua, MarkusLange, Shagrat2


 It is a base to work with 1W bus and emulate slave device (our remote unit panel)
 Modifications (c) 2018-2019 dUkk 
*/

#ifndef ONEWIRE_HEATPROTO_H
#define ONEWIRE_HEATPROTO_H

#include "OneWireItem.h"

class HeaterOW : public OneWireItem
{
public:
    uint8_t scratchpad[9];

    HeaterOW(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7);

    void duty(OneWireHub * hub) final;
    void updateCRC(void);

    bool NeedToPollSensor;
    bool NeedToRefreshSettings;
};

#endif
