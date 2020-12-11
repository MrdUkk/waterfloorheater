/*
 This class is based on wonderful library OneWireHub version=2.2.1
 written by authors: Ingmar Splitt, orgua, MarkusLange, Shagrat2


 It is a base to work with 1W bus and emulate slave device (our remote unit panel)
 Modifications (c) 2018-2020 dUkk 
*/
#ifndef ONEWIRE_HEATPROTO_H
#define ONEWIRE_HEATPROTO_H

#include "OneWireHub.h"
#if defined(__AVR__)
#include <util/crc16.h>
#endif


class OneWireItem
{
public:
    uint8_t scratchpad[9]; //we directly write here for lower system resources

    OneWireItem();
	
	~OneWireItem() = default;
    OneWireItem(const OneWireItem& owItem) = delete;
    OneWireItem(OneWireItem&& owItem) = default;
    OneWireItem& operator=(OneWireItem& owItem) = delete;
    OneWireItem& operator=(const OneWireItem& owItem) = delete;
    OneWireItem& operator=(OneWireItem&& owItem) = delete;

    uint8_t ID[8];  //1Wire this slave address bytes

    void sendID(OneWireHub * hub) const;
	
    void duty(OneWireHub * hub);
	
	  static uint8_t crc8(const uint8_t data[], uint8_t data_size, uint8_t crc_init = 0);

    bool needApplySettings;
    bool needSensorReadings;
    uint8_t howmuchSleep;
};

#endif
