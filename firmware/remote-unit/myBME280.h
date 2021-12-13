/*
  Bosch BME280 enviroment sensor based on
  1. BME280 Arduino and Teensy Driver by (c) 2015 Marshall Taylor @ SparkFun Electronics, MIT Licensed (https://github.com/sparkfun/BME280_Breakout) 
  2. Official documentation and Official git https://github.com/BoschSensortec/BME280_driver
  +my specific addidions and changes 
*/

#ifndef __BME280_H__
#define __BME280_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

#define BMEI2CAddress (uint8_t)0x76
//Register names:
#define BME280_DIG_T1_LSB_REG                   0x88
#define BME280_DIG_T1_MSB_REG                   0x89
#define BME280_DIG_T2_LSB_REG                   0x8A
#define BME280_DIG_T2_MSB_REG                   0x8B
#define BME280_DIG_T3_LSB_REG                   0x8C
#define BME280_DIG_T3_MSB_REG                   0x8D
#define BME280_DIG_P1_LSB_REG                   0x8E
#define BME280_DIG_P1_MSB_REG                   0x8F
#define BME280_DIG_P2_LSB_REG                   0x90
#define BME280_DIG_P2_MSB_REG                   0x91
#define BME280_DIG_P3_LSB_REG                   0x92
#define BME280_DIG_P3_MSB_REG                   0x93
#define BME280_DIG_P4_LSB_REG                   0x94
#define BME280_DIG_P4_MSB_REG                   0x95
#define BME280_DIG_P5_LSB_REG                   0x96
#define BME280_DIG_P5_MSB_REG                   0x97
#define BME280_DIG_P6_LSB_REG                   0x98
#define BME280_DIG_P6_MSB_REG                   0x99
#define BME280_DIG_P7_LSB_REG                   0x9A
#define BME280_DIG_P7_MSB_REG                   0x9B
#define BME280_DIG_P8_LSB_REG                   0x9C
#define BME280_DIG_P8_MSB_REG                   0x9D
#define BME280_DIG_P9_LSB_REG                   0x9E
#define BME280_DIG_P9_MSB_REG                   0x9F
#define BME280_DIG_H1_REG                               0xA1
#define BME280_CHIP_ID_REG                              0xD0 //Chip ID
#define BME280_RST_REG                                  0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG                   0xE1
#define BME280_DIG_H2_MSB_REG                   0xE2
#define BME280_DIG_H3_REG                               0xE3
#define BME280_DIG_H4_MSB_REG                   0xE4
#define BME280_DIG_H4_LSB_REG                   0xE5
#define BME280_DIG_H5_MSB_REG                   0xE6
#define BME280_DIG_H6_REG                               0xE7
#define BME280_CTRL_HUMIDITY_REG                0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG                                 0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG                    0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG                               0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG                 0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG                 0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG                0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG              0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG              0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG             0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG                 0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG                 0xFE //Humidity LSB

//Class SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
struct SensorSettings
{
  public:
    //tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
    uint8_t tStandby;
    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16    
    uint8_t filter;
    //0 turns off temp sensing
    //1 to 16 are valid over sampling values
    uint8_t tempOverSample;
    //0 turns off pressure sensing
    //1 to 16 are valid over sampling values
    uint8_t pressOverSample;
    //0 turns off humidity sensing
    //1 to 16 are valid over sampling values
    uint8_t humidOverSample;
    uint8_t mode;
    float tempCorrection; // correction of temperature - added to the result
};

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct SensorCalibration
{
  public:
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;

};

//This is the main operational class of the driver.

class BME280
{
  public:
    //settings
    SensorSettings settings;
    SensorCalibration calibration;
    int32_t t_fine;

    //Constructor generates default SensorSettings.
    //(over-ride after construction if desired)
    BME280( void );
    //~BME280() = default;

    bool begin(void);

    void refreshMeasurements();

    void setParams();
    
    bool isMeasuring(void); //Returns true while the device is taking measurement
    bool isReadingCalibration(void);

    //Returns the values
    float readFloatHumidity(void);
    int8_t readIntHumidity(void);
    float readTempC(void);

    //The following utilities read and write

    //ReadRegisterRegion takes a uint8 array address as input and reads
    //a chunk of memory into that array.
    void readRegisterRegion(uint8_t*, uint8_t, uint8_t );
    //readRegister reads one register
    uint8_t readRegister(uint8_t);
    //Writes a byte;
    void writeRegister(uint8_t, uint8_t);
};

#endif  // End of __BME280_H__ definition check
