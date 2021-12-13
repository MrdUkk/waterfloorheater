/*
  Bosch BME280 enviroment sensor based on
  1. BME280 Arduino and Teensy Driver by (c) 2015 Marshall Taylor @ SparkFun Electronics, MIT Licensed (https://github.com/sparkfun/BME280_Breakout) 
  2. Official documentation and Official git https://github.com/BoschSensortec/BME280_driver
  +my specific addidions and changes 
*/
#include "myBME280.h"

//Constructor -- Specifies default configuration
BME280::BME280( void )
{
  //Construct with these default settings
  settings.tStandby = 5; //1sec
  settings.filter = 2; //
  settings.tempOverSample = 2;
  settings.pressOverSample = 0;
  settings.humidOverSample = 2;
  settings.tempCorrection = 0.0;
  settings.mode = MODE_NORMAL;
}

bool BME280::begin()
{ 
  //Check communication with IC before anything else
  uint8_t chipID = readRegister(BME280_CHIP_ID_REG); //Should return 0x60
  if (chipID != 0x60)
    return (chipID);

  //reset chip
  writeRegister(BME280_RST_REG, 0xB6);

  //wait till it is bootup
  delay(15);

  //wait till chip is reading cal data
  while(isReadingCalibration()) delay(15);

  //Reading all compensation data, range 0x88:A1, 0xE1:E7
  calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
  calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
  calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

  calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
  calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
  calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
  calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
  calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
  calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
  calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
  calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
  calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

  calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
  calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
  calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
  calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
  calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
  calibration.dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));

  setParams();

  return true;
}

void BME280::setParams()
{
  // making sure sensor is in sleep mode before setting configuration as it otherwise may be ignored
  writeRegister(BME280_CTRL_MEAS_REG, MODE_SLEEP);

  writeRegister(BME280_CTRL_HUMIDITY_REG, settings.humidOverSample);
  writeRegister(BME280_CONFIG_REG, uint8_t((settings.tStandby << 5) | (settings.filter << 2) | 0) );
  writeRegister(BME280_CTRL_MEAS_REG, uint8_t((settings.tempOverSample << 5) | (settings.pressOverSample << 2) | settings.mode));
}

//Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
  uint8_t stat = readRegister(BME280_STAT_REG);
  return (stat & (1 << 3)); //If the measuring bit (3) is set, return true
}

bool BME280::isReadingCalibration(void) 
{
  uint8_t const stat = readRegister(BME280_STAT_REG);

  return (stat & (1 << 0)) != 0;
}

void BME280::refreshMeasurements()
{
  if(settings.mode == MODE_FORCED) writeRegister(BME280_CTRL_MEAS_REG, uint8_t((settings.tempOverSample << 5) | (settings.pressOverSample << 2) | settings.mode));
}

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//
float BME280::readTempC( void )
{
  // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
  // t_fine carries fine temperature as global value

  //get the reading (adc_T);
  uint8_t buffer[3];
  readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
  int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

  //By datasheet, calibrate
  int64_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calibration.dig_T1 << 1))) * ((int32_t)calibration.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)calibration.dig_T1))) >> 12) * ((int32_t)calibration.dig_T3)) >> 14;
  
  t_fine = var1 + var2 + ( ((int32_t(settings.tempCorrection * 100) << 8)) / 5);
  float output = (t_fine * 5 + 128) >> 8;
  output = (output / 100.0f);
  return output;
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280::readFloatHumidity( void )
{
  // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
  // Output value of “47445” represents 47445/1024 = 46. 333 %RH
  uint8_t buffer[2];
  readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
  int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);

  int32_t var1;
  var1 = (t_fine - ((int32_t)76800));
  var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
            ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                         ((int32_t)calibration.dig_H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);

  return (float)(var1 >> 12) / 1024.0;
}

int8_t BME280::readIntHumidity(void)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    
    uint8_t buffer[2];
    readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)calibration.dig_H4) * 1048576);
    var4 = ((int32_t)calibration.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calibration.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calibration.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calibration.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calibration.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    var5 = (var5 / 4096);
    return (int8_t)(var5);
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
  //define pointer that will point to the external space
  uint8_t i = 0;
  char c = 0;

  Wire.beginTransmission(BMEI2CAddress);
  Wire.write(offset);
  Wire.endTransmission();

  // request bytes from slave device
  Wire.requestFrom(BMEI2CAddress, length);
  while ( (Wire.available()) && (i < length))  // slave may send less than requested
  {
    c = Wire.read(); // receive a byte as character
    *outputPointer = c;
    outputPointer++;
    i++;
  }
}

uint8_t BME280::readRegister(uint8_t offset)
{
  //Return value
  uint8_t result = 0;
  uint8_t numBytes = 1;
  Wire.beginTransmission(BMEI2CAddress);
  Wire.write(offset);
  Wire.endTransmission();

  Wire.requestFrom(BMEI2CAddress, numBytes);
  while (Wire.available() ) // slave may send less than requested
  {
    result = Wire.read(); // receive a byte as a proper uint8_t
  }
  return result;
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  Wire.beginTransmission(BMEI2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  Wire.endTransmission();
}
