/******************************************************************************
SparkFunBME280.h
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

// Test derived class for base class SparkFunIMU
#ifndef __BME280_H__
#define __BME280_H__

#include "stdint.h"

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct CalibrationData
{
  public:
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	
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
	uint8_t dig_H6;
};

struct BME280Reading
{
    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
    float altitude = 0;
};

//This is the main operational class of the driver.
class BME280
{
protected:
    const uint8_t bme280Addr;
    CalibrationData calibrationData;

    void ReadCalibrationData(void);
    
    //helper functions for converting raw readings to useful values
    float CalcTemperature( int32_t, int32_t& );
    float CalcPressure( int32_t, int32_t );
    float CalcAltitude( float );
    float CalcHumidity(  int32_t, int32_t  );

public:
    BME280( uint8_t addr ) : bme280Addr(addr) {}
    
    uint8_t Init(void);
    BME280Reading TakeReading(void);

    uint8_t readRegister(const uint8_t&);
    void writeRegister(const uint8_t&, const uint8_t& );
};

#endif  // End of __BME280_H__ definition check
