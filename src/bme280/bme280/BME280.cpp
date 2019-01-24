/******************************************************************************
SparkFunBME280.cpp
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
//See SparkFunBME280.h for additional topology notes.

/*
 * Edited heavily by gcl. Streamlined for lab and fixed the improper sampling
 */

#include "BME280.h"
#include <math.h>
#include "Wire.h"

//Register names:
#define BME280_DIG_T1_LSB_REG            0x88
#define BME280_DIG_T1_MSB_REG            0x89
#define BME280_DIG_T2_LSB_REG            0x8A
#define BME280_DIG_T2_MSB_REG            0x8B
#define BME280_DIG_T3_LSB_REG            0x8C
#define BME280_DIG_T3_MSB_REG            0x8D
#define BME280_DIG_P1_LSB_REG            0x8E
#define BME280_DIG_P1_MSB_REG            0x8F
#define BME280_DIG_P2_LSB_REG            0x90
#define BME280_DIG_P2_MSB_REG            0x91
#define BME280_DIG_P3_LSB_REG            0x92
#define BME280_DIG_P3_MSB_REG            0x93
#define BME280_DIG_P4_LSB_REG            0x94
#define BME280_DIG_P4_MSB_REG            0x95
#define BME280_DIG_P5_LSB_REG            0x96
#define BME280_DIG_P5_MSB_REG            0x97
#define BME280_DIG_P6_LSB_REG            0x98
#define BME280_DIG_P6_MSB_REG            0x99
#define BME280_DIG_P7_LSB_REG            0x9A
#define BME280_DIG_P7_MSB_REG            0x9B
#define BME280_DIG_P8_LSB_REG            0x9C
#define BME280_DIG_P8_MSB_REG            0x9D
#define BME280_DIG_P9_LSB_REG            0x9E
#define BME280_DIG_P9_MSB_REG            0x9F
#define BME280_DIG_H1_REG                0xA1
#define BME280_CHIP_ID_REG               0xD0 //Chip ID
#define BME280_RST_REG                   0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG            0xE1
#define BME280_DIG_H2_MSB_REG            0xE2
#define BME280_DIG_H3_REG                0xE3
#define BME280_DIG_H4_MSB_REG            0xE4
#define BME280_DIG_H4_LSB_REG            0xE5
#define BME280_DIG_H5_MSB_REG            0xE6
#define BME280_DIG_H6_REG                0xE7
#define BME280_CTRL_HUMIDITY_REG        0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG                 0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG            0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG               0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG         0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG         0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG        0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG      0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG      0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG     0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG         0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG         0xFE //Humidity LSB

//****************************************************************************//
//
//  Configuration section
//
//****************************************************************************//
uint8_t BME280::Init(void)
{
    Wire.begin();
    
    //must be in sleep mode to write to registers
    writeRegister(BME280_CTRL_MEAS_REG, 0x00);
    
    //write to the config register
    writeRegister(BME280_CONFIG_REG, 0x00); //no IIR filter
    
    //write humidity settings before temp and pressure
    writeRegister(BME280_CTRL_HUMIDITY_REG, 0x01); //1X oversampling
    
    //then write temperature, pressure and mode settings -- leave in sleep mode
    writeRegister(BME280_CTRL_MEAS_REG, 0x24); //1X oversampling, sleep

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
    ReadCalibrationData();
    
	return readRegister(0xD0); //returns chip ID
}

BME280Reading BME280::TakeReading(void)
{
    //read existing temperature and pressure settings
    uint8_t ctrl_meas = readRegister(BME280_CTRL_MEAS_REG);
    
    //then write one-shot
    writeRegister(BME280_CTRL_MEAS_REG, ctrl_meas | 0x01);
    
    //wait until the measurement is done
    while(readRegister(BME280_STAT_REG) & 0x08) {} //wait until measurement is done
    
    //measurement is done; retrieve the data
    Wire.beginTransmission(bme280Addr);
    Wire.write(BME280_PRESSURE_MSB_REG); //first byte is the MBS pressure
    Wire.endTransmission();
    
    // request bytes from sensor
    uint8_t data[8];
    Wire.requestFrom(bme280Addr, 8);
    uint8_t i = 0;
    while (Wire.available() && (i < 8))  // slave may send more than requested
    {
        data[i++] = Wire.read(); // receive a byte as character
    }

    //OK, now we have a measurement, so calculate the readings
    BME280Reading reading;
    
    //do temperature first; update t_fine, a carryover adjustment parameter
    int32_t t_fine = 0;
    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((data[5] >> 4) & 0x0F);
    reading.temperature = CalcTemperature(adc_T, t_fine);
    
    //then pressure
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);
    reading.pressure = CalcPressure(adc_P, t_fine);
    reading.altitude = CalcAltitude(reading.pressure);

    //then humidity
    int32_t adc_H = ((uint32_t)data[6] << 8) | ((uint32_t)data[7]);
    reading.humidity = CalcHumidity(adc_H, t_fine);

    return reading;
}

//****************************************************************************//
//
//  Utilities
//
//****************************************************************************//

uint8_t BME280::readRegister(const uint8_t& regAddr)
{
    uint8_t result = 0;
    
    //first, tell the chip which register we want to read
    Wire.beginTransmission(bme280Addr);
    Wire.write(regAddr);
    Wire.endTransmission();

    //then read from the chip, which will now send the contents of that register
    Wire.requestFrom(bme280Addr, 1);
    while ( Wire.available() ) // slave may send less than requested
    {
        result = Wire.read(); // receive a byte as a proper uint8_t
    }

	return result;
}

void BME280::writeRegister(const uint8_t& regAddr, const uint8_t& dataToWrite)
{
    //Step 1: beginTransmission() is used to initiate a write connection to the BME280
    Wire.beginTransmission(bme280Addr);
    
    //Step 2: Send the address of the register to be written
    Wire.write(regAddr);
    
    //Step 3: Send the value to write to that register
    Wire.write(dataToWrite);
    
    //Step 4: Close the connection so that the I2C bus is free
    Wire.endTransmission();
}

void BME280::ReadCalibrationData(void)
{
    calibrationData.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
    calibrationData.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
    calibrationData.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));
    
    calibrationData.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
    calibrationData.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
    calibrationData.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
    calibrationData.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
    calibrationData.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
    calibrationData.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
    calibrationData.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
    calibrationData.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
    calibrationData.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));
    
    calibrationData.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
    calibrationData.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
    calibrationData.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
    calibrationData.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
    calibrationData.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
    calibrationData.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));
}

/*
 * Calculation routines below, from the manufacturer's datasheet.
 */

float BME280::CalcTemperature( int32_t adc_T, int32_t& t_fine ) //C
{
    // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
    // t_fine is passed by reference since it is needed in calculations for pressure and humidity
    
    int64_t var1, var2;
    
    var1 = ((((adc_T>>3) - ((int32_t)calibrationData.dig_T1<<1))) * ((int32_t)calibrationData.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calibrationData.dig_T1)) * ((adc_T>>4) - ((int32_t)calibrationData.dig_T1))) >> 12) *
            ((int32_t)calibrationData.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float output = (t_fine * 5 + 128) >> 8;
    
    output = output / 100.0;
    
    return output;
}

float BME280::CalcPressure( int32_t adc_P, int32_t t_fine )
{
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    
    int64_t var1, var2, p_acc;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibrationData.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibrationData.dig_P5)<<17);
    var2 = var2 + (((int64_t)calibrationData.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calibrationData.dig_P3)>>8) + ((var1 * (int64_t)calibrationData.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibrationData.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p_acc = (int32_t)1048576 - adc_P;
    p_acc = (((p_acc<<31) - var2)*3125)/var1;
    var1 = (((int64_t)calibrationData.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
    var2 = (((int64_t)calibrationData.dig_P8) * p_acc) >> 19;
    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibrationData.dig_P7)<<4);
    
    return p_acc / 256.0;
}

float BME280::CalcAltitude( float pressure )
{
    return 0;
}

float BME280::CalcHumidity( int32_t adc_H, int32_t t_fine )
{
    // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46. 333 %RH
    
    int32_t var1;
    var1 = (t_fine - ((int32_t)76800));
    var1 = (((((adc_H << 14) - (((int32_t)calibrationData.dig_H4) << 20) - (((int32_t)calibrationData.dig_H5) * var1)) +
              ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibrationData.dig_H6)) >> 10) * (((var1 * ((int32_t)calibrationData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                           ((int32_t)calibrationData.dig_H2) + 8192) >> 14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibrationData.dig_H1)) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);
    
    return (float)(var1>>12) / 1024.0;
}
