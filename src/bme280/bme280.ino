#include<BME280.h>

#define BME280_I2C_ADDRESS  //LEFT FOR THE STUDENT TO ENTER!!!
BME280 bme280(BME280_I2C_ADDRESS);

const uint32_t READ_INTERVAL = 500;

void setup() 
{
  Serial.begin(115200);
  Serial.println("setup()");

  bme280.Init();

  //bme280.writeRegister(<LEFT FOR THE STUDENT>);
  uint8_t regF4 = bme280.readRegister(0xf4);
  Serial.println(regF4, HEX);

  Serial.println("/setup()");
}

void loop() 
{
  static uint32_t lastRead = 0;
  uint32_t currTime = millis();
  if(currTime - lastRead > READ_INTERVAL)
  {
    BME280Reading reading = bme280.TakeReading();

    Serial.print(currTime);
    Serial.print('\t');
    Serial.print(reading.temperature);
    Serial.print('\t');
    Serial.print(reading.humidity);
    Serial.print('\t');
    Serial.print(reading.pressure);
    Serial.print('\t');
    Serial.print(reading.altitude);
    Serial.print('\n');

    lastRead += READ_INTERVAL;
  }
}
