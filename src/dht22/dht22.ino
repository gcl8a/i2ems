#include <DHT.h>

#define SENSOR_INTERVAL 2000UL

#define DHT_PIN_1 8
DHT dht(DHT_PIN_1, DHT22);

void setup() 
{
  Serial.begin(115200);

  dht.begin();
}

void loop() 
{
  static uint32_t lastRead = 0;
  
  if(millis() - lastRead > SENSOR_INTERVAL)
  {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    Serial.print(millis());
    Serial.print('\t');
    Serial.print(temperature);
    Serial.print('\t');
    Serial.print(humidity);
    Serial.print('\n');

    lastRead += SENSOR_INTERVAL;
  }

}
