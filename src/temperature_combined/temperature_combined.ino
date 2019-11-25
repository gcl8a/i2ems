//SD card stuff
#include <SD.h>

const int CS_SD = 4;  //chip select for the SD card
String filename;      //for managing filenames
File dataFile;        //dataFile manager

//DHT sensor
#include <DHT.h>

#define DHT_PIN_1 8
DHT dht(DHT_PIN_1, DHT22);

//BME280
#include<BME280.h>

#define BME280_I2C_ADDRESS 0x77 //THIS WON'T WORK! LEFT FOR THE STUDENT TO ENTER!!!
BME280 bme280(BME280_I2C_ADDRESS);

//define which pin the thermistor is on
const int THERMISTOR_PIN = A0;

//define which pin the T<P36 is on
const int TMP36_PIN = A1;

const uint32_t reportInterval = 250; //once per second
uint32_t lastReport = 0;
  
void setup() 
{
  Serial.begin(115200);
  
  Serial.println(F("\nsetup()"));

  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD)) 
  {
    Serial.println(F("card failed, or not present."));
    while(1) {} //die in a sad, infinite loop
  }
  Serial.println(F("card initialized."));

  filename = CreateNewFile();

  Serial.print(F("Filename: "));
  Serial.println(filename);

  //start the dht  
  dht.begin();

  //start the bme280
  bme280.Init();

  //bme280.writeRegister(<LEFT FOR THE STUDENT>);
  uint8_t regF4 = bme280.readRegister(0xf4);
  Serial.println(regF4, HEX);

  lastReport = millis();

  Serial.println(F("/setup()"));
}

void loop() 
{
  while(Serial.available())
  {
    if(Serial.read() == 'S')
    {
      Serial.println(F("It is now safe to remove the SD card"));
      while(1) {} //stop
    }
  }
  
  uint32_t currTime = millis();
  
  if(currTime - lastReport > reportInterval) //time for a new reading
  {
    //read the thermistor
    uint16_t adc_thermistor = analogRead(THERMISTOR_PIN);

    //The student needs to put the proper equations in the next three lines!!!!
    float voltage = adc_thermistor * 5.0 / 1024.0;      //fine the voltage from the voltage divder equation
    float resistance = 10000.0 * (5.0 - voltage) / voltage;   //then find the resistance of the thermistor
    float T_inv = 1.0 / 296.0 + log(resistance / 10000.0) / 4300.0;
    float T_thermistor = 1 / T_inv - 273.15;  //then the temperature

    //read the TMP36
    uint16_t adc_36 = analogRead(TMP36_PIN);
    float T_tmp36 = (((adc_36*5.0) / 1024.0) - 0.5) / 0.01;

    //dht
    dht.read();
    float T_dht = dht.CalcTemperature();
    float RH_dht = dht.CalcHumidity();

    //bme280
    BME280Reading bme280_reading = bme280.TakeReading();

    String dataString = String(currTime) + '\t' 
//                        + String(adc) + '\t'
//                        + String(voltage) + '\t'
//                        + String(resistance) + '\t'
                        + String(T_thermistor) + '\t'
                        + String(T_tmp36) + '\t'
                        + String(T_dht) + '\t'
                        + String(bme280_reading.temperature) + '\n';
                        
    Serial.print(dataString);

    //write to SD card
    dataFile = SD.open(filename, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.print(dataString);
      dataFile.close();
    }
    
    else
    {
      Serial.println(F("Error opening file!"));
      while(1) {} //die in a sad, infinite loop
    }
    
    lastReport += reportInterval;
  }
}

uint32_t CalcDecayTime(uint8_t pin)
{
  pinMode(pin, OUTPUT); //set the pin to be an output
  digitalWrite(pin, HIGH); //set the pin HIGH to charge the capacitor
  delay(2); //delay a couple of milliseconds to let the capacitor charge

  uint32_t startTime = micros(); //start the timer
  pinMode(pin, INPUT); //make the pin an INPUT so the capacitor discharges slowly
  while(digitalRead(pin) == HIGH) {} //wait while the pin is HIGH
  uint32_t endTime = micros(); //after the loop breaks, take the end time

  return endTime - startTime;
}

String CreateNewFile(void) //creates a new file of the form therm_##.csv; auto-increments so old files aren't clobbered
{
  Serial.println(F("Creating new file."));

  int fileCount = 0;
  bool fileSuccess = false;

  char filename[13]; //use 8.3 format 
  while(!fileSuccess)
  {
    sprintf(filename, "therm%03i.csv", fileCount);
    if(SD.exists(filename))
    {
      fileCount++;
    }
    
    else
    {
      fileSuccess = 1;
    }
  }

  return String(filename);
}

