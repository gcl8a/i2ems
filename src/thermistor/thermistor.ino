#include <SD.h>

const int CS_SD = 4;  //chip select for the SD card
String filename;      //for managing filenames
File dataFile;        //dataFile manager

//define which pin the thermistor is on
const int THERMISTOR_PIN = A0;

const uint32_t reportInterval = 1000; //once per second

void setup() 
{
  Serial.begin(115200);
  
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD)) 
  {
    Serial.println("card failed, or not present.");
    while(1) {} //die in a sad, infinite loop
  }
  Serial.println("card initialized.");

  filename = CreateNewFile();

  Serial.print("Writing to file: ");
  Serial.println(filename);
}

void loop() 
{
  while(Serial.available())
  {
    if(Serial.read() == 'S')
    {
      Serial.println("It is now safe to remove the SD card");
      while(1) {} //stop
    }
  }
  
  static uint32_t lastReport = 0;
  uint32_t currTime = millis();
  
  if(currTime - lastReport > reportInterval) //time for a new reading
  {
    lastReport += reportInterval;

    uint32_t adc = analogRead(THERMISTOR_PIN);

    //The student needs to put the proper equations in the next three lines!!!!
    float voltage = 0;      //fine the voltage from the voltage divder equation
    float resistance = 0;   //then find the resistance of the thermistor
    float temperature = 0;  //then the temperature

    String dataString = String(currTime) + '\t' 
                        + String(adc) + '\t'
                        + String(voltage) + '\t'
                        + String(resistance) + '\t'
                        + String(temperature) + '\n';
                        
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
      Serial.println("Error opening file!");
      while(1) {} //die in a sad, infinite loop
    }
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
}

String CreateNewFile(void) //creates a new file of the form therm_##.csv; auto-increments so old files aren't clobbered
{
  Serial.println("Creating new file.");

  int fileCount = 0;
  bool fileSuccess = false;

  char filename[13]; //use 8.3 format 
  while(!fileSuccess)
  {
    sprintf(filename, "therm_%02i.csv", fileCount);
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

