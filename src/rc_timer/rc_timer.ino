#include <SD.h>

const int RC_PIN = 6;
const int CS_SD = 4; //chip select for the SD card 

const uint32_t reportInterval = 1000; //once per second

File dataFile;

void setup() 
{
  Serial.begin(115200);
  
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD)) 
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  dataFile = SD.open("datalog.txt", FILE_WRITE);
}

void loop() 
{
  static uint32_t lastReport = 0;
  uint32_t currTime = millis();
  
  if(currTime - lastReport > reportInterval) //a second has passed
  {
    lastReport += reportInterval;

    uint32_t decayTime = CalcDecayTime(RC_PIN);

    String dataString = String(currTime) + '\t' + String(decayTime) + '\n';
    Serial.print(dataString);

    // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.print(dataString);
    }
  }
  
  // if the file isn't open, pop up an error:
  else 
  {
    Serial.println("Error writing to file!");
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

