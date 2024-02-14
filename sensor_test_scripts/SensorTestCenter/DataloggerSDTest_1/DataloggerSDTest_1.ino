// good progress, need to find out how to clear the file, writing to it is chilliard billiards
/* 
  This code demonstrates how to write data from a sensor (using the accelerometer for this example) to a data file. It specifically takes the data from the first
    10 seconds of the code and writes to it. I have not been able to clear the file once the code stops, so after every re-run of the script, it just adds it to the 
    end of the file. If you don't like this just open the text file and clear is and save it and it will reset.
*/

#include <Wire.h>
#include <TimeLib.h> // Library to access the time 
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <SD.h> // Library to access the SD card

Adafruit_ADXL375 accel = Adafruit_ADXL375(0x53);
const int chipSelect = BUILTIN_SDCARD; // The teensy has a pre-named chip number that is classified under "BUILTIN_SDCARD"
int currentMinute;
int currentSecond;
long currentMillis;

bool onlyOnce;  // To display the "Done" message only once

void setup()
{
  Serial.begin(115200);

  onlyOnce = false;
  //SD.remove("datalogSDTest.txt"); // My attempt to remove the text file, could use more testing but not necessary. Clear manually


  while (!Serial); // wait for serial port to connect.
  if (!accel.begin()) { Serial.println("Could not find accelerometer!"); while(1); }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) { // Test to see if the SD card is there
    Serial.println("Card failed, or not present");
    while (1) {}
  }
  Serial.println("card initialized.");
}

void loop()
{
  File dataFile;
  currentMinute = minute();
  currentSecond = second();
  currentMillis = millis() % 1000;
  dataFile = SD.open("datalogSDTest.txt", FILE_WRITE | O_TRUNC);  // Opening/creating the file with the same in quotes and specifying that we will write to it. O_TRUNC is supposed to clear the file but it didn't work



  if (currentSecond < 10 && !onlyOnce) { // Take the first 10 seconds of data
    String dataString = "";

    sensors_event_t eventACCEL;
    accel.getEvent(&eventACCEL);

    long accelX = eventACCEL.acceleration.x;
    long accelY = eventACCEL.acceleration.y;
    long accelZ = eventACCEL.acceleration.z;

    // Getting acceleration and time stamp to appear 
    dataString += "X: " + String(accelX) + " | " 
                + "Y: " + String(accelY) + " | " 
                + "Z: " + String(accelZ) + "\t\t\t"
                + currentMinute + ":" + currentSecond + ":" + currentMillis + "\n";

    if (dataFile) { // writing to text file
      dataFile.println(dataString);
      dataFile.close();
      Serial.println(dataString);
    } else {
      Serial.println("error opening datalogSDTest.txt");
    }
    delay(100);
  }
  
  else {
    if (!onlyOnce) { dataFile.println("---------DONE---------"); }
    onlyOnce = true;
    dataFile.close();
  }
}