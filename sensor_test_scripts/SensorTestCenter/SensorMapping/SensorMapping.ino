//This script will be used to get the values from all of the sensors that we need so we know their optimal ranges for mapping. 
//  Wire each of them idividually and take measurements of their maximum and minimum values. For the servo don't need to check since
//  we are going to be mapping numbers 0 --> 1023 to 0 --> 179 (analogue values --> degrees).

// Here is a list of all of the sensors' mapping ranges we should get familiar with: 
//    - Servo
//        - MAX:      179
//        - NEUTRAL:  90
//        - MIN:      0
//    - Absolute Orientation Sensor
//        - MAX:
//        - NEUTRAL:
//        - MIN:
//    - Accelerometer
//        - MAX:
//        - NEUTRAL:
//        - MIN:
//    - Current Sensor
//        - MAX:
//        - NEUTRAL:
//        - MIN:
//    - GPS
//        - MAX:
//        - NEUTRAL:
//        - MIN:
//    - Altimiter
//        - MAX:
//        - NEUTRAL:
//        - MIN:

// COMMENT OUT THE OTHERS WHEN TESTING WITH /* -> */ 

// GPS ------------------------------------------------------------------------------
#include <Adafruit_GPS.h> //Install "Adafruit_GPS" library and go to File → Examples → Adafruit_GPS → GPS_HardwareSerial_Parsing or GPS_SoftwareSerial_Parsing for more examples
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

uint32_t timer = millis(); // To keep track of time

// Notes:
//  - Serial baud rate must be 115200 so we can read the GPS fast enough and echo without dropping chars, set GPS.begin(9600) which is the default standard for NMEA (National Marine Electronics Association)
//    - RMC (Recommended Minimum Navigation Information)
//        - Time, date, latitude, longitude, speed over ground (knots), course over ground (degrees), magnetic variation (E or W), Hemisphere indicator for latitude (N or S)
//    - GGA (Global Positioning System Fix Data)
//        - Number of satellites in view, GPS fix quality, altitude, geodial separation, horizontal dilution of precision (HDOP) (calultates the potential error in horizontal position)
//  - I am choosing RMC+GGA for our tests and will output all possible data
// MORE INFO -- https://learn.adafruit.com/adafruit-ultimate-gps/parsed-data-output
// WIRING    -- https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
//-------------------------------------------------------------------------------------


// Accelerometer ----------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h> // Install "Adafruit ADXL375" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_ADXL375 → sensortest for more examples

#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define ADXL375_CS 10

// Assign a unique ID to this sensor at the same time 
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

void displayDataRate(void) {
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

// Notes:
//  - FILL IN
// MORE INFO -- https://www.adafruit.com/product/5374
//-------------------------------------------------------------------------------------






void setup() {
//GPS SETUP -----------------------------------------------------------------------
  // Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200); //VERY IMPORTANT
  delay(5000);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600); //VERY IMPORTANT
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
//-----------------------------------------------------------------------------------

//ACCELEROMETER SETUP --------------------------------------------------------------
  Serial.begin(115200);
  while (!Serial);
  Serial.println("ADXL375 Accelerometer Test"); Serial.println("");

  // Initialise the sensor 
  if(!accel.begin())
  {
    // Check to see if issue connecting to the sensor
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  // Display some basic information on this sensor 
  accel.printSensorDetails();
  displayDataRate();
  Serial.println("");
//----------------------------------------------------------------------------------
}

void loop() {
  //gpsRead();
  //altRead();
  //accelRead();
  //currRead();
  //AOSRead();
}

void gpsRead() { // Print all possible data given through RMC+GGA by using Software Serial Parsing
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  }
}

void altRead() {

}

void accelRead() { // Print X, Y, Z accelerations in m/s^2
  // Check for sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2 and is already converted into those units by the library)
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  delay(500);
}

void currRead() {

}

void AOSRead() {

}



