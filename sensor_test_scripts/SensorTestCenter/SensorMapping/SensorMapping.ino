//Include any libraries here:

// GPS ------------------------------------------------------------------------------
#include <Adafruit_GPS.h> //Install "Adafruit_GPS" library and go to File → Examples → Adafruit_GPS → GPS_HardwareSerial_Parsing or GPS_SoftwareSerial_Parsing for more examples

#include <Adafruit_GPS.h>
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

//Notes:
//  - Serial baud rate must be 115200 so we can read the GPS fast enough and echo without dropping chars, set GPS.begin(9600) which is the default standard for NMEA (National Marine Electronics Association)
//    - RMC (Recommended Minimum Navigation Information)
//        - Time, date, latitude, longitude, speed over ground (knots), course over ground (degrees), magnetic variation (E or W), Hemisphere indicator for latitude (N or S)
//    - GGA (Global Positioning System Fix Data)
//        - Number of satellites in view, GPS fix quality, altitude, geodial separation, horizontal dilution of precision (HDOP) (calultates the potential error in horizontal position)
//  - I am choosing RMC+GGA for our tests and will output all possible data
// MORE INFO -- https://learn.adafruit.com/adafruit-ultimate-gps/parsed-data-output
// WIRING    -- https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
//-------------------------------------------------------------------------------------




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

}

void loop() {
  //gpsRead();
  //altRead();
  //accelRead();
  //currRead();
  //AOSRead();
}

void gpsRead() { //print all possible data given through RMC+GGA by using Software Serial Parsing
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

void accelRead() {

}

void currRead() {

}

void AOSRead() {

}



