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

// COMMENT OUT THE OTHERS WHEN TESTING WITH /* -> */ (comment out each of the collapsed sections, it will make it easier)

// look at -- how to get live serial chart, any modifications to serial.print?

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

  #define ADXL375_SCK 13  //Define the serial clock
  #define ADXL375_MISO 12 // Define the master in slave out data line
  #define ADXL375_MOSI 11 // Define the slave out master in data line
  #define ADXL375_CS 10   // Define the chip select

  // Assign a unique ID to this sensor at the same time 
  Adafruit_ADXL375 accel = Adafruit_ADXL375(12345); // Assigning a unique ID to the ADXL375 for clarity and to avoid confusion if using multiple sensors 

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
  //  - Measures up to 200 g's of force in X, Y, Z direction
  //  - Here we are using SPI (Serial Peripheral Interface) communication protocol, but it can also use I2C (I will only describe SPI)
  //    - SPI:
  //      - SPI is a synchronous serial communication protocol, which means that it relies on a clock signal for synchronization
  //      - Typically requires 4 wires (the ones defined above), to take data and select a certain chip, moved by the clock pulse
  //          - SCK  (Serial Clock): Clock signal that synchronizes the data transmission between the master and the slave devices. 
  //          - MOSI (Master Out Slave In): The data line through which the master sends data to the slave device. 
  //          - MISO (Master In Slave Out): The data line through which the slave sends data to the master device. 
  //          - CS   (Chip Select): The signal used to select a specific slave device.
  // MORE INFO -- https://www.adafruit.com/product/5374
  // WIRING    -- https://learn.adafruit.com/adafruit-adxl375/pinouts (Can use JST HS connectors instead of soldering)
//-------------------------------------------------------------------------------------

// Altimiter --------------------------------------------------------------------------
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include "Adafruit_BMP3XX.h" // Install "Adafruit BMP3XX" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_BMPXX → simpletest for more examples

  #define BMP_SCK 13  // Define the serial clock
  #define BMP_MISO 12 // Define the master in slave out data line
  #define BMP_MOSI 11 // Define the slave out master in data line
  #define BMP_CS 10   // Define the chip select

  #define SEALEVELPRESSURE_HPA (1013.25) // This defines the standard atmospheric pressure at sea level in hPa (hectopascals), may have to change this at launch site. 

  Adafruit_BMP3XX bmp; // Creating the instance "bmp" so it can be used with the library (this is a label for the sensor)

  // Notes:
  //  - This is a barometric altimiter, which means it uses the barometric formula and uses the change in barometric pressure to calculate its altutude. 
  //  - Relative accuracy of 3 Pascals, which is about +/- 0.25 meters --> 0.82020997 feet
  //  - For absolute height, will need to enter barometeric pressure at sea level (if weather changes) (part of the calibration process)
  // MORE INFO -- https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx
  // WIRING    -- https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/pinouts
  //      - **VERY IMPORTANT** Depending on I2C (simpler) or SPI wiring, it will change the setup in the code (there will be comments clarifying but make sure!)
//-------------------------------------------------------------------------------------

// Absolute Orientation Sensor --------------------------------------------------------
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h> // Install "Adafruit BNO055" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_BNO055 → read_all_data for more examples
  #include <utility/imumaths.h>

  /* 
    You should also assign a unique ID to this sensor for use with
    the Adafruit Sensor API so that you can identify this particular
    sensor in any data logs, etc.  To assign a unique ID, simply
    provide an appropriate value in the constructor below (12345
    is used by default in this example).

    Connections
    ===========
    Connect SCL to analog 5
    Connect SDA to analog 4
    Connect VDD to 3.3-5V DC
    Connect GROUND to common ground
  */

  uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // Set the delay between fresh samples 

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, address
  //                                                          I believe this sensor is 0x28 which means ADR is connected to LOW
  // Notes:
  //  - This is a 9-D0F (9 Degree of Freedom) sensor, which means it has a 3-DOF accelerometer, gyrosocpe, and magnetometer 
  //  - Data Outputs:
  //     - Absolute Orientation (Euler Vector or Quaternion 100Hz)
  //     - Angular Velocity Vector (100Hz)
  //     - Acceleration Vector (100Hz)
  //     - Magnetic Field Strength Vector (20Hz)
  //     - Linear Acceleration Vector (100Hz)
  //     - Gravity Vector (100Hz)
  //     - Temperature (1Hz)
  //  - The sensor includes an on-chip sensor fusion algoritm, which uses data from the accelerometer, gyroscope, and magnetometer to provide accurate orientation information. 
  //  - Uses I2C and UART (Universal Asynchronous Reciever-Transmitter) interfaces
  //  - Does have an on-chip calibration algorithm
  //  - Contains a voltage level shifter, so it can work with 3.3 and 5V
  // MORE INFO -- https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
  // WIRING    -- https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts
//-------------------------------------------------------------------------------------

void setup() {
//GPS SETUP -----------------------------------------------------------------------
  Serial.begin(115200); //VERY IMPORTANT - Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
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

//ALTIMITER SETUP ------------------------------------------------------------------
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
//----------------------------------------------------------------------------------

// Absolute Orientation Sensor ------------------------------------------------------
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
//-----------------------------------------------------------------------------------

}

void loop() {
  //gpsRead();
  //altRead();
  //accelRead();
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

void altRead() { // Prints temperature (F), pressure (hPa), altitude (ft). Temperature and altitude have been modified to fit imperial units.
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  float fahr = (9/5) * bmp.temperature + 32;
  Serial.print(fahr);
  Serial.println(" *F");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  float feet = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  Serial.print(feet);
  Serial.println(" ft");

  Serial.println();
  delay(2000);
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

void AOSRead() { // Prints accelerometer, orientation, magnetic field, gyroscope, rotation vector, linear acceleration, gravity
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// EXTRA FUNCTION NEEDED FOR AOS (COMMENT AS NEEDED) ---------------------------------------
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
//----------------------------------------------------------------------
