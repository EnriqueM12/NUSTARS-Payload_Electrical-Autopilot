/* 
- This script will be used as a reference for the base code of every sensor. Information about them along with how I got them to work and what the results were 
    are under the notes and post testing notes of each sensor. The main goal is to get a good grasp of each of the sensors before starting to implement them all. 

 - Use the interpolate data to smooth out the curve when viewing in serial plotter.

 - DO NOT USE THIS CODE TO RUN THE SENSORS, USE IT AS A REFERENCE AND LOOK AT THE EXAMPLES FOR UPLOADING IT INSTEAD 

 - look at -- how to get live serial chart, any modifications to serial.print?    
 */

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

  /* Notes:
   - Serial baud rate must be 115200 so we can read the GPS fast enough and echo without dropping chars, set GPS.begin(9600) which is the default standard for NMEA (National Marine Electronics Association)
     - RMC (Recommended Minimum Navigation Information)
         - Time, date, latitude, longitude, speed over ground (knots), course over ground (degrees), magnetic variation (E or W), Hemisphere indicator for latitude (N or S)
     - GGA (Global Positioning System Fix Data)
         - Number of satellites in view, GPS fix quality, altitude, geodial separation, horizontal dilution of precision (HDOP) (calultates the potential error in horizontal position)
   - I am choosing RMC+GGA for our tests and will output all possible data
   - The EN pin is an enable pin, it can turn the GPS on or off
   - GPS uses the UART Protocol (Universal Asynchronous Receiver-Transmitter):
     - TX: Transmitting data
     - RX: Recieving data
   - MUST CONNECT TO PIN 7 (RX) AND 8 (TX) WITH TEENSY!!
   MORE INFO -- https://learn.adafruit.com/adafruit-ultimate-gps/parsed-data-output
   WIRING    -- https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
   ANTENNA (WILL NEED) -- https://learn.adafruit.com/adafruit-ultimate-gps/external-antenna     */
//-------------------------------------------------------------------------------------

// Accelerometer ----------------------------------------------------------------------
  #include <Wire.h> 
  #include <Adafruit_Sensor.h>
  #include <Adafruit_ADXL375.h> // Install "Adafruit ADXL375" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_ADXL375 → sensortest for more examples

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

  /* Notes:
    - Measures up to 200 g's of force in X, Y, Z direction

    - WE ARE NO LONGER USING SPI, I WILL LEAVE THE NOTES IF CURIOUS!!!
    - Here we are using SPI (Serial Peripheral Interface) communication protocol, but it can also use I2C (I will only describe SPI)
      - SPI:
        - SPI is a synchronous serial communication protocol, which means that it relies on a clock signal for synchronization
        - Typically requires 4 wires (the ones defined above), to take data and select a certain chip, moved by the clock pulse
            - SCK  (Serial Clock): Clock signal that synchronizes the data transmission between the master and the slave devices. 
            - MOSI (Master Out Slave In): The data line through which the master sends data to the slave device. 
            - MISO (Master In Slave Out): The data line through which the slave sends data to the master device. 
            - CS   (Chip Select): The signal used to select a specific slave device.
    
    Post Testing:
    - Using the I2C protocol everything runs smoothly
    - With the on LED on the top left:
      - Horizontal motion is X
      - Vertical motion is Y
      - Upwards/lifting motion is Z
    - This is very important data to determine the success of the mission, so will need to conduct tests saving this data so SD card with Teensy. 
    - Connected Black to GND, Red to 5V, SCL (Yellow) to 19, and SDA (Blue) to 18
    - To plot, use the same printing method as found in other sensors: Serial.print(min); Serial.print(", "); Serial.print(max); Serial.print(", "); Serial.println(data);
   MORE INFO -- https://www.adafruit.com/product/5374
   WIRING    -- https://learn.adafruit.com/adafruit-adxl375/arduino (Can use JST HS connectors instead of soldering)
        - **VERY IMPORTANT** As of right now (1/27/24) we are using the I2C protocol for this sensor.     */

//-------------------------------------------------------------------------------------

// Altimiter --------------------------------------------------------------------------
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include "Adafruit_BMP3XX.h" // Install "Adafruit BMP3XX" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_BMPXX → simpletest for more examples

  #define SEALEVELPRESSURE_HPA (1013.25) // This defines the standard atmospheric pressure at sea level in hPa (hectopascals), may have to change this at launch site. 

  Adafruit_BMP3XX bmp; // Creating the instance "bmp" so it can be used with the library (this is a label for the sensor)

  /* Notes:
    - This is a barometric altimiter, which means it uses the barometric formula and uses the change in barometric pressure to calculate its altutude. 
    - Relative accuracy of 3 Pascals, which is about +/- 0.25 meters --> 0.82020997 feet
    - For absolute height, will need to enter barometeric pressure at sea level (if weather changes) (part of the calibration process)
   Post Testing Notes:
    - The test I conducted uses the I2C protocol by connecting SCL to pin 19 (Yellow) and SDA to pin 18 (Blue) of the Teensy 4.1 (and GND/VIN (5V))
    - Do not need to define any of the pins for the wires and can go straight into bmp.begin_I2C() with no inputs (we can input if we are using multiple sensors on the same I2C dataline)
    - In order to see in the serial plotter properly, we can do a series of print statements to 'bound' the output between a max and min by doing Serial.print(min); Serial.print(", "); Serial.print(max); Serial.print(", "); Serial.println(data);
    - In the NUSTARS basement it was at around 400ish ft with excellent precision, we can subtract all our data by a given amount based on on site testing to zero the data.
    - Since the altimiter uses data from barometer, it is important to keep it isolated from the breeze or any fast winds as it could mess up the readings (try blowing on it it will fluctuate a decent amount). 
   MORE INFO -- https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx
   WIRING    -- https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/pinouts
        - **VERY IMPORTANT** As of right now (1/25/24) we are using the I2C protocol for this sensor.     */
//-------------------------------------------------------------------------------------

// Absolute Orientation Sensor --------------------------------------------------------
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h> // Install "Adafruit BNO055" library and (all of the other libraries that come with it) go to File → Examples → Adafruit_BNO055 → sensorapi for more examples
  #include <utility/imumaths.h>

  /* 
    You should also assign a unique ID to this sensor for use with
    the Adafruit Sensor API so that you can identify this particular
    sensor in any data logs, etc.  To assign a unique ID, simply
    provide an appropriate value in the constructor below (12345
    is used by default in this example).
  */

  /* Set the delay between fresh samples */
  #define BNO055_SAMPLERATE_DELAY_MS (100) 

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, 0x28 worked during testing

  /**************************************************************************/
  /*
      Displays some basic information on this sensor from the unified
      sensor API sensor_t type (see Adafruit_Sensor for more information)
  */
  /**************************************************************************/
  void displaySensorDetails(void)
  {
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
  }

  /**************************************************************************/
  /*
      Display some basic info about the sensor status
  */
  /**************************************************************************/
  void displaySensorStatus(void)
  {
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
  }

  /**************************************************************************/
  /*
      Display sensor calibration status
  */
  /**************************************************************************/
  void displayCalStatus(void) {
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
      Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
  }

                                                            
  /* Notes:
    - This is a 9-D0F (9 Degree of Freedom) sensor, which means it has a 3-DOF accelerometer, gyrosocpe, and magnetometer 
    - Data Outputs:
       - Absolute Orientation (Euler Vector or Quaternion 100Hz)
       - Angular Velocity Vector (100Hz)
       - Acceleration Vector (100Hz)
       - Magnetic Field Strength Vector (20Hz)
       - Linear Acceleration Vector (100Hz)
       - Gravity Vector (100Hz)
       - Temperature (1Hz)
    - The sensor includes an on-chip sensor fusion algoritm, which uses data from the accelerometer, gyroscope, and magnetometer to provide accurate orientation information. 
    - Uses I2C and UART (Universal Asynchronous Reciever-Transmitter) interfaces
    - Does have an on-chip calibration algorithm
    - Contains a voltage level shifter, so it can work with 3.3 and 5V
   Post Testing Notes:
    - Using the onboard algorithm that takes the raw data an computes in on board instead of in software. I am using the I2C protocol for this, connecting
        the SCL to pin 19 (Yellow) and SDA to pin 18 (Blue) (along with VIN (5V) and GND)
    - X direction is yaw (0 to 360 degrees)
    - Y direction is pitch (-90 to 90 degrees)
    - Z direction is roll (-90 to 90 degrees)
    - Once it reaches the max, it loops back to 0, watch out at this can cause servo snapping. To fix this look at the ServoAOS_Test_1 script for more details.
    - Samplilng produces no noise in the post-processed data (this shows up to 4 decimal places)
   MORE INFO -- https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
   WIRING    -- https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts
   PDF (VERY GOOD) -- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf
        - **VERY IMPORTANT** As of right now (1/26/24) we are using the I2C protocol for this sensor.           */
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

  delay(1000); }
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

  if (!bmp.begin_I2C()) {   // Just run it like this for now
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

    /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
//-----------------------------------------------------------------------------------



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
  delay(100); // data every 0.1s 
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

void AOSRead() { // Prints orientation of X (Yaw), Y (Pitch), Z (Roll)
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

