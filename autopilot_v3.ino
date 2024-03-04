#include <Servo.h>  

// ** Sensor libraries **
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// altimeter
#include <"Adafruit_BMP3XX.h>
#define BMP_SCK 13  // Define the serial clock
#define BMP_MISO 12 // Define the master in slave out data line
#define BMP_MOSI 11 // Define the slave out master in data line
#define BMP_CS 10   // Define the chip select
#define SEALEVELPRESSURE_HPA (1013.25) // This defines the standard atmospheric pressure at sea level in hPa (hectopascals), may have to change this at launch site.

// accelerometer
#include <Adafruit_ADXL375.h>
#define ADXL375_SCK 13  //Define the serial clock
#define ADXL375_MISO 12 // Define the master in slave out data line
#define ADXL375_MOSI 11 // Define the slave out master in data line
#define ADXL375_CS 10   // Define the chip select


// vectors
#include <iostream>
#include <vector>
using namespace std;

// servo object declration
Servo ail_l; // left aileron
Servo ail_r; //right aileron
Servo elev_l; // left elevator
Servo elev_r; //right elevator
Servo rudd; // rudder

// sensor object declaration
Adafruit_BMP3XX bmp; // altimeter
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345); // accelerometer

//pins (change as needed)
// Pin 4 to ch1
// Pin 5 to ch2
// Pin 6 to ch4
// pin 6 to ch5
// pin 9 to ail_l
// pin 10 to ail_r
// pin 11 to elev_l
// pin 12 to elev_r
// pin 13 to rudd
// TX/RX PINS TO GPS
// SDL/SCL PINS TO ALTIMETER

//servo pos init 
int pos_ail = 90; 
int pos_elev = 90; 
int pos_rudd = 90;

//channel resolution
int ch1; //ch1 to ailerons cntrl
int ch2; //ch2 to elevators cntrl
int ch4; //ch4 to rudder cntrl
int ch5; //ch5 to manual override switch assuming channel 5 is 2 way switch

// sensor vals
float altitude;

void setup() {
    //servos
    ail_l.attach(9); 
    ail_r.attach(10); 
    elev_l.attach(11); 
    elev_r.attach(12);
    rudd.attach(13);

    //start at initial pos
    ail_l.write(pos_ail); ail_r.write(pos_ail);
    elev_l.write(pos_ail); elev_r.write(pos_ail);
    rudd.write(pos_rudd);

    //channels
    pinMode(4, INPUT); 
    pinMode(5, INPUT); 
    pinMode(6, INPUT); 
    pinMode(7, INPUT); 

    Serial.begin(9600); // Start serial communication


    // altimeter setup -------------
    Serial.begin(115200);

    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("No BMP3 detected");
        while (1);
    }
    // Set up oversampling and filter initialization
    // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    // bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // -----------------------------

    // accelerometer setup ---------
    Serial.begin(115200);

    if(!adxl.begin()) {
        // Check to see if issue connecting to the sensor
        Serial.println("No ADXL375 detected");
        while(1);
    }

    // // Display some basic information on this sensor 
    // adxl.printSensorDetails();
    // displayDataRate();
    // -----------------------------

}

void loop() {
  //get channel vals
    ch1 = pulseIn(4, HIGH);
    ch2 = pulseIn(5, HIGH);
    ch4 = pulseIn(6, HIGH); 
    ch5 = pulseIn(7, HIGH);
    
    //manual override switched on
    if (ch5 > 3000){ //switch flipped to HIGH
    //MANUAL CODE HERE

    }


    else {
        altitude = altRead();

    }
}

float altRead() { // Prints temperature (F), pressure (hPa), altitude (ft). Temperature and altitude have been modified to fit imperial units.
    if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    // Sensor temperature reading
    // Serial.print("Temperature = ");
    // float fahr = (9/5) * bmp.temperature + 32;
    // Serial.print(fahr);
    // Serial.println(" *F");

    // Sensor pressure reading
    // Serial.print("Pressure = ");
    // Serial.print(bmp.pressure / 100.0);
    // Serial.println(" hPa");

    // Sensor altitude reading 
    Serial.print("Approx. Altitude = ");
    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399; // read altitude and convert meters to feet
    Serial.print(alt);
    Serial.println(" ft");

    return alt

}

std::vector<float> accelRead() { 

    // Check for sensor event
    sensors_event_t event;
    adxl.getEvent(&event);

    std::vector<float> accel {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Display the results (acceleration is measured in m/s^2 and is already converted into those units by the library)
    Serial.print("X: "); Serial.print(accel[0]); Serial.print("  ");
    Serial.print("Y: "); Serial.print(accel[1]); Serial.print("  ");
    Serial.print("Z: "); Serial.print(accel[2]); Serial.print("  ");Serial.println("m/s^2 ");

    return accel;
}

float get_nominal_v(float alt) {
    return 10
}
