/* The code below shows the simple interaction between the Altimeter and Accelerometer. More details on where and why 
    I used certain parts of the code can be found in the SensorMappins script.

  - https://www.pjrc.com/teensy/td_libs_Wire.html <-- Good reference to see all of the I2C pins 
*/
#include <Wire.h>

#include <Servo.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO055_SAMPLERATE_DELAY_MS (100) 


/* 
Notes:
  - Can connect as many I2C supported sensors on each of the ports on the Teensy as needed.
  - The libraries for the 3 sensors (AOS, altimeter, and the accelerometer) manage the I2C communications internally,
      so do not need to switch between the transmissions.
  - Here I used the altimeter and the accelerometer on 1 I2C port and the AOS on the other.
      To specify the difference, change the Wire to Wire1 or Wire2!!!!
  -  Be vary careful connecting power and ground and the chips can get easily fried. 
Wiring (excluding 5V and GND):
  - BMP390: 
    - SCL to 19 (Yellow)
    - SDA to 18 (Blue)
  - ADXL375: 
    - SCL to 19 (Yellow)
    - SDA to 18 (Blue)
  - BNO055: (use wire1)
    - SCL to 16 (Yellow)
    - SDA to 17 (Blue)
*/

// servo object declration
Servo ail_l; // left aileron
Servo ail_r; //right aileron
Servo elev_l; // left elevator
Servo elev_r; //right elevator
Servo rudd; // rudder

// servo pos init 
int pos_ail = 90; 
int pos_elev = 90; 
int pos_rudd = 90;

// sensor object declaration
Adafruit_ADXL375 accel = Adafruit_ADXL375(0x53);
Adafruit_BMP3XX altimeter; // default adress set to 0x77 (I2C address)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // WIRE1 <--- VERY IMPORTANT!!!!!! 



void setup() {

    Serial.begin(115200);
    while (!Serial);

    //servos
    ail_l.attach(9); 
    ail_r.attach(10); 
    elev_l.attach(11); 
    elev_r.attach(12);
    rudd.attach(13);

    //start at initial pos
    ail_l.write(pos_ail); 
    ail_r.write(pos_ail);
    elev_l.write(pos_elev);
    elev_r.write(pos_elev);
    rudd.write(pos_rudd);

    if (!altimeter.begin_I2C()) { Serial.println("Could not find altimeter!"); while(1); } 
    if (!accel.begin()) { Serial.println("Could not find accelerometer!"); while(1); }
    if (!bno.begin()) { Serial.println("Could not find AOS!"); while(1); }

    altimeter.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    altimeter.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    altimeter.setOutputDataRate(BMP3_ODR_50_HZ);

}

void loop() {

    // if (!altimeter.performReading()) {
    //   Serial.println("Failed to get reading from altimeter!");
    //   return;
    // }

    float feet = altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;

    // Serial.print("Feet: "); Serial.println(feet);

    sensors_event_t eventACCEL;
    accel.getEvent(&eventACCEL);

    // Display the results (acceleration is measured in m/s^2 and is already converted into those units by the library)
    // Serial.print("X: "); Serial.print(eventACCEL.acceleration.x); Serial.print("  ");
    // Serial.print("Y: "); Serial.print(eventACCEL.acceleration.y); Serial.print("  ");
    // Serial.print("Z: "); Serial.print(eventACCEL.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

    sensors_event_t eventAOS;
    bno.getEvent(&eventAOS);

    // Serial.print("X: "); Serial.print(eventAOS.orientation.x, 4); // The 4 stands for 4 decimal places
    // Serial.print("\tY: "); Serial.print(eventAOS.orientation.y, 4);
    // Serial.print("\tZ: "); Serial.println(eventAOS.orientation.z, 4);

    // update elevator position
    pos_elev = new_elev_pos(eventAOS.orientation.y, nominal_o_y(feet), pos_elev, 0, 0, 0, 0);
    elev_l.write(pos_elev);
    elev_r.write(pos_elev);

    delay(100);
  
}

// nominal orientation functions

float nominal_o_x(float alti) { 
    return 0;
}

float nominal_o_y(float alti) { // pitch
    return 0; // plug current altitude into a funciton that gives us a nominal pitch
}

float nominal_o_z(float alti) {
    return 0;
}

// get a new ideal angle for the elevators to approach nominal pitch
int required_elev_pos(float current_pitch, float desired_pitch) {
    float delta_pitch = desired_pitch - current_pitch;
    float new_pos = 0; // plug delta pitch into some function to get how much we need to move elevators
    if (new_pos > 170) { 
        new_pos = 170; 
    } else if (new_pos < 10) {
        new_pos = 10;
    }
    return int(new_pos); 
}

// get a new position (smoothed) for the elevators given current and nominal pitch, 
// and the position of all? control surfaces
int new_elev_pos(float current_pitch, float nominal_pitch, int elevator_pos, int left_ail_pos, int right_ail_pos, int rudder_pos) {
    int desired_pos = required_elev_pos(current_pitch, nominal_pitch);
    int delta = desired_pos - elevator_pos; 
    if (delta > 10) { // some arbitrarily large change in elevator angle that we dont want to exceed
        delta = 10;
    } else if (delta < -10) {
        delta = -10;
    }
    return elevator_pos + delta; // add change in pos to current pos
}


// Thoughts 
// Given nominal orientation based on altitude, 
// get to that orientation based on servo positions.
// maybe alter orientation also based on speed
