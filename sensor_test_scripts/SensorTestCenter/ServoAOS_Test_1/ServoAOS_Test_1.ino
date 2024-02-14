/* This script is an exmple of how we can interface between the absolute orientation sensor, servo, and teensy. It also highlights 
    an important bug where the servo jumps from the max to min mapped values when its sensor reading wrap around from 0 to 360.*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100) 

int angle;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo myServo; 
int pos = 0; //initial posistion for servo

void setup() {
  Serial.begin(115200);

  myServo.attach(20);

    /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);


  int x = event.orientation.x;

  /* If we were to map x from 0 to 360 regularly, then when the sensor goes from 0 to 360 it would cause the servo to jump
      In order to fix this, half of the range is mapped from min to max, while the other half is mapped from max to min. */
  if (x <= 180) {
    angle = map(x, 0, 180, 0, 80);
  }
  else {
    angle = map(x, 180, 360, 80, 0);
  }
  
  Serial.println(angle);

  Serial.print("X: ");
  Serial.println(event.orientation.x, 4);


  myServo.write(angle);

  delay(15);
                   
}

