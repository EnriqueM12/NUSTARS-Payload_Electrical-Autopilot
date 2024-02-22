#include <Servo.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
// missing libraries for altimeter and accel

/*
  - 1:Ailerons inverted - 2:Elevator inerted - 3:Throttle - 4:Rudder - 5: Ejection (Swtich C) - 6: Autopilot/Manual (Switch B) (Up - safe // Down - manual)
  - Servos will be powered by the R12DS and signal wire all connected to the teensy pins
  - They have to be PWM pins and non I2C pins
  - This should also log data
*/

//obj declration
Servo ailLeft; 
Servo ailRight; 
Servo elevatorLeftRight;
Servo rudder;
Servo ejectServo;

//channel resolution
int chAileronLeftRight; //ch1 to aileron R cntrl, 900left-1800right
int chElevator;
int chRudder; 
int chEject;
int chMode;


int posAilLeft;
int posAilRight;
int posRudder;
int posElevator;
int ejectServo;


void setup() {

  //missing sensor setup
  Serial.begin (115200);

  pinMode(20, INPUT);
  pinMode(19, INPUT);
  pinMode(18, INPUT);
}


/* 
  - Missing sensor declaration
  - Missing flight mode switch if cases
  - Missing data logging
  - Missing writing to servo
*/

void loop() {

  // this is corresponding reciever pin to teensy pin 
  chAileronLeftRight = pulseIn(4, HIGH);
  chElevator = pulseIn(5, HIGH);
  chRudder = pulseIn(6, HIGH);
  chEject = pulseIn(7, HIGH);
  chMode = pulseIn(8, HIGH);



}

void printChannel() { // If wired correctly should see prints for each channel
    Serial.print("Channel-AileronLeftRight: "); Serial.print(chAileronLeftRight); Serial.print("    ");
    Serial.print("||  Channel-Elevator: "); Serial.print(chElevator); Serial.print("    ");
    Serial.print("||  Channel-Rudder: "); Serial.print(chRudder); Serial.print("    ");    
    Serial.print("||  Channel-Eject: "); Serial.print(chEject); Serial.print("    ");
    Serial.print("||  Channel-Mode: "); Serial.print(chMode); Serial.println("    ");
}





