/*
  This code is slighly incomplete as it is missing the control for the ejection pin, but it can still be used and is useful
*/

#include <Servo.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <SD.h>
#include <TimeLib.h>

#define SEALEVELPRESSURE_HPA (1013.25)

/*
  - 1:Ailerons inverted (1071-1937) - 2:Elevator inverted (1061-1928) - 3:Throttle - 4:Rudder - 6: Autopilot/Manual & Ejection (Switch B) (Up - safe // Down - manual) (1066-1933) - Swtich C (1066-1500-1933)
  - Servos will be powered by the R12DS and signal wire all connected to the teensy pins
  - They have to be PWM pins and non I2C pins
  - This should also log data

  Wiring (excluding 5V and GND):
  - BMP390: 
    - SCL to 19 (Yellow)
    - SDA to 18 (Blue)
  - ADXL375: 
    - SCL to 19 (Yellow)
    - SDA to 18 (Blue)
*/


Adafruit_ADXL375 accel = Adafruit_ADXL375(0x53);
Adafruit_BMP3XX altimeter; 

//obj declration
Servo ailLeft; 
Servo ailRight; 
Servo elevatorLeftRight;
Servo ejectServo;

// servo positions
int posAileronLeft;
int posAileronRight;
int posElevators;
int posEject;

//min and max values for servo angles
int servoMin;
int servoMax;
int ejectMin;
int ejectMid;
int ejectMax;

// set up alitude
float feet;
float refAlt;
float junkAlt;
int iteration = 1;

//channel resolution
int chAileronLeftRight; //ch1 to aileron R cntrl, 900left-1800right
int chElevator;
int chEject;
int chMode;

// set up ejection boolean
bool ejected = false;

// time
const int chipSelect = BUILTIN_SDCARD; // The teensy has a pre-named chip number that is classified under "BUILTIN_SDCARD"
int currentMinute;
int currentSecond;
long currentMillis;

void setup() {
  Serial.begin (115200);

  // altimeter setup
  altimeter.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  altimeter.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  altimeter.setOutputDataRate(BMP3_ODR_50_HZ);

  // servo min and max
  servoMin = 40;
  servoMax = 120;
  ejectMin = 0;
  ejectMid = 90;
  ejectMax = 180;


  // attach servos to pins (check if these are good)
  ailLeft.attach(9);
  ailRight.attach(10);
  elevatorLeft.attach(11);
  elevatorRight.attach(12);
  ejectServo.attach(25);
}
/*


*/
void loop() {
  // calculate all values needed 
  feet = altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  chAileronLeftRight = pulseIn(4, HIGH);
  chElevator = pulseIn(5, HIGH);
  chEject = pulseIn(6, HIGH);
  chMode = pulseIn(7, HIGH);


  manualControl();
  logData();
}

void manualControl() {
  //set the servo values
  posAileronLeft = map(chAileronLeftRight, 1075, 1945, servoMin, servoMax);
  posAileronRight = map(chAileronLeftRight, 1075, 1945, servoMax, servoMin);
  posElevator = map(chElevator, 1070, 1091, servoMin, servoMax);
  
  if (feet <= 400 && chEject >= 1870) {
    ailLeft.write(posAileronLeft);
    ailRight.wrire(posAileronRight);
    elevatorLeftRight.write(posElevator);

  }
}


/*
void loop() {

  // set alt to zero
  if (iteration == 1){
    iteration++;
    junkAlt = altimeter.readAltitude(1013.25);
  }
  else if (iteration == 2){
    refAlt = altimeter.readAltitude(1013.25);
    iteration++;
  }

  else if (iteration>2){
    float alt = altimeter.readAltitude(1013.25)-refAlt;
    iteration++;
    // this is corresponding reciever pin to teensy pin 

    //check if these pins are good
    chAileronLeftRight = pulseIn(4, HIGH);
    chElevator = pulseIn(5, HIGH);
    chRudder = pulseIn(6, HIGH);
    chEject = pulseIn(7, HIGH);
    chMode = pulseIn(8, HIGH);
  
    // data logging (if wired correctly)

    Serial.print("Altitude: "); Serial.print(altimeter.readAltitude(1017.61)); Serial.print("    ");

    // ejection code
    if (!ejected) {
      //alt = readAlt();
      if (alt <= 400 && chMode <1071) {
        ejectServo.write(180); // likely need to edit how servo moves placeholder for now
        ejected = true;
      }
    }
  
    // once ejected, allow for manual control
    if (ejected) {
      // map receivers to servo positions (not real values)
      int ailAngle = map(chAileronLeftRight, 1075, 1945, 0, 180); // adjust mapping values so it flies how we want
      int eleAngle = map(chElevator, 1070, 1941, 0, 180); // adjust mapping values so it flies how we want
      int rudderAngle = map(chRudder, 1066, 1936, 0, 180); // adjust mapping values so it flies how we want

      // write servo positions
      ailLeft.write(ailAngle);
      ailRight.write(ailAngle);
      elevatorLeft.write(eleAngle);
      elevatorRight.write(eleAngle);
      rudder.write(rudderAngle);
    }

    //delay(50)
  }
}
*/


void logData() { // Prints the altimeter and 
  // Time 
  File dataFile;
  currentMinute = minute();
  currentSecond = second();
  currentMillis = millis() % 1000;
  dataFile = SD.open("datalogSDTest.txt", FILE_WRITE | O_TRUNC);

  String dataString = "";

  // Accel
  sensors_event_t eventACCEL;
  accel.getEvent(&eventACCEL);
  long accelX = eventACCEL.acceleration.x;
  long accelY = eventACCEL.acceleration.y;
  long accelZ = eventACCEL.acceleration.z;

  // Altimeter



  // Logging
  dataString += "X: " + String(accelX) + " | " 
              + "Y: " + String(accelY) + " | " 
              + "Z: " + String(accelZ) + "\t\t\t | " 
              + "Altitude: " + String(feet) + "\t\t\t" 
              + currentMinute + ":" + currentSecond + ":" + currentMillis + "\n";
  

  dataFile.println(dataString);
  dataFile.close();
}

void printData() {
  Serial.print("Channel-AileronLeftRight: "); Serial.print(chAileronLeftRight); Serial.print("    ");
  Serial.print("||  Channel-Elevator: "); Serial.print(chElevator); Serial.print("    ");
  Serial.print("||  Channel-Rudder: "); Serial.print(chRudder); Serial.print("    ");    
  Serial.print("||  Channel-Eject: "); Serial.print(chEject); Serial.print("    ");
  Serial.print("||  Channel-Mode: "); Serial.print(chMode); Serial.println("    ");
}











