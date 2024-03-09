#include <Servo.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <TimeLib.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX altimeter; 
Adafruit_BNO055 accelerometer = Adafruit_BNO055(55);
float feet;
float altInit;
float acc_x;
float acc_y;
float acc_z;
float acc_mag;

/*  
Wiring (excluding 5V and GND):
  - BMP390: 
    - SCL to 19 (Yellow)
    - SDA to 18 (Blue)
*/

Servo ejectServo;
int ejectMin;
int ejectMax;

int posEject;

int chEject;


// time
const int chipSelect = BUILTIN_SDCARD; // The teensy has a pre-named chip number that is classified under "BUILTIN_SDCARD"
int currentMinute;
int currentSecond;
long currentMillis;


void setup() {
  
  Serial.begin (115200);

  altimeter.begin_I2C();

  // altimeter setup
  altimeter.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  altimeter.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  altimeter.setOutputDataRate(BMP3_ODR_50_HZ);

  altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  delay(100);

  // accelerometer set up
  //if (!accelerometer.begin()){
    //Serial.println("Could not find a valid BNO055 sensor");
    //while (1);
  //}
  accelerometer.setExtCrystalUse(true);
  

  ejectServo.attach(8);
  
  ejectMin = 0;
  ejectMax = 180;

  ejectServo.write(ejectMin);

  altInit = 0;
  for (int i = 0; i < 50; i++) {
    altInit += altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
    Serial.print(altInit); Serial.print(" ");
    delay(100);
  }
  altInit = altInit / 50;

  Serial.print("INITAIAL VALUE: "); Serial.println(altInit);

  SD.begin(chipSelect);
}
 
void loop() {

  if (!altimeter.performReading()) {
    Serial.println("Failed to get reading from altimeter!");
    return;
  }

  
  feet = altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  feet -= altInit;

  chEject = pulseIn(4, HIGH);
  posEject = map(chEject, 1070, 1931, ejectMin, ejectMax);



  Serial.print("Feet: "); Serial.print(feet);
  Serial.print("\t\tChannel: "); Serial.println(chEject);
  
  sensors_event_t event;
  accelerometer.getEvent(&event);
  Serial.print("Acceleration X: ");
  acc_x = event.acceleration.x;
  Serial.print(acc_x);
  Serial.print(" m/s^2");
  Serial.print("Acceleration Y: ");
  acc_y = event.acceleration.y;
  Serial.print(acc_y);
  Serial.print(" m/s^2");
  Serial.print("Acceleration Z: ");
  acc_z = event.acceleration.z;
  Serial.print(acc_z);
  Serial.println(" m/s^2");

  acc_mag = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  logData();
  servoMove();

  delay(50);
}

void servoMove() {
  if (feet <= 400) {
    if (chEject >= 1870) {
      ejectServo.write(ejectMax);
    }
    else if (chEject <= 1870) {
      ejectServo.write(ejectMin);
    }
      
  }
}

void logData() { // Prints the altimeter and accelerometer
  // Time 
  File dataFile;
  currentMinute = minute();
  currentSecond = second();
  currentMillis = millis() % 1000;
  
  dataFile = SD.open("data.txt", FILE_WRITE);

  String dataString = "";


  // Logging
  dataString += "Altitude: " + String(feet) + "\t\t\t";
  dataString += "Acceleration: " + String(acc_mag) + "\t\t\t";
  if (feet <= 400 && feet >=50) {
    dataString += "*********BELOW 400 BELOW 400 BELOW 400**********\t\t\t";
  }
  if (chEject >= 1870) {
    dataString += "*********ARMED ARMED ARMED**********\t\t\t";
  }
  if (feet <= 50) {
    dataString += "*********ABOUT TO LAND*************\t\t\t";
  }
  dataString += String(currentMinute) + ":" + String(currentSecond) + ":" + String(currentMillis) + "\n";

  dataFile.println(dataString);
  dataFile.close();
}
