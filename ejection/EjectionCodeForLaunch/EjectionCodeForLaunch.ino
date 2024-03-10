#include <Servo.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <TimeLib.h> 

#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO055_SAMPLERATE_DELAY_MS (100) 

Adafruit_BMP3XX altimeter; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // WIRE <--- VERY IMPORTANT!!!!!! 


float feet;
float altInit;

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
  bno.begin();

  // altimeter setup
  altimeter.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  altimeter.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  altimeter.setOutputDataRate(BMP3_ODR_50_HZ);

  altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  delay(100);
  

  ejectServo.attach(8);
  
  ejectMin = 0;
  ejectMax = 90;

  ejectServo.write(ejectMin);

  altInit = 0;
  for (int i = 0; i < 50; i++) {
    altInit += altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
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

  logData();
  servoMove();
}

void servoMove() {
  if (feet <= 400) {
    if (chEject >= 1870) {
      ejectServo.write(ejectMin);
    }
    else if (chEject <= 1870) {
      ejectServo.write(ejectMax);
    }
      
  }
}

void logData() { // Prints the altimeter and 
  // Time 
  File dataFile;  
  dataFile = SD.open("data.txt", FILE_WRITE);

  sensors_event_t eventAOS;
  bno.getEvent(&eventAOS);

  float xOrient = eventAOS.orientation.x;
  float yOrient = eventAOS.orientation.y;
  float zOrient = eventAOS.orientation.z;

  float xAccel = eventAOS.acceleration.x;
  float yAccel = eventAOS.acceleration.y;
  float zAccel = eventAOS.acceleration.z;

  String dataString = "";


  // Logging

  //if (millis() % 100 == 0) {
    dataString += String(feet) + ", " 
                + String(xAccel) + ", " + String(yAccel) + ", " + String(zAccel) + ", " 
                + String(xOrient) + ", " + String(yOrient) + ", " + String(zOrient);

    if (feet <= 400) {
      dataString += ", below, ";
    }
    else {
      dataString += ", above, ";
    }
    if (chEject >= 1870) {
      dataString += ", armed, \n";
    }
    else {
      dataString += ", unarmed, \n";
    }
  //}
  dataFile.println(dataString);
  dataFile.close();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}






