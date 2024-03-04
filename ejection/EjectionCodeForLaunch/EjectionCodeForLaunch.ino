#include <Servo.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
#include <TimeLib.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX altimeter; 
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

  // altimeter setup
  altimeter.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  altimeter.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  altimeter.setOutputDataRate(BMP3_ODR_50_HZ);

  altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
  delay(100);
  

  ejectServo.attach(8);
  
  ejectMin = 0;
  ejectMax = 180;

  ejectServo.write(ejectMin);

  altInit = 0;
  for (int i = 0; i < 50; i++) {
    altInit += altimeter.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808399;
    //Serial.print(altInit); Serial.print(" ");
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

  Serial.print("Feet: "); Serial.print(feet);
  Serial.print("\t\tChannel: "); Serial.println(chEject);
  

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

void logData() { // Prints the altimeter and 
  // Time 
  File dataFile;
  currentMinute = minute();
  currentSecond = second();
  currentMillis = millis() % 1000;
  
  dataFile = SD.open("data.txt", FILE_WRITE);

  String dataString = "";


  // Logging
  dataString += "Altitude: " + String(feet) + "\t\t\t";
  if (feet <= 400) {
    dataString += "*********BELOW 400 BELOW 400 BELOW 400**********\t\t\t";
  }
  if (chEject >= 1870) {
    dataString += "*********ARMED ARMED ARMED**********\t\t\t";
  }
  dataString += String(currentMinute) + ":" + String(currentSecond) + ":" + String(currentMillis) + "\n";

  dataFile.println(dataString);
  dataFile.close();
}






