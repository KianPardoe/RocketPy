#include <Servo.h>
#include <math.h>
#include "Wire."
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"

// C: change motor to clockwise/anticlockwise
#define SERVO1 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5

#define BUZZ_PIN 6

#define G 9.81

#define PRO 0.3 
#define INT 0.0
#define DER 0.0

#define SWEEP_SIZE 1
#define DEG2RAD PI/180.0

//Use SD card instead of flash cos we ballin, and by we I mean the arduino portenta and by ballin I mean died a horrible death
const int CS = BUILTIN_SDCARD;
File dataFile;

void setUpMemory();
void writeToMemory(String toWrite);
void writeBaro();
void writeIMU();
void writePredict();

void getAltitude();
void getIMU();
void updateApogee(int pred);
void updateApogeeErrors();
void updateFinAngles(int cont);
void writeFinAngles();

Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo my_servo4;

// ROCKET DYNAMICS
float rocketPos[] = {0,0,0};
float rocketVel[] = {0,0,0};
float rocketAcc[] = {0,0,0};

float rocketAngPos[] = {0,0,0};
float rocketAngVel[] = {0,0,0};
float rocketAngAcc[] = {0,0,0};

// CONTROLLER DYNAMICS
float finsAngles[] = {10,10,10,10};

// SUPERVISOR
float setApogee = 3000;
float predApogee = setApogee;
float lastPredApogee = setApogee;

float apogeeError = 0;
float cumaApogeeError = 0;
float changeApogeeError = 0;

/****************************************************/
// Sensor Declerations
/****************************************************/
// BNO055
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BMP390
Adafruit_BMP3XX bmp;
float groundLevelPressurehPa = 1013.25;  // (hPa) Gets changed on setup to current pressure val

/****************************************************/

void setup() {
  /****************************************************/
  // Debug Code
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /****************************************************/
  // Servos
  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);
  
  my_servo1.write(10);
  my_servo2.write(10);
  my_servo3.write(10);
  my_servo4.write(10);

  // Use built in LED for indicating error
  pinMode(LED_BUILTIN, OUTPUT);
  bool Error_LED = false;

  // Use Buzzer to indicate calibration complete
  pinMode(BUZZ_PIN, OUTPUT);

  /****************************************************/
  // Sensor Setup
  /****************************************************/
  // BNO055
  /* Initialise the sensor */
  
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    while(1){
      digitalWrite(LED_BUILTIN, Error_LED);
      Error_LED = !Error_LED;
      delay(100);
    }
  } 

  bno.setExtCrystalUse(true);

  // BMP390
  /* Initialise the sensor */
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    while(1){
      digitalWrite(LED_BUILTIN, Error_LED);
      Error_LED = !Error_LED;
      delay(500);
    }
  }
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Sensor Calibration BMP390
  // Average 10 readings over 5 seconds to set as ground level
  /* Take 100 readings to get through initial eronious readings*/
  for(int i=0; i<100; i++){
  bmp.readPressure();
  }
  float pressureSum = 0;
  for(int i=0; i<10; i++){
    pressureSum += bmp.readPressure()/100.0F;
    delay(100);
  }
  groundLevelPressurehPa = pressureSum/10.0;

  // Sensor Calibration BNO055
  // Wait for Calibration and Confirm with 1 second Buzz
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  while(1){
    if(system < 1 || gyro < 1 || accel < 1 || mag < 1){
      delay(500);
      bno.getCalibration(&system, &gyro, &accel, &mag);
    }else{
      digitalWrite(BUZZ_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZ_PIN, LOW);
      break;
    }
  }

  setUpMemory();

}

void loop() {
  getAltitude();
  getIMU();
  updateApogee(1);
  updateApogeeErrors();
  updateFinAngles(1);
  writeFinAngles();

  /****************************************************/
  // Debug Code
  Serial.print("Altitude: ");
  Serial.print(rocketPos[2], 4); Serial.println(" (m)");
  
  Serial.print("Orientation: ");
  Serial.print("X: ");
  Serial.print(rocketAngPos[0], 4);
  Serial.print("\tY: ");
  Serial.print(rocketAngPos[1], 4);
  Serial.print("\tZ: ");
  Serial.print(rocketAngPos[2], 4);
  Serial.println("");

  delay(33);
  /****************************************************/
}

void setUpMemory(){ 
//initialise the SD card, create a nice little file for our data to go in :)
  if(!SD.begin(CS)){
    Serial.println('uh oh...that is genuinely not good');
  }
  dataFile = SD.open("flightData.csv", FILE_WRITE);

}

void writeToMemory(String toWrite){
//we just write our string line by line, it's pretty much as easy as that. 
//the flush command is needed as we aren't closing the file, flush just forces is to make sure everything in the buffer gets written
  dataFile.println(toWrite);
  dataFile.flush();

}

void writeBaro(){

  String toWrite = String(rocketPos[2]);
  toWrite = toWrite + "," + String(rocketVel[2]);
  writeToMemory(toWrite);

}
void writeIMU(){

  String toWrite =  "," + String(rocketAngPos[0]);
  toWrite = toWrite + "," + String(rocketAngPos[1]);
  toWrite = toWrite + "," + String(rocketAngPos[2]);
  toWrite = toWrite + "," + String(rocketAcc[0]);
  toWrite = toWrite + "," + String(rocketAcc[1]);
  toWrite = toWrite + "," + String(rocketAcc[2]);
  toWrite = toWrite + "," + String(rocketAngVel[0]);
  toWrite = toWrite + "," + String(rocketAngVel[1]);
  toWrite = toWrite + "," + String(rocketAngVel[2]);
  
  writeToMemory(toWrite);

}

void writePredict(){

  String toWrite =  "," + String(predApogee) + "\n";
  writeToMemory(toWrite);
  
}

void getAltitude(){
  
  // C: get altitude from barometers
  float hold = rocketPos[2];
  rocketPos[2] = bmp.readAltitude(groundLevelPressurehPa);
  rocketVel[2] = rocketPos[2] - hold;
  // rocketVel[2] = ((rocketPos[2] - rocketPos[2])/2+rocketVel[2])/2; % filter version
  writeBaro();

}

void getIMU(){
  
  // C: get IMU data, get acceleration or whatever
  /* Get a new sensor event */
  sensors_event_t event;
  // Get Orientation
  bno.getEvent(&event);
  rocketAngPos[0] = event.orientation.x * DEG2RAD;
  rocketAngPos[1] = event.orientation.y * DEG2RAD;
  rocketAngPos[2] = event.orientation.z * DEG2RAD;
  
  // Get Linear Acceleration (m/s^2)
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rocketAcc[0] = event.acceleration.x;
  rocketAcc[1] = event.acceleration.y;
  rocketAcc[2] = event.acceleration.z;

  // Get Angular Velocity (deg/s)
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rocketAngVel[0] = event.gyro.x * DEG2RAD;
  rocketAngVel[1] = event.gyro.y * DEG2RAD;
  rocketAngVel[2] = event.gyro.z * DEG2RAD;

  //writeIMU();

}

void updateApogee(int pred){

    double p = rocketPos[2];
    double v = rocketVel[2];
    double a = rocketAcc[2]-G;
    
    if(pred==1){
      predApogee = v*v*log(abs((a)/G))/(2*abs(a+G)) + p;
    }
    
    if(pred==2){
      // C: predictor 2 needs Cd values
    }

    writePredict();
    
}

void updateApogeeErrors(){
  
  apogeeError = predApogee - setApogee;
  cumaApogeeError = cumaApogeeError + apogeeError;
  changeApogeeError = ((predApogee-lastPredApogee)/2+changeApogeeError)/2; //Filter Maybe?
  
}

void updateFinAngles(int cont){

  if(cont==1){
    double out = PRO*apogeeError + INT*cumaApogeeError + DER*changeApogeeError;
    finsAngles[0] = -out;
    finsAngles[1] = out;
    finsAngles[2] = out;
    finsAngles[3] = -out;
  }
  
  //if(cont==2){
    // C: lqr 2
  //}
 
}

void writeFinAngles(){

  my_servo1.write(finsAngles[0]);
  my_servo2.write(finsAngles[1]);
  my_servo3.write(finsAngles[2]);
  my_servo4.write(finsAngles[3]);

}
