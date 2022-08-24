#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"

// C: change motor to clockwise/anticlockwise
#define SERVO1 6
#define SERVO2 11
#define SERVO3 10
#define SERVO4 9

#define G 9.81

#define PRO 1.0 
#define INT 0.0
#define DER 0.0

#define SWEEP_SIZE 1

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
float finsAngles[] = {0,0,0,0};

// SUPERVISOR
float setApogee = 3000;
float predApogee = setApogee;
float lastPredApogee = setApogee;

float apogeeError = 0;
float cumaApogeeError = 0;
float changeApogeeError = 0;

/****************************************************/
// Sensor Declerations

// BNO055
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address. Not sure 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BMP390
Adafruit_BMP3XX bmp;
float groundLevelPressurehPa = 1013.25;  // (hPa) Gets changed on setup to current pressure val

/****************************************************/

void setup() {

  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);

  my_servo1.write(0);
  my_servo2.write(0);
  my_servo3.write(0);
  my_servo4.write(0);

  // BNO055
  bno.setExtCrystalUse(true);

  // BMP390
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  // Read current pressure and set as ground level
  groundLevelPressurehPa = bmp.readPressure()/100.0F;
  
}

void loop() {

  getAltitude();
  getIMU();
  updateApogee(1);
  updateApogeeErrors();
  updateFinAngles(1);
  writeFinAngles();
  
}

void getAltitude(){
  
  // C: get altitude from barometers
  rocketPos[2] = bmp.readAltitude(groundLevelPressurehPa);
  
}

void getIMU(){
  
  // C: get IMU data, get acceleration or whatever
  /* Get a new sensor event */
  sensors_event_t event;
  // Get Orientation
  bno.getEvent(&event);
  rocketAngPos[0] = event.orientation.x;
  rocketAngPos[1] = event.orientation.y;
  rocketAngPos[2] = event.orientation.z;
  
  // Get Linear Acceleration
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rocketAcc[0] = event.acceleration.x;
  rocketAcc[1] = event.acceleration.y;
  rocketAcc[2] = event.acceleration.z;

  // Get Angular Velocity
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rocketAngVel[0] = event.gyro.x;
  rocketAngVel[1] = event.gyro.y;
  rocketAngVel[2] = event.gyro.z;
}

void updateApogee(int pred){

    double p = rocketPos[2];
    double v = rocketVel[2];
    double a = rocketAcc[2];
    
    if(pred==1){
      predApogee = v*v*log(abs((a-G)/G))/(2*abs(a+G)) + p;
    }
    
    if(pred==2){
      // C: predictor 2 needs Cd values
    }
    
}

void updateApogeeErrors(){
  
  apogeeError = predApogee - setApogee;
  cumaApogeeError = cumaApogeeError + apogeeError;
  changeApogeeError = (predApogee-lastPredApogee+changeApogeeError)/2; //Filter Maybe?
  
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
