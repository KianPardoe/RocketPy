#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"
#include <ArduinoEigen.h>

// PIN CONSTANTS
#define SERVO1 6
#define SERVO2 8
#define SERVO3 10
#define SERVO4 12

#define BUZZ_PIN 32

// DYNAMIC CONSTANTS
#define G 9.81

// CONTROLLER CONSTANTS
#define PRO 5.3
#define INT -1.9
#define DER 0.0

#define VEL_PRO 2.2
#define VEL_INT 0.0/130.0
#define VEL_DER 0.0/130.0

#define SWEEP_SIZE 1
#define DEG2RAD PI/180.0

// FIN OFFSETS
#define FIN_MAX 130
#define FIN_MIN 15
//0 DEG
#define MIN_OFFSET_1 -5
#define MIN_OFFSET_2 8
#define MIN_OFFSET_3 0
#define MIN_OFFSET_4 -10
//90 DEG
#define MAX_OFFSET_1 5
#define MAX_OFFSET_2 0
#define MAX_OFFSET_3 -11
#define MAX_OFFSET_4 -10

// FIN CONTROL
#define HEIGHT_ACTIVE 2
#define FIXED_FIN_ANGLE 0

// Kalman Filter Variables
Eigen::Matrix<float, 3, 1> Xk;         // State Vector
Eigen::Matrix<float, 3, 1> Xkp_min;    // X_{k+1}^-
Eigen::Matrix<float, 2, 1> Y;          // Measurment Vector
Eigen::Matrix<float, 2, 1> Ykp_min;    // Y_{k+1}^-
Eigen::Matrix<float, 2, 3> C;          // Measurment Matrix y = Cx + Du
Eigen::Matrix<float, 3, 3> F;          // State Transition Matrix
Eigen::Matrix<float, 3, 3> Pk;         // State Estimate Covariance
Eigen::Matrix<float, 3, 3> Pkp_min;    // P_{k+1}^-
Eigen::Matrix<float, 2, 2> R;          // Sensor Covariance
Eigen::Matrix<float, 3, 3> Q;          // Model Covariance
Eigen::Matrix<float, 3, 2> K;          // Kalman Gain
Eigen::Matrix<float, 2, 2> S;          // Measurement prediction covariance
Eigen::Matrix<float, 3, 3> I;          // Identity Matrix


//Use SD card instead of flash cos we ballin, and by we I mean the arduino portenta and by ballin I mean died a horrible death
const int CS = BUILTIN_SDCARD;
File dataFile;

int i=0;

void setUpMemory();
void writeToMemory(String toWrite);
void writeTime();
void writeBaro();
void writeVel();
void writeIMU();
void writePredict();
void writeFinAngles();

void getTime();
void getAltitude();
void getIMU();
void getKalmanFilterPred();

void updateApogee(int pred);
void updateApogeeErrors();
void updateOffset(int AOA);
void updateFinAngles(int cont);
void sendFinAngles();



Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo my_servo4;

// ROCKET DYNAMICS
struct xyzType{
  float x;
  float y;
  float z;
};

xyzType rocketPos = {0,0,0};
xyzType rocketVel = {0,0,0};
xyzType rocketAcc = {0,0,0};

xyzType rocketAngPos = {0,0,0};
xyzType rocketAngVel = {0,0,0};


// CONTROLLER DYNAMICS
float finsAngles[] = {0,0,0,0};
float AOAHold = 0;
float der_enable = 0;

// KALMAN FILTERED STATE VECTOR (Height, Velocity, Acceleration)
float rocketKF[] = {0,0,0};

// SUPERVISOR
float setApogee = 70.0;
float predApogee = setApogee;
float lastPredApogee = setApogee;

float apogeeError = 0.0;
float cumaApogeeError = 0.0;
float changeApogeeError = 0.0;

// TIMING
float delta_T;
unsigned long last_millis = 0;

String Headers = "Time,Altitude,Velocity,AngleX,AngleY,AngleZ,AccX,AccY,AccZ,OmegaX,OmegaY,OmegaZ,Prediction,finAng1,finAng2,finAng3,finAng4,finAngMapped1,finAngMapped2,finAngMapped3,finAngMapped4,\n";

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
  //Serial.begin(115200);

// Initialise Kalman Filter Variables
Xk << 0, 0, 0;

C << 1, 0, 0,
      0, 0, 1;

Pk << 0.01, 0, 0,
      0, 0.01, 0,
      0, 0, 0.01;

R << 0.001, 0,
      0, 0.1;

Q << 0.000001, 0, 0,
      0, 0, 0,
      0, 0, 0.1;

I << 1, 0, 0,
      0, 1, 0,
      0, 0, 1;
  
  // Servos
  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);
  

  //Set fin angles
  int AOA=0;
  my_servo1.write(FIN_MIN+AOA+MIN_OFFSET_1);
  my_servo2.write(FIN_MAX-AOA+MIN_OFFSET_2);
  my_servo3.write(FIN_MIN+AOA+MIN_OFFSET_3);
  my_servo4.write(FIN_MAX-AOA+MIN_OFFSET_4);

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
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    }
  }

  setUpMemory();
  writeToMemory(Headers);

}

void loop() {
  getTime();
  getAltitude();
  getIMU();
  getKalmanFilterPred();
  updateApogee(1);
  updateApogeeErrors();
  updateFinAngles(2);
  sendFinAngles();

  writeTime();
  writeBaro();
  writeVel();
  writeIMU();
  writePredict();
  writeFinAngles();
  writeToMemory("\n");

}

void setUpMemory(){ 
//initialise the SD card, create a nice little file for our data to go in :)
  if(!SD.begin(CS)){
    digitalWrite(BUZZ_PIN, HIGH);
  }
  dataFile = SD.open("flightData.csv", FILE_WRITE);

}

void writeToMemory(String toWrite){
//we just write our string line by line, it's pretty much as easy as that. 
//the flush command is needed as we aren't closing the file, flush just forces is to make sure everything in the buffer gets written
  dataFile.print(toWrite);
  dataFile.flush();

}

void writeTime(){

  String toWrite = String(last_millis);
  toWrite = toWrite + ",";
  writeToMemory(toWrite);

}

void writeBaro(){

  String toWrite = String(rocketPos.z);
  toWrite = toWrite + ",";
  writeToMemory(toWrite);

}

void writeVel(){

  String toWrite = String(rocketVel.z);
  toWrite = toWrite + ",";
  writeToMemory(toWrite);

}

void writeIMU(){

  String toWrite = String(rocketAngPos.x);
  toWrite = toWrite + "," + String(rocketAngPos.y);
  toWrite = toWrite + "," + String(rocketAngPos.z);
  toWrite = toWrite + "," + String(rocketAcc.x);
  toWrite = toWrite + "," + String(rocketAcc.y);
  toWrite = toWrite + "," + String(rocketAcc.z);
  toWrite = toWrite + "," + String(rocketAngVel.x);
  toWrite = toWrite + "," + String(rocketAngVel.y);
  toWrite = toWrite + "," + String(rocketAngVel.z);
  
  writeToMemory(toWrite);

}

void writePredict(){

  String toWrite =  "," + String(predApogee);
  writeToMemory(toWrite);
  
}

void writeFinAngles(){

  String toWrite =  "," + String(finsAngles[0]);
  toWrite = toWrite + "," + String(finsAngles[1]);
  toWrite = toWrite + "," + String(finsAngles[2]);
  toWrite = toWrite + "," + String(finsAngles[3]);

  //Map motor offsets
  int offset1 = (MAX_OFFSET_1-MIN_OFFSET_1)*fabs(finsAngles[0])/90.0+MIN_OFFSET_1;
  int offset2 = (MAX_OFFSET_2-MIN_OFFSET_2)*fabs(finsAngles[1])/90.0+MIN_OFFSET_2;
  int offset3 = (MAX_OFFSET_3-MIN_OFFSET_3)*fabs(finsAngles[2])/90.0+MIN_OFFSET_3;
  int offset4 = (MAX_OFFSET_4-MIN_OFFSET_4)*fabs(finsAngles[3])/90.0+MIN_OFFSET_4;

  toWrite = toWrite + "," + String(FIN_MIN+finsAngles[0]+offset1);
  toWrite = toWrite + "," + String(FIN_MAX+finsAngles[1]+offset2);
  toWrite = toWrite + "," + String(FIN_MIN+finsAngles[2]+offset3);
  toWrite = toWrite + "," + String(FIN_MAX+finsAngles[3]+offset4);
  writeToMemory(toWrite);

}


void getTime(){

  unsigned long temp = millis();
  delta_T = ((float)(temp - last_millis))/1000.0f;
  last_millis = temp;

}

void getAltitude(){
  
  // C: get altitude from barometers
  rocketPos.z = bmp.readAltitude(groundLevelPressurehPa);

}

void getIMU(){

  // C: get IMU data, get acceleration or whatever
  /* Get a new sensor event */
  sensors_event_t event;
  // Get Orientation
  bno.getEvent(&event);
  rocketAngPos.x = event.orientation.y * DEG2RAD;
  rocketAngPos.y = event.orientation.z * DEG2RAD;
  rocketAngPos.z = event.orientation.x * DEG2RAD;
  
  // Get Linear Acceleration (m/s^2)
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rocketAcc.x = event.acceleration.y;
  rocketAcc.y = event.acceleration.z;
  rocketAcc.z = event.acceleration.x;

  // Get Angular Velocity (deg/s)
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rocketAngVel.x = event.gyro.y * DEG2RAD;
  rocketAngVel.y = event.gyro.z * DEG2RAD;
  rocketAngVel.z = event.gyro.x * DEG2RAD;

}

void getKalmanFilterPred(){
  
  Y << rocketPos.z, rocketAcc.z;

  F << 1, delta_T, 0.5*delta_T*delta_T,
  0, 1, delta_T,
  0, 0, 1;

  Xkp_min << F * Xk;
  Ykp_min << C * Xkp_min;
  Pkp_min << F * Pk * F.transpose() + Q;
  S << C * Pkp_min * C.transpose() + R;
  K << Pkp_min * C.transpose() * S.inverse();
  Xk << Xkp_min + K * (Y - Ykp_min);
  Pk << (I - K * C) * Pkp_min;

  // Assign velocity to rocketVel and rocketKF variables
  rocketVel.z = Xk(1);
  rocketKF[0] = Xk(0);
  rocketKF[1] = Xk(1);
  rocketKF[2] = Xk(2);
}

void updateApogee(int pred){

  double p = rocketPos.z;
  double v = rocketVel.z;
  double a = rocketAcc.z-G;
  

  if(pred==1){
    if(rocketAcc.z!=0){  
      predApogee = v*v*log(fabs(a/G))/(2*fabs(a+G)) + p;
    } 
  }
  
  if(pred==2){
    // C: predictor 2 needs Cd values
  }

}

void updateApogeeErrors(){
  
  apogeeError =  predApogee - setApogee;
  cumaApogeeError = cumaApogeeError + apogeeError;
  changeApogeeError = ((predApogee-lastPredApogee)/2+changeApogeeError)/2; //Filter Maybe?
  
}

void updateFinAngles(int cont){

  double AOA;

  if(der_enable == 0){
    if(finsAngles[0]!=0){
      der_enable = 1.0;
    }
  }


  if(cont==0){
    AOA = FIXED_FIN_ANGLE;
  }
  
  if(cont==1){
    AOA = PRO*apogeeError + INT/DEG2RAD*cumaApogeeError + DER*der_enable/DEG2RAD*changeApogeeError;
  }
  
  if(cont==2){
    float deltaAngle = VEL_PRO*apogeeError + VEL_INT/DEG2RAD*cumaApogeeError + VEL_DER/DEG2RAD*changeApogeeError;
    AOA = AOAHold+ deltaAngle;
  }

  // Limits
  if(AOA>90){
    AOA = 90;
  }
  else if(AOA<0){
    AOA = 0;
  }

  AOAHold = AOA;

  // Do not acutate during ground phase
  if(rocketPos.z<HEIGHT_ACTIVE){
    AOA = 0;
  }

  finsAngles[0] = AOA;
  finsAngles[1] = -AOA;
  finsAngles[2] = AOA;
  finsAngles[3] = -AOA;
 
}

void sendFinAngles(){

  // Map motor offsets
  int offset1 = (MAX_OFFSET_1-MIN_OFFSET_1)*fabs(finsAngles[0])/90.0+MIN_OFFSET_1;
  int offset2 = (MAX_OFFSET_2-MIN_OFFSET_2)*fabs(finsAngles[1])/90.0+MIN_OFFSET_2;
  int offset3 = (MAX_OFFSET_3-MIN_OFFSET_3)*fabs(finsAngles[2])/90.0+MIN_OFFSET_3;
  int offset4 = (MAX_OFFSET_4-MIN_OFFSET_4)*fabs(finsAngles[3])/90.0+MIN_OFFSET_4;

  // Actuate motors
  my_servo1.write(FIN_MIN+finsAngles[0]+offset1);
  my_servo2.write(FIN_MAX+finsAngles[1]+offset2);
  my_servo3.write(FIN_MIN+finsAngles[2]+offset3);
  my_servo4.write(FIN_MAX+finsAngles[3]+offset4);

}