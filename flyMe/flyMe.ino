#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo my_servo4;

float rocketPos[] = {0,0,0};
float rocketVel[] = {0,0,0};
float rocketAcc[] = {0,0,0};

float rocketAngPos[] = {0,0,0};
float rocketAngVel[] = {0,0,0};
float rocketAngAcc[] = {0,0,0};

float dragCo = 0.5;

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
float groundLevelPressure = 1013.25  // (hPa) Gets changed on setup to current pressure val

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
  groundLevelPressureHPA = bmp.readPressure()/100.0F;
  
  
}

void loop() {

  //getAltitude();
  //getIMU();
  //updateApogee(1);
  //updateApogeeErrors();
  //updateFinAngles(1)
  
  for(int pos = 0; pos <= 45; pos = pos +SWEEP_SIZE){
    my_servo1.write(pos);
    delay(100);    
  }

  for(int pos = 45; pos <= 0; pos = pos -SWEEP_SIZE){
    my_servo1.write(pos);
    delay(100); 
  }
  
  for(int pos = 0; pos <= 45; pos = pos +SWEEP_SIZE){
    my_servo2.write(pos);
    delay(100);    
  }

  for(int pos = 45; pos <= 0; pos = pos -SWEEP_SIZE){
    my_servo2.write(pos);
    delay(100);
  }
  
  for(int pos = 0; pos <= 45; pos = pos +SWEEP_SIZE){
    my_servo3.write(pos);
    delay(100);    
  }

  for(int pos = 45; pos <= 0; pos = pos -SWEEP_SIZE){
    my_servo3.write(pos);
    delay(100);
  }

    for(int pos = 0; pos <= 45; pos = pos +SWEEP_SIZE){
    my_servo4.write(pos);
    delay(100);    
  }

  for(int pos = 45; pos <= 0; pos = pos -SWEEP_SIZE){
    my_servo4.write(pos);
    delay(100);
  }
  
}

void getAltitude(){
  
  // C: get altitude from barometers
  rocketPos[2] = bmp.readAltitude(groundLevelPressureHPA)
  
}

void getIMU(){

  // C: barometer ata
  
}

void updateApogee(int pred){

    double p = rocketPos[2];
    double v = rocketVel[2];
    double a = rocketAcc[2];
    
    if(pred==1){
      predApogee = v*v*log(abs((a-G)/G))/(2*abs(a+G)) + p;
    }
    
    if(pred==2){
      // C: pre 2
    }
    
}

void updateApogeeErrors(){
  
  apogeeError = predApogee - setApogee;
  cumaApogeeError = cumaApogeeError + apogeeError;
  changeApogeeError = (predApogee-lastPredApogee+changeApogeeError)/2; //Filter Maybe?s  
  
}

void updateFinAngles(int cont){

    if(cont==1){
      
    }
    
    //if(cont==2){
      // C: lqr 2
    //}
 
}
