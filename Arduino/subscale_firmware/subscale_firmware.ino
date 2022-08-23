#include <Servo.h>
#include <math.h>

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
double rocketPos[] = {0,0,0};
double rocketVel[] = {0,0,0};
double rocketAcc[] = {0,0,0};

double rocketAngPos[] = {0,0,0};
double rocketAngVel[] = {0,0,0};
double rocketAngAcc[] = {0,0,0};

// CONTROLLER DYNAMICS
double finsAngles[] = {0,0,0,0};

// SUPERVISOR
double setApogee = 3000;
double predApogee = setApogee;
double lastPredApogee = setApogee;

double apogeeError = 0;
double cumaApogeeError = 0;
double changeApogeeError = 0;

void setup() {

  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);

  my_servo1.write(0);
  my_servo2.write(0);
  my_servo3.write(0);
  my_servo4.write(0);
  
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
  
  // C: get altitude from barometer or whatever, Kalman filter maybe
  double alt = 20;
  
}

void getIMU(){

  // C: get IMU data, get acceleration or whatever
  
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
