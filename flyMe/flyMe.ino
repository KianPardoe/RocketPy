#include <Servo.h>
#include <math.h>

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

double rocketPos[] = {0,0,0};
double rocketVel[] = {0,0,0};
double rocketAcc[] = {0,0,0};

double rocketAngPos[] = {0,0,0};
double rocketAngVel[] = {0,0,0};
double rocketAngAcc[] = {0,0,0};

double dragCo = 0.5;

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
  double alt = 20;
  
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
