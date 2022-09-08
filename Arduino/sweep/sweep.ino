#include <Servo.h>
#include <math.h>

#define SERVO1 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5


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

int maxx=130;
int minn=10;
int SWEEP_SIZE=1;

// CONTROLLER DYNAMICS
int finsAngles[] = {minn,minn,minn,minn};

void setup() {

  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);
  
  my_servo1.write(10);
  my_servo2.write(10);
  my_servo3.write(10);
  my_servo4.write(10);
  
}

void loop() {

  for(int pos = minn; pos <= maxx; pos = pos +SWEEP_SIZE){
     my_servo1.write(pos);
     my_servo2.write(maxx+minn-pos);
     my_servo3.write(pos);
     my_servo4.write(maxx+minn-pos);
    delay(10);    
  }
delay(200);
  for(int pos = maxx; pos >= minn; pos = pos -SWEEP_SIZE){
     my_servo1.write(pos);
     my_servo2.write(maxx+minn-pos);
     my_servo3.write(pos);
     my_servo4.write(maxx+minn-pos);
    delay(10); 
  }
 
}
