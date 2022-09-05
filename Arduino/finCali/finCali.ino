#include <Servo.h>

#define SERVO1 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5

Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo my_servo4;

void changeAngle();
void writeFinAngles();

// CONTROLLER DYNAMICS
int finsAngles[] = {10,10,10,10};
int currMotor = 0;
char read2;
int inc;
void setup() {

  Serial.begin(115200);

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

  if (Serial.available() > 0) {

    read2 = Serial.read();
    switch(read2){
      case 'd':
          currMotor = (currMotor + 1)%4;
          Serial.print("Selected Motor ");
          Serial.println(currMotor);
          break;

      case 's':
          inc = -1;
          changeAngle();
          break;

      case 'w':
          inc = 1;
          changeAngle();
          break;

      case 'a':
        currMotor = (currMotor - 1);
        if(currMotor == -1){
           currMotor = 3;
        }
          Serial.print("Selected Motor ");
          Serial.println(currMotor);
          break;

      case 10:
          break;

      default:
          Serial.println("Dickhead, Wrong Input");
    }

  }
  writeFinAngles();
  delay(10);
  
}

void changeAngle(){

  finsAngles[currMotor] = finsAngles[currMotor] + inc;
  Serial.print(String(finsAngles[0]) + ",");
  Serial.print(String(finsAngles[1]) + ",");
  Serial.print(String(finsAngles[2]) + ",");
  Serial.println(finsAngles[3]);
  
}

void writeFinAngles(){

  my_servo1.write(finsAngles[0]);
  my_servo2.write(finsAngles[1]);
  my_servo3.write(finsAngles[2]);
  my_servo4.write(finsAngles[3]);

}
