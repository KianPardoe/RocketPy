#include <Servo.h>

#define SERVO1 14
#define SERVO2 13
#define SERVO3 12
#define SERVO4 9

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);

  servo1.write(0)
  servo2.write(0)
  servo3.write(0)
  servo4.write(0)

}

void loop() {

}
