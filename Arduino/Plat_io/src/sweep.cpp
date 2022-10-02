// #include <Arduino.h>
// #include <Servo.h>
// #include <math.h>

// #define SERVO1 6
// #define SERVO2 8
// #define SERVO3 10
// #define SERVO4 12


// Servo my_servo1;
// Servo my_servo2;
// Servo my_servo3;
// Servo my_servo4;


// #define FIN_MAX 130
// #define FIN_MIN 15

// // 0 DEG
// #define OFFSET_1 -5
// #define OFFSET_2 8
// #define OFFSET_3 0
// #define OFFSET_4 -10


// // //90 DEG
// // #define OFFSET_1 5
// // #define OFFSET_2 0
// // #define OFFSET_3 -11
// // #define OFFSET_4 -10


// // ROCKET DYNAMICS
// double rocketPos[] = {0,0,0};
// double rocketVel[] = {0,0,0};
// double rocketAcc[] = {0,0,0};

// double rocketAngPos[] = {0,0,0};
// double rocketAngVel[] = {0,0,0};
// double rocketAngAcc[] = {0,0,0};

// int minn =0; 
// int maxx=0;
// int SWEEP_SIZE=3;

// // CONTROLLER DYNAMICS
// int finsAngles[] = {0,0,0,0};

// void setup() {

//   my_servo1.attach(SERVO1);
//   my_servo2.attach(SERVO2);
//   my_servo3.attach(SERVO3);
//   my_servo4.attach(SERVO4);
  
// //   my_servo1.write(10);
// //   my_servo2.write(10);
// //   my_servo3.write(10);
// //   my_servo4.write(10);


//   int AOA=0;
//   my_servo1.write(FIN_MIN+AOA+OFFSET_1);
//   my_servo2.write(FIN_MAX-AOA+OFFSET_2);
//   my_servo3.write(FIN_MIN+AOA+OFFSET_3);
//   my_servo4.write(FIN_MAX-AOA+OFFSET_4);


  
// }

// void loop() {
// // minn=0;
// // maxx=110;
// //   for(int AOA = minn; AOA <= maxx; AOA = AOA +SWEEP_SIZE){
// //       my_servo1.write(FIN_MIN+AOA+OFFSET_1);
// //   my_servo2.write(FIN_MAX-AOA+OFFSET_2);
// //   my_servo3.write(FIN_MIN+AOA+OFFSET_3);
// //   my_servo4.write(FIN_MAX-AOA+OFFSET_4);
// //     delay(10);    
// //   }
// // delay(200);
// //   for(int AOA = maxx; AOA >= minn; AOA = AOA -SWEEP_SIZE){
// //     my_servo1.write(FIN_MIN+AOA+OFFSET_1);
// //   my_servo2.write(FIN_MAX-AOA+OFFSET_2);
// //   my_servo3.write(FIN_MIN+AOA+OFFSET_3);
// //   my_servo4.write(FIN_MAX-AOA+OFFSET_4);
// //     delay(10); 
// //   }
 
// }

