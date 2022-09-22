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
// #define FIN_MIN 10

// // 0 DEG
// #define OFFSET_1 -8
// #define OFFSET_2 -3
// #define OFFSET_3 -7
// #define OFFSET_4 0


// // //90 DEG
// // #define OFFSET_1 0
// // #define OFFSET_2 -18
// // #define OFFSET_3 -2
// // #define OFFSET_4 -5


// // ROCKET DYNAMICS
// double rocketPos[] = {0,0,0};
// double rocketVel[] = {0,0,0};
// double rocketAcc[] = {0,0,0};

// double rocketAngPos[] = {0,0,0};
// double rocketAngVel[] = {0,0,0};
// double rocketAngAcc[] = {0,0,0};

// int maxx=130;
// int minn=10;
// int SWEEP_SIZE=1;

// // CONTROLLER DYNAMICS
// int finsAngles[] = {minn,minn,minn,minn};

// void setup() {

//   my_servo1.attach(SERVO1);
//   my_servo2.attach(SERVO2);
//   my_servo3.attach(SERVO3);
//   my_servo4.attach(SERVO4);
  
// //   my_servo1.write(10);
// //   my_servo2.write(10);
// //   my_servo3.write(10);
// //   my_servo4.write(10);


//   int AOA=45;
//   my_servo1.write(FIN_MIN+AOA+OFFSET_1);
//   my_servo2.write(FIN_MAX-AOA+OFFSET_2);
//   my_servo3.write(FIN_MIN+AOA+OFFSET_3);
//   my_servo4.write(FIN_MAX-AOA+OFFSET_4);


  
// }

// void loop() {

//   for(int pos = minn; pos <= maxx; pos = pos +SWEEP_SIZE){
//      my_servo1.write(pos);
//      my_servo2.write(maxx+minn-pos);
//      my_servo3.write(pos);
//      my_servo4.write(maxx+minn-pos);
//     delay(10);    
//   }
// delay(200);
//   for(int pos = maxx; pos >= minn; pos = pos -SWEEP_SIZE){
//      my_servo1.write(pos);
//      my_servo2.write(maxx+minn-pos);
//      my_servo3.write(pos);
//      my_servo4.write(maxx+minn-pos);
//     delay(10); 
//   }
 
// }

