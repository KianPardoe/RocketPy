#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include "QSPIFBlockDevice.h"
#include "MBRBlockDevice.h"
#include <stdio.h>
#include <stdlib.h>

// C: change motor to clockwise/anticlockwise
#define SERVO1 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5

#define BUZZ_PIN 6

#define BLOCK_DEVICE_SIZE 1024 * 8 // 8 KB
#define PARTITION_TYPE 0x0B // FAT 32

#define G 9.81

#define PRO 0.3 
#define INT 0.0
#define DER 0.0

#define SWEEP_SIZE 1
#define DEG2RAD PI/180.0

void setUpMemory();
void clearMemory();
void writeToMemory(String toWrite);
void readMemory();
void writeBaro();
void writeIMU();

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

// Memory
using namespace mbed;
QSPIFBlockDevice root(PD_11, PD_12, PF_7, PD_13,  PF_10, PG_6, QSPIF_POLARITY_MODE_1, 40000000);
MBRBlockDevice blockDevice(&root, 1); 
int memoryCursor = 0;

void setup() {
  /****************************************************/
  // Debug Code
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /****************************************************/
  // Servos
  my_servo1.attach(SERVO1);
  my_servo2.attach(SERVO2);
  my_servo3.attach(SERVO3);
  my_servo4.attach(SERVO4);
  
  // Use built in LED for indicating error
  pinMode(LED_BUILTIN, OUTPUT);
  bool Error_LED = false;

  // Use Buzzer to indicate calibration complete
  pinMode(BUZZ_PIN, OUTPUT);

  setUpMemory();
  readMemory();
  blockDevice.deinit();

}

void loop() {

  /****************************************************/
  // Debug Code
  Serial.print("Altitude: ");
  Serial.print(rocketPos[2], 4); Serial.println(" (m)");
  
  Serial.print("Orientation: ");
  Serial.print("X: ");
  Serial.print(rocketAngPos[0], 4);
  Serial.print("\tY: ");
  Serial.print(rocketAngPos[1], 4);
  Serial.print("\tZ: ");
  Serial.print(rocketAngPos[2], 4);
  Serial.println("");

  delay(1000);
  /****************************************************/
}

void setUpMemory(){ 

  if(blockDevice.init() != 0 || blockDevice.size() != BLOCK_DEVICE_SIZE) {    
    Serial.println("Partitioning block device...");
    blockDevice.deinit();
    // Allocate a FAT 32 partition
    MBRBlockDevice::partition(&root, 1, PARTITION_TYPE, 0, BLOCK_DEVICE_SIZE);
    blockDevice.init();
  }

}

void readMemory(){

    const auto size = blockDevice.get_read_size();
    char buffer[size] {};
    blockDevice.read(buffer, 0,size);
    Serial.println(buffer);
    FILE * fPtr;    
    fPtr = fopen("log.txt", "w");
    if(fPtr == NULL)
    {
        Serial.println("Unable to create file.\n");
        exit(EXIT_FAILURE);
    }
    fputs(buffer, fPtr);
    fclose(fPtr);
    
}

void writeToMemory(String toWrite){

  const auto eraseBlockSize = blockDevice.get_erase_size();
  const auto programBlockSize = blockDevice.get_program_size();
  const auto messageSize = toWrite.length() + 1;
  const unsigned int requiredEraseBlocks = ceil(messageSize / (float)  eraseBlockSize);
  const unsigned int requiredBlocks = ceil(messageSize / (float)  programBlockSize);
  const auto dataSize = requiredBlocks * programBlockSize;  
  blockDevice.erase(memoryCursor, requiredEraseBlocks * eraseBlockSize);
  blockDevice.program(toWrite.c_str(), memoryCursor, dataSize);
  memoryCursor = memoryCursor + requiredBlocks; // maybe + 1 or + dataSize

}

void writeBaro(){

  String toWrite = "Altitude (Baro): " + String(rocketPos[2]);
  toWrite = toWrite + "\nZ-Velocity (Baro): " + String(rocketVel[2]);
  writeToMemory(toWrite);

}
void writeIMU(){

  String toWrite = "X-Ang: " + String(rocketAngPos[0]);
  toWrite = toWrite + "\nY-Ang: " + String(rocketAngPos[1]);
  toWrite = toWrite + "\nZ-Ang: " + String(rocketAngPos[2]);
  toWrite = toWrite + "\nY-Accel: " + String(rocketAcc[0]);
  toWrite = toWrite + "\nY-Accel: " + String(rocketAcc[1]);
  toWrite = toWrite + "\nZ-Accel: " + String(rocketAcc[2]);
  toWrite = toWrite + "\nX-AngVel: " + String(rocketAngVel[0]);
  toWrite = toWrite + "\nY-AngVel: " + String(rocketAngVel[1]);
  toWrite = toWrite + "\nZ-AngVel: " + String(rocketAngVel[2]);

  writeToMemory(toWrite);

}

void clearMemory(){

}
