#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DualMC33926MotorShield.h"
#include "PingSensor.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
DualMC33926MotorShield motors;

#define OUTPUT_READABLE_YAWPITCHROLL
#define BALANCE 1

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PID Controller variables
double theta, thetaDot, error, lastError, setPoint, errorDot;
double headingAngle, angle, angleDot;
//angle to stay within of original heading
double bigAngleThreshold = 10;
double smallAngleThreshold = 3;
unsigned long int lastTime;
float lastTheta;
double Kp = 90;            //Kp = 90;    for small robot    // Kp = 120          //K = 200;     // k = 200 initial working value
double Kd = 335;          //Kd = 335;                    // Kd = 2100          //B = 2100;      // B = 500 initial working value
double Ki = 1.80;          //Ki = 0.5
double Proportional = 0;
double Integral = 0;
double Derivative = 0;
double IntThresh = 1;
int pwm;
int usePID = 1;

// Turning of the robot while balancing
double turn_ratio_l = 1;   // 0<turn_ratio<1
double turn_ratio_r = 1;   // 0<turn_ratio<1
double spinSpeed  = 00;
long int oldTurnTime = 0;
boolean balanceMode = true;


//Ping Sensors
pingSensor frontPing(3);
pingSensor leftPing(13);
long prevPingDist = 0;
long pingFrontThreshold = 15;
long pingLeftThreshold = 20;
boolean turnState; 


void MPUCalc();
void PDCompute();
void PIDCompute();
void updateGain();
void ping();
