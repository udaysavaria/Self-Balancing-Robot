#include "Declarations.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "DualMC33926MotorShield.h"
#include "Wire.h"

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  motors.init();
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(47);
  mpu.setYGyroOffset(-27);
  mpu.setZGyroOffset(-27);
  mpu.setXAccelOffset(-3071); // mpu.setXAccelOffset(-3071);
  // mpu.setYAccelOffsetle(1497);
  //  mpu.setZAccelOffset(4371); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    setPoint = 0;
  }
}


boolean first = true;
void loop()
{
//
//  motors.setSpeeds(50,50);
//  delay(100);
//  motors.setSpeeds(0,0);
  MPUCalc();
  if(first){
    headingAngle = getStabilizedAngle();
    first = false;
  }
    Serial.print("Angle: ");
  Serial.println(angle);
  //MPUCalc();
  wallFollowLeft();


  /************************
    turning based on turn_ratio and spin(turn zero radius)

    md.setSpeeds(pwm*turn_ratio_l + spinSpeed, pwm*turn_ratio_r - spinSpeed)

   *************************/



  /****************************************

     balancing and turning

    long int turnTime = millis();
    //if want balance
    if (balanceMode) {
      //have been balancing for 200ms
      if (turnTime - oldTurnTime > 2000) {
        //switch to turn mode
        balanceMode = false;
        //store time
        oldTurnTime = millis();
      } else {
        //continue balancing
        md.setSpeeds(pwm, pwm);
      }
      md.setSpeeds(pwm, pwm);
    } else {
      //have been turning for 50ms
      if (turnTime - oldTurnTime > 80) {
        //switch to balance mode
        balanceMode = true;
        //store time
        oldTurnTime = millis();
      } else {
        //continue turning
        md.setSpeeds(135,225);
      }
    }

  ***************************************/
}
