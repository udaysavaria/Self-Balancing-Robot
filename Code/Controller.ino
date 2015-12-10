/*****************************
  INTERRUPT DETECTION ROUTINE
 ******************************/

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/**********************
   Stops Motors if Fault
 *********************/
void stopIfFault()
{
  if (motors.getFault())
  {
    Serial.println("fault");
    while (1);
  }
}

/**************
   Getting angle form MPU
 ************/
void MPUCalc()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
  }
  //Getting the ypr[2] angle values from MPU
  //  theta = ypr[2] * 180 / M_PI;
  angle = ypr[0] * 180 / M_PI;
  //
  //    Serial.print("ypr\t");
  //    Serial.println(ypr[0] * 180 / M_PI);
  //    Serial.print("\t");
  //    Serial.print(ypr[1] * 180 / M_PI);
  //    Serial.print("\t");
  //    Serial.println(ypr[2] * 180 / M_PI);
}

double diff = 0.01;
//check v1 against v2, and v2 against v3 for differences
boolean checkStability(double v1, double v2, double v3, double v4, double v5) {
  if (v1 == v2 && v2 == v3 && v3 == v4 && v4 == v5) {
    return true;
  }
  return false;
}


boolean isStable = false;
//return an angle
double getStabilizedAngle() {
  double pAngle = 0.0;
  double ppAngle = 0.0;
  double pppAngle = 0.0;
  double ppppAngle = 0.0;

  unsigned long startTime = millis();

  for (int i = 0; i < 10; i++) {
    MPUCalc();
  }

  while (millis() - startTime < 20000) {

    ppppAngle = pppAngle;
    pppAngle = ppAngle;
    ppAngle = pAngle;
    pAngle = angle;
    MPUCalc();//sets new angle

    if (checkStability(angle, pAngle, ppAngle, pppAngle, ppppAngle)) {
      Serial.print("Stabilized at angle ");
      Serial.println(angle);
      Serial.print("Took time: ");
      Serial.println(millis() - startTime);
      return angle;
    }
  }

  Serial.println("Never stabilized");
  return -1;
}

void updateGain()
{
  if (Serial.available() > 0)
  {
    int x = Serial.read();
    if (x == '4')
    {
      Kp += 5;
    }
    else if (x == '1')
    {
      Kp -= 5;
    }
    else if (x == '5')
    {
      Kd += 5;
    }
    else if (x == '2')
    {
      Kd -= 5;
    }
    else if (x == '6')
    {
      Ki += 0.05;
    }
    else if (x == '3')
    {
      Ki -= 0.05;
    }

    else if (x == 'w')
    {
      setPoint += 0.10;
    }
    else if (x == 's')
    {
      setPoint -= 0.10;
    }

    else if (x == '0')
    {
      usePID = -1 * usePID;
    }
    if (usePID < 0)
    {
      Serial.print("PD");
      Serial.print("\t");
    }
    else
    {
      Serial.print("PID");
      Serial.print("\t");
    }
    Serial.print(Kp);
    Serial.print("\t");
    Serial.print(Kd);
    Serial.print("\t");
    Serial.print(Ki);
    Serial.print("\t");
    Serial.println(setPoint);
  }

}


/*************
   PID
 ***********/
void PIDCompute()
{
  error = setPoint - theta;
  error = -error;
  unsigned long now = millis();
  unsigned long int timeStep = now - lastTime;
  //thetaDot = (theta - lastTheta) / timeStep;
  errorDot = (error - lastError) / timeStep;

  if (abs(error) < IntThresh  )
  { // prevent integral 'windup'          && (lastError*error) > 0 for later
    Integral = Integral + (error * timeStep);           // accumulate the error integral
  }
  else
  {
    Integral = 0; // zero it if out of bounds
  }

  // PWM calculation for motor torque using PID control
  pwm = (Kp * error) + (Ki * Integral) + (Kd * errorDot);
  // keep it in within the range of motor driver speed range
  pwm = constrain(pwm, -400, 400);

  lastTime = now;
  lastError = error;
  lastTheta = theta;
}


/*************
   PD
 ***********/
void PDCompute()
{
  error = setPoint - theta;

  //Because the motor wires are connected in reverse and too lazy to change that
  error = -error;
  unsigned long now = millis();
  unsigned long int timeStep = now - lastTime;
  errorDot = (error - lastError) / timeStep;

  //PWM calculation for motor torque using PD Control
  pwm = (Kp * error) + (Kd * errorDot);
  pwm = constrain(pwm, -400, 400);

  lastTime = now;
  lastError = error;
  lastTheta = theta;
}


/*****************
   Robot Movement codes
 *****************88*/

void goForTime(int motorSpeedRight, int motorSpeedLeft, int timeToGoForward) {
  motors.setSpeeds(motorSpeedRight, motorSpeedLeft);
  delay(timeToGoForward);
}

//function that will wait the timeToWait, OR break out sooner if the state changes.
//If state changes, we want to go back to the point in the code where we figure out current state and run associated code
boolean waitForTimeOrInterrupt(long timeToWait) {
  unsigned long int start = millis();
  boolean interrupted = false;
  while ((millis() - start) < timeToWait) {
    if (hasStateChanged()) {
      interrupted = true;
      Serial.println("State changed sensed.");
      break;
    }
  }
  return interrupted;
}

//Carles calculated
long turn90Number = 1450;//1630 ;//1830;

//robot speed we want to run at
int rSpeed = 200;


void goForward() {
  //function sets the pwms
  keepTheDistance();
  motors.setSpeeds(rSpeed - pwm, rSpeed + pwm);
  Serial.print ("\nexecuting Forward\n");
  //12/7 changed from waitForTime to delay()
  delay(25);
//  waitForTimeOrInterrupt(100000000);
}

double difference(double v1, double v2) {
  return abs(v1) - abs(v2);
}

void fixTheAngle() {
  //angleDiffThreshold, headingAngle, angle

  //headingangle was -61
  //angle was -57
  //turned left, increases the value

  if (difference(headingAngle, angle) > bigAngleThreshold) {
    Serial.print("headingAngle= ");
    Serial.print(headingAngle);
    Serial.print(". angle=");
    Serial.print(angle);
    if (headingAngle < angle) {
      Serial.print("headingAngle < angle");
      //aimed left
      //adjust to the right

      while (difference(headingAngle, angle) > smallAngleThreshold) {
        Serial.println("ADJUSTING TO THE RIGHT");
        motors.setSpeeds(-100, 100);
        delay(25);
        MPUCalc();
      }

    } else {
      Serial.print("headingAngle < angle");
      //headingAngle > angle
      //aimed right
      //adjust to the left
      while (difference(headingAngle, angle) > smallAngleThreshold) {
        Serial.println("ADJUSTING TO THE LEFT");
        motors.setSpeeds(100, -100);
        delay(25);
        MPUCalc();
      }
    }
  }

}

//just go forward
void keepForward() {
  motors.setSpeeds(rSpeed, rSpeed);
  Serial.print ("\nkeep Forward\n");
  waitForTimeOrInterrupt(100000000);
  // goForwardForTime(200);
}

void executeLeft90() {
  if (turnState == false)
  {
    turnState = true;
    Serial.print ("\nExecuting Left 90");
    //goForTime() parameters are p1: right speed, p2: left speed, p3: time)
    //go forward for 200ms
    goForTime(rSpeed, rSpeed, 200);
    //turn left 90 degrees
    goForTime(rSpeed, 0, turn90Number);
    headingAngle += 90;
    if (headingAngle > 180) {
      headingAngle  -= 360;
    }
    //go forward indefinitely until state change
    //    Serial.print("\nleft-forward indef");
    //    goForTime(rSpeed, rSpeed, 400);

    motors.setSpeeds(rSpeed, rSpeed);
//    waitForTimeOrInterrupt(10000000);
    //coming off of a turn state

  } else {
    keepForward();
  }
}

//go forward for 1 second
//then go right for 1 second OR less if state changes
void executeLeft90Inplace() {
  Serial.print ("\nexecuting left 90 Inplace");
  motors.setSpeeds(rSpeed, -rSpeed);
  headingAngle += 90;
  if (headingAngle > 180) {
    headingAngle  -= 360;
  }

  //    waitForTimeOrInterrupt(turn90);
  delay(turn90Number / 2);

}

void executeRight90() {
  Serial.print ("\nexecuting right 90");
  //  goForwardForTime(1000);
  motors.setSpeeds(0, rSpeed);
  headingAngle -= 90;
  if (headingAngle < -180) {
    headingAngle  += 360;
  }
  //  waitForTimeOrInterrupt(turn90);
  delay(turn90Number);
}

void executeRight90InPlace() {
  Serial.print ("\nexecuting right 90 Inplace");

  if (turnState == false)
  {
    turnState = true;
    motors.setSpeeds(-rSpeed, rSpeed);
    headingAngle -= 90;
    if (headingAngle < -180) {
      headingAngle  += 360;
    }
    //  waitForTimeOrInterrupt(turn90);
    delay(turn90Number / 2);
  }
  else
  {
    keepForward();
  }
}

void executeStop() {
  Serial.print ("\nStop");
  motors.setSpeeds(0, 0);
}


