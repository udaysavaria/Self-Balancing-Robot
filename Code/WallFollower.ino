int prev_state = -1;

int getState()
{
  int state = -1;
  if (leftPing.getPingCm() >= pingLeftThreshold && frontPing.getPingCm() >= pingFrontThreshold)
    state = 0;
  else if (leftPing.getPingCm() >= pingLeftThreshold && frontPing.getPingCm() <= pingFrontThreshold)
    state = 1;
  else if (leftPing.getPingCm() <= pingLeftThreshold && frontPing.getPingCm() >= pingFrontThreshold)
    state = 2;
  else if (leftPing.getPingCm() <= pingLeftThreshold && frontPing.getPingCm() <= pingFrontThreshold)
    state = 3;
  
//  Serial.print("State= ");
//  Serial.print(state);
//  Serial.print(" left ping= ");
//  Serial.print(leftPing.getPingCm());
//  Serial.print(" front ping= ");
//  Serial.println(frontPing.getPingCm());
  return state;
}


void wallFollowLeft() {
  int state = getState();
//  
  Serial.print("State= ");
  Serial.print(state);
  Serial.print(" left ping= ");
  Serial.print(leftPing.getPingCm());
  Serial.print(" front ping= ");
  Serial.println(frontPing.getPingCm());

  
  if (state == 0) {
    // Nothing on Left and Front
    //turn left in place
    executeLeft90();
  } else if (state == 1) {
    //turn right
    //Should never happen because we'll never be at a front wall with nothing on the left
    executeRight90InPlace();
  } else if (state == 2) {
    //forward
    // Nothing on Front and Wall on Left side
//    Serial.print("\nGoing Forward \n");
    turnState = false;
    goForward();
    fixTheAngle();
  } else if (state == 3) {
    //turn right
    // Wall on Front and Left both
    executeRight90InPlace();
  }
  prev_state = state;
}

boolean hasStateChanged() {
  return getState() != prev_state;
}


void keepTheDistance()
{
  long currentDist = leftPing.getPingCm();
  error = 10 - currentDist;

  double pingKp = 40, pingKd = 30;
  unsigned long now = millis();
  unsigned long int timeStep = now - lastTime;
  errorDot = (error - lastError) / timeStep;

  //PWM calculation for motor torque using PD Control
  pwm = (pingKp * error) + (pingKd * errorDot);
  Serial.print("\nPWM = ");
 
  pwm = constrain(pwm, -15, 15);
  Serial.println(pwm);
//  if(pwm < 0){
//    if(pwm < -20){
//      pwm = -20;
//    }else{
//      pwm = -10;
//    }
//  }
//
//  if(pwm > 0){
//    if(pwm > 20){
//      pwm = 20;
//    }else{
//      pwm = 10;
//    }
//  }




  lastTime = now;
  lastError = error;
}


