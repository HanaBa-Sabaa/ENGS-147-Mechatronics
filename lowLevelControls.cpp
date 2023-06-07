#include "lowLevelControls.h"

ArduinoMotorShieldR3 md;

bool flag = false;

/*
initialize the motor shield so that the system can operate both motors
*/
void controlSetup(){
  md.init();
}

/*
This speed uses a PID contorller to moderate the speed of the motors and make sure they reach the steady-state.
The sped yoou want is passed as "refSpeed" and the motor you want to control is passed as "motoID"
*/
void controlSpeed(float refSpeed, int motorID){

  //variables for storing necessary data
  float lastError = 0; //last error calculation
  float lastVoltage = 0; //last voltage signal

  float currentError; //calculated from 'currentVelocity' and 'refSpeed'
  float currentVoltage; //calculated from compensator difference equation
  float currentVelocity; //taken from the velocity data array
  int controlSignal; //PWM command sent to the motor
  

  if (motorID == 1){//if we're controlling motor
    currentVelocity = velocity1[arrayIndex];//grab the most recent velocity measurement
    lastError = error1[arrayIndex - 1];//last time step's error/voltage
    lastVoltage = voltage1[arrayIndex - 1];
  }
  else if (motorID == 2){//if we're controlling motor2
    currentVelocity = velocity2[arrayIndex];
    lastError = error2[arrayIndex - 1];
    lastVoltage = voltage2[arrayIndex - 1];
  }
  else{
    Serial.println("missing or incorrect motorID");//if the motor ID isn't recognized sned a notification
  }

  if(refSpeed > 0){
    currentError = refSpeed - currentVelocity;//calculate the error between the 'refSpeed' and the 'currentVelocity'
  }
  else{
    currentError = refSpeed + currentVelocity;//calculate the error between the 'refSpeed' and the 'currentVelocity'
  }

  if(arrayIndex == 0){//difference equation for fist time step
    currentVoltage = 1.688*currentError;
  }
  else{//difference equation for all other time steps
    currentVoltage = 1*lastVoltage + 1.75*currentError - 1*lastError; //I was testing this differencce equation
  }

  //make sure voltage does not exceed ±9.6V
  if(currentVoltage > 9.6){
    currentVoltage = 9.6;    
  }
  if(currentVoltage < -9.6){
    currentVoltage = -9.6;
  }

  controlSignal = getPWM(currentVoltage); //grab the PWM signal based on th voltage

  if (motorID == 1){//if we're controlling motor1
    md.setM1Speed(controlSignal);//set the motor speed with the found PWM command
    error1[arrayIndex] = currentError; //set the current error, voltage, and PWM signal
    voltage1[arrayIndex] = currentVoltage;
    control1[arrayIndex] = controlSignal;
  }
  else if (motorID == 2){//if we'rre controlling motor2
    md.setM2Speed(controlSignal);
    error2[arrayIndex] = currentError;
    voltage2[arrayIndex] = currentVoltage;
    control2[arrayIndex] = controlSignal;
  }
}

/*
This function shutts off the motors
*/
void motorOff(int motorID){
  if (motorID ==1){//if we're controlling motor1
    md.setM1Speed(0);//set the motor speed with the found PWM command
  }
  else if (motorID == 2){//if we'rre controlling motor2
    md.setM2Speed(0);
  }
}

void wallFollower(float refSpeed){
  float limit = 4; //rad/sec
  float minSpeed = 3;
  float refDistance = 12; //cm from the wall

  float ref1 = refSpeed;
  float ref2 = refSpeed;

  float rightDistance = ir2Data[arrayIndex] - refDistance;
  float leftDistance = ir1Data[arrayIndex] - refDistance;

  float rightDistance5 = rightDistance;
  float leftDistance5 = leftDistance;

  if(arrayIndex > 5){
  rightDistance5 = ir2Data[arrayIndex - 5] - refDistance;
  leftDistance5 = ir1Data[arrayIndex - 5] - refDistance;
  }

  float dref2 = (rightDistance5 - rightDistance);

  if(rightDistance < 0){
    dref2 = dref2 - 0.5*(rightDistance);
  }
  if(dref2 > limit || dref2 < -limit){
    dref2 = 0;
  }

  ref2 = ref2 + dref2;
  if(ref2 < minSpeed){
    ref2 = minSpeed;
  }

  float dref1 = (leftDistance5 - leftDistance);

  if(leftDistance < 0){
    dref1 = dref1 - 0.5*leftDistance;
  }
  if(dref1 > limit || dref1 < -limit){
    dref1 = 0;
  }

  ref1 = ref1 + dref1;
  if(ref1 < minSpeed){
    ref1 = minSpeed;
  }

  //Serial.println(ref1);
  controlSpeed(-ref1, 1);
  controlSpeed(ref2, 2);
}

void braker(){
  float maxSpeed = 5; //Maximum speed signal to be sent tto the wheels (rad/sec)
  float frontDistance = ir0Data[arrayIndex]; //IR measureed distance between the front of the MicroMouse and wall (cm)
  float refDistance = 9; //Reference distance that the MicroMouse will settle to (cm)
  float error = frontDistance - refDistance; //error in our current system (cm)
  float speed = 0.5*error; //a proportional gain is applied ot the error calculated in the system

  if(speed > maxSpeed){ //i the spped is to high, set it to maxSpeed
    speed = maxSpeed;
  }

  if (abs(error) < 1){ //if the error is within 1cm of the desired target
    motorOff(1); //turn off the motors
    motorOff(2);
    flag = true; //raise a flag to show the process is done running
  }
  else{
    controlSpeed(-speed, 1); //set thee motor speed to put the MicorMouse closer to the reefeerence distance
    controlSpeed(speed, 2);
  }
}

/*
This is an unused function that we wanted to use to rplace the original turn function
*/
void turn2(char turnDirection){
  float maxSpeed = 4;    // Maximum motor speed in rad/sec
  float minSpeed = 0;    // Minimum motor speed in rad/sec
  float targetAngle;
  float turnAngle;
  float currentAngle = heading[arrayIndex];

  switch (turnDirection){
    case 'n':
      targetAngle = 359;
      break;

    case 'e':  // your hand is close to the sensor
      targetAngle = 90;
      break;

    case 's':  // your hand is a few inches from the sensor
      targetAngle = 180;
      break;

    case 'w':  // your hand is nowhere near the sensor
      targetAngle = 270;
      break;

  }

  turnAngle = (round(currentAngle) - round(targetAngle))%360;   

  float error = abs(targetAngle - currentAngle);
  float speed = 0.1*error;

  if(speed > maxSpeed){
    speed = maxSpeed;
  }
  else if(speed < minSpeed){
    speed = minSpeed;
  }
            
  if (round(turnAngle) > 180){
    controlSpeed(speed , 2);
    controlSpeed(speed , 1); 
  }      

  else{
    controlSpeed(-speed , 2);
    controlSpeed(-speed , 1); 
  }
  if(error < 10){
    motorOff(1); //turn motors off
    motorOff(2);
    flag = true;
  } 
  
}

/*
Actual function used to moderate the turning angle of the mous
*/
void turner(float turnAngle, float startAngle) {
  float speed; //reference speed to be passed into the controlSpeeed function
  float maxSpeed = 8; // Maximum motor speed in rad/sec
  float minSpeed = 0; // Minimum motor speed in rad/sec
  float currentAngle = heading[arrayIndex]; //grab the current angle the mouse is facing
  float targetAngle;

  targetAngle = (round(startAngle) + round(turnAngle))%360; //target angle is equal to the starting angle plus the turning angle

  if(abs(targetAngle - 90) < 44){ //if the target angel is between 46-134 degree (hard set it to 90)
    targetAngle = 90;
  }
  else if(abs(targetAngle - 180) < 44){ //if the target angel is between 136-224 degree (hard set it to 180)
    targetAngle = 180;
  }
  else if(abs(targetAngle - 270) < 44){ //if the target angel is between 226-314 degree (hard set it to 270)
    targetAngle = 270;
  }
  else if(abs(targetAngle - 360) < 44){ //if the target angel is between 316-44 degree (hard set it to 359)
    targetAngle = 359;
  }

  float error = abs(targetAngle - currentAngle);
  const float slope = (maxSpeed-minSpeed)/targetAngle;
  speed = slope*error + 2;
  //speed = 0.15*error;

  if(speed > maxSpeed){
    speed = maxSpeed;
  }
  else if(speed < minSpeed){
    speed = minSpeed;
  }
  //Serial.println(speed);

  if (round(turnAngle) > 180){
    controlSpeed(speed , 2);
    controlSpeed(speed , 1); 
  }      

  else{
    controlSpeed(-speed , 2);
    controlSpeed(-speed , 1); 
  }
  if(error < 2){
    motorOff(1); //turn motors off
    motorOff(2);
    flag = true; //rasie a flag that the process is over
  } 
}
