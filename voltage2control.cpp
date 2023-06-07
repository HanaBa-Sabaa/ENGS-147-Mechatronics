#include "voltage2control.h"
/*
This function is used to convert a voltage controrl signal into a Pulse Width Modulated command to be sent to the motors.
The desired voltag is sent in as an arrgunment and the associated PWM command is returned as an int
*/
int getPWM(float voltage){
  int controlSignal;//temp PWM variable to besent to the motors

  if(voltage > 0) {
    controlSignal = -0.3794*pow(voltage,4) + 8.0202*pow(voltage,3) - 52.0403*pow(voltage, 2) + 135.8683*(voltage) + 23.6273; //positive voltage to PWM command polyfit
  }
  else {
    controlSignal = 0.3982*pow(voltage,4) + 8.4556*pow(voltage,3) + 55.0285*pow(voltage, 2) + 141.2458*(voltage) + -23.5285; //negative voltage to PWM command polyfit
  }

  //make sure PWM signal does not exceed ±400
  if(controlSignal > 400) {
    controlSignal = 400;
  }
  if(controlSignal < -400) {
    controlSignal = -400;
  }

  return controlSignal;
}