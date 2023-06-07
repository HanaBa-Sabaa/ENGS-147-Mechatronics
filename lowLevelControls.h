#ifndef LOWLEVELCONTROLS_H
#define LOWLEVELCONTROLS_H

#include <Arduino.h>
#include "ArduinoMotorShieldR3.h"
#include "voltage2control.h"
#include "data_collection.h"

extern bool flag;

void controlSetup();
void controlSpeed(float refSpeed, int motorID);
void motorOff(int motorID);
void wallFollower(float refSpeed);
void braker();
void turn3(char turnDirection);
void turner(float turnAngle, float startAngle);

#endif