#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

#include <Arduino.h>
#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "NAxisMotion.h" //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

#define BUFFER_SIZE 201 //size of the arrays (equal to total runtime divided by stream period but can be longer) 1000000/50000 = 200

//arrays storing the time, position, and velocity measurements
extern long t[BUFFER_SIZE];

//motor1 data
extern float position1[BUFFER_SIZE];
extern float velocity1[BUFFER_SIZE];
extern float error1[BUFFER_SIZE];
extern float voltage1[BUFFER_SIZE];
extern float control1[BUFFER_SIZE];

//motor2 data
extern float position2[BUFFER_SIZE];
extern float velocity2[BUFFER_SIZE];
extern float error2[BUFFER_SIZE];
extern float voltage2[BUFFER_SIZE];
extern float control2[BUFFER_SIZE];

//IR sensor data
extern float ir1Data[BUFFER_SIZE];
extern float ir2Data[BUFFER_SIZE];
extern float ir0Data[BUFFER_SIZE];

//Direction the robit is facing
extern float heading[BUFFER_SIZE]; //in degrees
extern char orientation[BUFFER_SIZE]; //cardinal directions

extern unsigned int arrayIndex; //index of the arrays 

void dataSetup();
void incrementIndex();
void collectData(unsigned long currentTime);
void printData();
void printArray(float arr[]);
void printArrayLong(long arr[], String text);
long getEncoderValue(int encoder);
void selectEncoder(int encoder);
void deselectEncoder(int encoder);
void LS7366_Init(void);

#endif