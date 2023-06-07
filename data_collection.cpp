#include "data_collection.h"

int chipSelectPin1=10;
int chipSelectPin2=9;
int chipSelectPin3=8;

const int ir0 = A3;
const int ir1 = A4;
const int ir2 = A2;

NAxisMotion mySensor;                 //Object that for the sensor
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

//bool hasRun = false; //flag to determine if the loop fnunction has run (used to make sure the loop function only runs once)

signed long basePos1; //initial encoder positions for each motor
signed long basePos2;

long t[BUFFER_SIZE] = {0};

//motor1 data
float position1[BUFFER_SIZE] = {0};
float velocity1[BUFFER_SIZE] = {0};
float error1[BUFFER_SIZE]= {0};
float voltage1[BUFFER_SIZE] = {0};
float control1[BUFFER_SIZE] = {0};

//motor2 data
float position2[BUFFER_SIZE] = {0};
float velocity2[BUFFER_SIZE] = {0};
float error2[BUFFER_SIZE] = {0};
float voltage2[BUFFER_SIZE] = {0};
float control2[BUFFER_SIZE] = {0};

//IR sensor data
float ir1Data[BUFFER_SIZE] = {0};
float ir2Data[BUFFER_SIZE] = {0};
float ir0Data[BUFFER_SIZE] = {0};

//Direction the robit is facing
float heading[BUFFER_SIZE] = {0}; //in degrees
char orientation[BUFFER_SIZE] = {0}; //cardinal directions

unsigned int arrayIndex = 0; //index of the arrays

void dataSetup() {
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  
  //Axis Encoder Initiallization
  pinMode(chipSelectPin1, OUTPUT);
  digitalWrite(chipSelectPin1, HIGH);

  //Axis Encoder Initiallization
  pinMode(chipSelectPin2, OUTPUT);
  digitalWrite(chipSelectPin2, HIGH);
  LS7366_Init();
  
  //Sensor Initialization
  mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  
  basePos1 = getEncoderValue(1); //set the base enoder positions
  basePos2 = getEncoderValue(2);
}

/*
this fnuction is used to collect vital data on the motors and the micomouuse
*/
void collectData(unsigned long currentTime){
  int CPR = 2500; //counts per revolution of the encoder (20 counts per revolution * 125 gear ratio)
  float currentPos1 = abs(getEncoderValue(1) - basePos1)*1.0F/CPR*6.28; //the encoder count of motor1
  float currentPos2 = abs(getEncoderValue(2) - basePos2)*1.0F/CPR*6.28; //the encoder count of motor2
  float currentVel1; //the velocity of motor1
  float currentVel2; //the velocity of motor2
  float currentHeading = mySensor.readEulerHeading();

  if (updateSensorData){ //Keep the updating of data as a separate task
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }

  if(arrayIndex == 0){//set the velocity measurements to 0 at the first time step
    currentVel1 = 0;
    currentVel2 = 0;
  }
  else{
    float dp;//change in position
    float dt;//change in time

    //for motor1
    dp = (currentPos1 - position1[arrayIndex-1]); //calculate change in position from last measurement (rad)
    dt = (currentTime - t[arrayIndex-1])*1.0F/1000000; //calculate change in time from last measurement (sec)
    currentVel1 = (dp / dt); //calcculate current velocity (rad/sec)

    //for motor2
    dp = (currentPos2 - position2[arrayIndex-1]);
    dt = (currentTime - t[arrayIndex-1])*1.0F/1000000;
    currentVel2 = (dp / dt);
  }

  t[arrayIndex] = currentTime; //microseconds

  position1[arrayIndex] = currentPos1;//*1.0F/CPR*6.28; //rad
  velocity1[arrayIndex] = currentVel1; // rad/sec
  error1[arrayIndex] = 0;
  voltage1[arrayIndex] = 0;
  control1[arrayIndex] = 0;

  position2[arrayIndex] = currentPos2;//*1.0F/CPR*6.28; //rad
  velocity2[arrayIndex] = currentVel2; //rad/sec
  error2[arrayIndex] = 0;
  voltage2[arrayIndex] = 0;
  control2[arrayIndex] = 0;

  ir1Data[arrayIndex] = 8085.4 * pow(analogRead(ir1),-1.066);
  ir2Data[arrayIndex] = 8085.4 * pow(analogRead(ir2),-1.066);;
  ir0Data[arrayIndex] = 8085.4 * pow(analogRead(ir0),-1.066);;

  heading[arrayIndex] = currentHeading;

  if(currentHeading < 45 || currentHeading > 315){
    orientation[arrayIndex] = 'n';
  }
  else if(currentHeading < 135){
    orientation[arrayIndex] = 'e';
  }
  else if(currentHeading > 225){
    orientation[arrayIndex] = 'w';
  }
  else{
    orientation[arrayIndex] = 's';
  }
  updateSensorData = true;

  // Serial.print(" H: ");
  // Serial.print(orientation[arrayIndex]); //Heading data
  // Serial.println();
}

/*
This function is used to increment the current arayIndex of th system.
It will also reset the index if it exceeds th BUFFER_SIZE
*/
void incrementIndex(){
  arrayIndex++; //increment the index

  if (arrayIndex >= BUFFER_SIZE) { //if buffer size is exceeded
    arrayIndex = 0;//hard reset the index
    //Serial.println("index has been reset..."); //send a notification (not necessary, but good for debugging)
  }
}

void printData(){
  //All arrays will be printed to the serial monitor, you just need to copy and past into MatLab.
  Serial.println("%Printing...");

  Serial.print("position = ");
  printArray(ir0Data);

  Serial.println("%motor1");
  Serial.print("time = ");
  printArrayLong(t, "/1000000"); // "/1000000" is added to automatically convert the time array into seconds when copied into matlab

  Serial.print("position = ");
  printArray(position1);

  Serial.print("velocity = ");
  printArray(velocity1);

  Serial.print("error = ");
  printArray(error1);

  Serial.print("voltage = ");
  printArray(voltage1);

  Serial.print("command = ");
  printArray(control1);

  Serial.println();


  Serial.println("%motor2");
  Serial.print("time = ");
  printArrayLong(t, "/1000000"); // "/1000000" is added to automatically convert the time array into seconds when copied into matlab

  Serial.print("position = ");
  printArray(position2);

  Serial.print("velocity = ");
  printArray(velocity2);

  Serial.print("error = ");
  printArray(error2);

  Serial.print("voltage = ");
  printArray(voltage2);

  Serial.print("command = ");
  printArray(control2);

  Serial.println();
}

void printArray(float arr[]) {
  Serial.print("[");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(arr[i], 2); // Print the array element with 2 decimal places.
    if (i < BUFFER_SIZE - 1) {
      Serial.print(", ");
    }
  }
  Serial.println("];");  
}

void printArrayLong(long arr[], String text) {
  Serial.print("[");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(arr[i]); // Print the array element with 2 decimal places.
    if (i < BUFFER_SIZE - 1) {
      Serial.print(", ");
    }
  }
  Serial.print("]"); 
  Serial.print(text); 
  Serial.println(";"); 
}

long getEncoderValue(int encoder){
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
     SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

void selectEncoder(int encoder){
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;    
  }//end switch
  
}//end func

void deselectEncoder(int encoder){
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;    
  }//end switch
  
}//end func

// LS7366 Initialization and configuration
void LS7366_Init(void){
   
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 
   
   
   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH); 
   
   
   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH); 
   
}//end func