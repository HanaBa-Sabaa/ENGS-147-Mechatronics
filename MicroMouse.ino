
#include "lowLevelControls.h"

bool hasRun = false; //flag to determine if the loop fnunction has run (used to make sure the loop function only runs once)

//defaul speed for MicroMouse driving (rad/sec)
float ref = 5;
float turnAngle;
float startAngle;
int nextState = 1;

unsigned long baseTime; //initial time(us) of running and used to determine current time
unsigned long currentTime = 0; //the time(us) since the loop started
unsigned long lastSampleTime = 0; //time stamp of the last time motor data was sampled
const long samplePeriod = 50000; //period between data samples (data is collected every 50ms)
int rightCount = 0; //the numbeer of times the car has turned right (4 consective right turns denotes the centr of the maze). We understand that this is not always the case but forr th maze setups used in practice this was not an issue

void setup() {
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  
  dataSetup(); //initialize the encoders and sensor to begin reading data
  controlSetup(); //initialize the motor shield

  Serial.println("Streaming in..."); //Countdown
  Serial.print("3...");
  delay(1000);  //Wait for a second
  Serial.print("2...");
  delay(1000);  //Wait for a second
  Serial.println("1...");
  delay(1000);  //Wait for a second
  Serial.println();
  baseTime = micros(); //set the base time(us) 
}

void loop() {
  if(hasRun == false){ //if the maze is uninished, run the loop
    currentTime = micros() - baseTime; //measure the current time(us) since the program has began running

    if ((currentTime - lastSampleTime) >= samplePeriod){ //if the current time has surpassed the samplePeriod
      lastSampleTime = currentTime; //set the most recent sample time stamp to the current time

      //Serial.println("sample!");
      collectData(currentTime); //collect general data on the motors and 
      solveMaze(); //run the next part of the maze solver state-machine
      checkCenter(); //check if the center has been reeached
      incrementIndex(); //increment the array index
    }
  }
  else{
    controlSpeed(ref, 2); //once the maze is complete, do a little celebratory spin!!
    controlSpeed(ref, 1);
  }
}

/*
This function is used to test the checkWalls() function.
*/
void wallTest(){
  int * walls = checkWalls(); //run the checkWalls()fuunction and store it in an array.
  for (int i = 0; i <= 3; i++){ //print out the 4 elements of wall array
    if (walls[i]){
      Serial.print("wall "); // 1 -> wall present
    } 
    else{
      Serial.print("path "); // 0 -> path present
    }
  }
  Serial.println();
}

/*
This fuunction checks to see if the center has been reached
*/
void checkCenter(){
  if(rightCount >= 4){ //if the MicroMouse has turrned 4 times in a row, the maze is complete
    hasRun = true;
  }
}

/*
The state machine which commands the MicorMouse throuhg every individual step.
It always begins in the driving state and will run continuously unless stopeed externally (see the checkCenter() function)
*/
void solveMaze(){
  int currState; //the current statte of the machine

  float currentAngle = heading[arrayIndex]; //the current angle the MicroMouse is facing
  char currentOrientation; //the current cardinal direction the MicroMous eis facing (based on initiallization)

  unsigned long driveTime = 0; //how long the MicroMouse ha been inf the "driving" state
  unsigned long baseDriveTime = 0; //The time when the MicroMouse enters the "driving" state
  unsigned long lastStopTime = 0; //last time the MicroMouse stopped to check its surrounding walls
  const long stopPeriod = 2350000; //the period it takes the MicroMouse to enter the center of a new cell

  int * walls; //array to store the surround wall locations

  currState = nextState; //set the current state to the next desired state
  
  switch(currState){ //check which state the MicroMouse is in

    case(1): //The driving state

      if(flag){ //a flag is raised whenever a state is left so this will only run when this state is entered the first time
        Serial.println("Driving");
        baseDriveTime = micros(); //set the time when the MicroMouse enters the drriving state
        lastStopTime = 0; //reset the last time the MicroMouse stopped and checked the walls
      }

      driveTime = micros() - baseDriveTime; //current drive time is equal to the current time minus the baseDriveTime

      flag = false;
      wallFollower(ref); //whilee in the drive state only run the wallFollower() function wih the given refernce speed

      if(ir0Data[arrayIndex] < 20){ //if a wall is detected within 20cm of the front o the MicroMouse
        nextState = 2; //set the next state to 2 (braking)
        flag = true; //set the flag to true to show the state is over
      }

      if((driveTime - lastStopTime) > stopPeriod){ //if the MicroMouse has been driving for at least 2.35sec
        Serial.println("Stop");
        lastStopTime = driveTime; //rreset the last time the MicrroMouse stopped
        motorOff(1); //set the motors off (this is t give a physical indication that the mouse is checking its walls)
        motorOff(2);
        nextState = 3; //set the next state to 3 (checking the walls)
        flag = true;
      }

      break;

    case(2)://the braking state

      if(flag){ //when this state is entered for the first time
        Serial.println("Breaking");
      }

      flag = false;
      braker(); //run the braker function

      if(flag){ //when the braker function is done, it raises the flag
        nextState = 3; //set the next state 3 (check the surrounding walls)
        flag = true; //make sure the flag is still raised (should be)
      }
      break;

    case(3): //checking surrounding walls state

      if(flag){//when this state is entered for the first time
        Serial.println("Checking");
      }

      flag = false;
      walls = checkWalls(); //check the location of the surrounding walls and store it
      
      currentOrientation = orientation[arrayIndex]; //find the current orientation of the MicroMouse (North, South, East, West)
      //Serial.println(currentOrientation);

      switch (currentOrientation){ //depending on the current orientation of the MicroMouse the walls a different turn signal is needed

        case 'n': //If the MicroMouse is facing north

          if(!walls[3]){ //if there is no wall to the mouse's left
            turnAngle = 270; //turn the mouse to the left (270 degrees)
            Serial.print("Turn Left: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[1]){ //if there is no wall to the mouse's right 
            turnAngle =  90; //turn the mouse to the right (90 degrees)
            Serial.print("Turn Right: ");
            Serial.println(turnAngle);
            rightCount++; //increment the right turn count
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[0]){ //if there is no wall ahead of the mouse
            turnAngle =  0;
            Serial.print("Straight: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 1; //set the next state to 1 (driving)
          }
          else if(!walls[2]){ //if there is no wall behind the mouse
            turnAngle =  180;
            Serial.print("Turn Around: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else{
            Serial.println("No Direction"); //this is in case there is no definite orientation
          }
          break;

        case 'e':  // your hand is close to the sensor
          if(!walls[0]){//if there is no wall to the mouse's left
            turnAngle = 270; //turn the mouse to the left (270 degrees)
            Serial.print("Turn Left: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[2]){//if there is no wall to the mouse's right 
            turnAngle =  90; //turn the mouse to the right (90 degrees)
            Serial.print("Turn Right: ");
            Serial.println(turnAngle);
            rightCount++; //increment the right turn count
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[1]){//if there is no wall ahead of the mouse
            turnAngle =  0;
            Serial.print("Straight: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 1; //set the next state to 1 (driving)
          }
          else if(!walls[3]){//if there is no wall behind the mouse
            turnAngle =  180;
            Serial.print("Turn Around: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else{
            Serial.println("No Direction");
          }
          break;

        case 's':  // your hand is a few inches from the sensor
          if(!walls[1]){//if there is no wall to the mouse's left
            turnAngle = 270; //turn the mouse to the left (270 degrees)
            Serial.print("Turn Left: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[3]){//if there is no wall to the mouse's right 
            turnAngle =  90; //turn the mouse to the right (90 degrees)
            Serial.print("Turn Right: ");
            Serial.println(turnAngle);
            rightCount++; //increment the right turn count
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[2]){//if there is no wall ahead of the mouse
            turnAngle =  0;
            Serial.print("Straight: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 1; //set the next state to 1 (driving)
          }
          else if(!walls[0]){//if there is no wall behind the mouse
            turnAngle =  180;
            Serial.print("Turn Around: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else{
            Serial.println("No Direction");
          }
          break;

        case 'w':  // your hand is nowhere near the sensor
          if(!walls[2]){//if there is no wall to the mouse's left
            turnAngle = 270; //turn the mouse to the left (270 degrees)
            Serial.print("Turn Left: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[0]){//if there is no wall to the mouse's right 
            turnAngle =  90; //turn the mouse to the right (90 degrees)
            Serial.print("Turn Right: ");
            Serial.println(turnAngle);
            rightCount++; //increment the right turn count
            nextState = 4; //set the next state to 4 (turrning)
          }
          else if(!walls[3]){//if there is no wall ahead of the mouse
            turnAngle =  0;
            Serial.print("Straight: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 1; //set the next state to 1 (driving)
          }
          else if(!walls[1]){//if there is no wall behind the mouse
            turnAngle =  180;
            Serial.print("Turn Around: ");
            Serial.println(turnAngle);
            rightCount = 0; //reset the right turn count (rightt turns must be consecutive)
            nextState = 4; //set the next state to 4 (turrning)
          }
          else{
            Serial.println("No Direction");
          }
          break;
        
      }
      flag = true; //flag that this state is over (it only needs to run once)

      break;

    case(4): //the turning state
      if(flag){ //if tthis is the first time the state is running
        Serial.print("Turning to: ");
        startAngle = currentAngle; //set the starting angle of the turn to the current angle
        Serial.println((round(startAngle) + round(turnAngle))%360); //print the final angle
      }
      flag = false;
      turner(turnAngle, startAngle); //Run the turn funciton with how far the mouse should turn and its initial heading

      if(flag){ //thee turn() function raises a flag when the fnuction is done running
        nextState = 1; //set the next state to 1 (driving state)
        flag = true; //make sure a flag is raised
      }
      break;
  }
}

int * checkWalls(){
  static int wall[4] = {0, 0, 0, 0}; //initialize a variable to store the wall location. Stored as: {North, South, East, West}

  char currentOrientation;
  float refDistance = 20; //maximum distance a wall should be in order to be counted 
  double leftDistance = ir1Data[arrayIndex]; //current distance measurements on the left IR sensor
  double rightDistance = ir2Data[arrayIndex]; //current distance measurements on the right IR sensor
  double frontDistance = ir0Data[arrayIndex]; //current distance measurements on the front IR sensor

  int frontWall = 0; //int to store the pesence of front wall (1-> wall present,  0-> no wall present)
  int leftWall = 0; //int to store the pesence of left wall (1-> wall present,  0-> no wall present)
  int rightWall = 0; //int to store the pesence of right wall (1-> wall present,  0-> no wall present)

  if(frontDistance < refDistance){ //if a wall is detected within the reference distance mark that a wall is thee
    frontWall = 1;
  }
  if(leftDistance < refDistance){
    leftWall = 1;
  }
  if(rightDistance < refDistance){
    rightWall = 1;
  }

  currentOrientation = orientation[arrayIndex];
  switch (currentOrientation){ //the wall array is populated depeending on the orientation of the mouse 
    case 'n':  // your hand is on the sensor
      wall[0] = frontWall;
      wall[1] = rightWall;
      wall[2] = 0;
      wall[3] = leftWall;
      break;
    case 'e':  // your hand is close to the sensor
      wall[0] = leftWall;
      wall[1] = frontWall;
      wall[2] = rightWall;
      wall[3] = 0;
      break;
    case 's':  // your hand is a few inches from the sensor
      wall[0] = 0;
      wall[1] = leftWall;
      wall[2] = frontWall;
      wall[3] = rightWall;
      break;
    case 'w':  // your hand is nowhere near the sensor
      wall[0] = rightWall;
      wall[1] = 0;
      wall[2] = leftWall;
      wall[3] = frontWall;
      break;
  }
  
  return wall;
}
