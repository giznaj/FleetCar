/*
  FLEET CAR (TBD)
  ## NOTES ##
  # Car works the best in an environment that is white
  POC
*/

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

//ultrasonic sensors - max distance for all sensors
#define maxDistance 300 //200 centimeters = 6.6 feet

//front right ultrasonic sensor
#define echoPin1 30 //microphone (listening for ping)
#define trigPin1 31 //speaker (making ping sound)

//front left ultrasonicsensor
#define echoPin2 32 //microphone (listening for ping)
#define trigPin2 33 // speaker (making ping sound)

//middle rear ultrasonic sensor
#define echoPin4 34 //microphone (listening for ping)
#define trigPin4 35 //speaker (making ping sound)

//ir sensors (looking out the car from the driver's seat)
#define pinFrontLeft 40 //front left IR sensor
#define pinFrontCentre 42 //front centre IR sensor
#define pinFrontRight 44 //front right IR sensor
#define pinRearRight 46 //rear right IR sensor
#define pinRearLeft 48//rear left IR sensor

//servos
#define servoPinSteering 10 //turning servo (front wheels)
#define servoPinReverse 9 //reverse camera/ultrasonic servo

AF_DCMotor motor(1); //driving motor (back wheels)
Servo servoSteering; //steering servo
Servo servoReverse; //reverse camera/ultrasonic sensory
NewPing hcsr04Right(trigPin1, echoPin1, maxDistance); //front right ultrasonic sensor
NewPing hcsr04Left(trigPin2, echoPin2, maxDistance); //front left ultrasonic sensor
NewPing hcsr04MiddleRear(trigPin4, echoPin4, maxDistance); //back ultrasonic sensor

//front end wheels/servo
float initialAngle = 90.0; //initial angle of the direction the wheels face (straight ahead)
float minAngle = 69.0; //lowest angle we'll let the servo turn left (so we don't break the steering column)
float maxAngle = 111.0; //highest angle we'll let the servo turn right (so we don't break the steering column)
float rightDistance = 0.0; //(previous) distance from wall/object to front right wheel
float leftDistance = 0.0; //(previous) distance from wall/object to front left wheel
float rightCurrentDistance = 0.0; //(current) distance from wall/object to front left wheel
float leftCurrentDistance = 0.0; //(current) distance from wall/object to front left wheel
float currentAngle = 0.0; //current angle (when being checked before it is probably moved)
const float turnIncrement = 5.0; //the angle amount the steering wheel will turn left or right
const float deltaThreshold = 60.0; //the distance to set the signicance or willingness to turn.  How much more or less one side has other the other.  This can be changed to increase performance - CAREFUL!
float distanceTotal = 0.0; //total distance (sum of right and left distances)
float distanceTolerance = 13.0; //the distance to account for noise (ping echo distance).  Delta between distance and current distance is valid or an error. (the lower the number, more aggressive steering)
float distanceDelta = 0.0; //the difference between the distances of both sides (i.e.  left tire is 90 and right tire is 110.  the delta is the abs(20) |20|.
int rightAbsoluteValue = 0; //absolute value of right distance
int leftAbsoluteValue = 0; //absolute value of left distance

//front IR sensor - //HIGH - means no obstacle in view //LOW - means there is an obstacle in view. HIGH == 1, LOW == 0
int isObjectInFront = 1;
int isObjectBehind = 1;

//back camera
float lookLeftAngle = 25.0; //the angle the rear camera points when looking left (as driver looks over the left shoulder)
float lookRightAngle = 155.0; //the angle the rear camera points when looking right (as driver looks over the right shoulder)
float lookStraightBackAngle = 82.0; //the angle the rear camera points when it is look directly backwards
float reverseDistance = 0.0; //(current) distance from the wall behind the car while backing up (turning around)
float backLeftDistance = 0.0; //distance to object when looking back over left shoulder
float backRightDistance = 0.0; //distance to object when looking back over right shoulder
float isMovingDistance1 = 0.0; //first distance to check (see if car is moving forward or not)
float isMovingDistance2 = 0.0; //second distance to check (see if car is moving forward or not)

//dc motor - speeds
int carSpeeds[255];
int carGearFwd = 0; 
int carGearRev = 0;
int carGear = 0;
int iRList[4] = {1, 1, 1, 1}; //HIGH = no object, LOW = object 1 = HIGH, 0 = LOW

//global vars (both front and back sensors)
const float rearCollisionDistance = 900.0;
const float frontCollisionDistance = 300.0;
const int turnAroundDelay = 500;
const int lookDelay = 600;

void setup()
{
  Serial.begin(9600);
  Serial.print("MTO AI Fleet Car");
  Serial.println();

  //setup speeds/gears array
  for (int i = 0; i < 255; i++)
  {
    carSpeeds[i] = i;
  }
  
  //delay
  delay(3000);

  //obstacle detection IR
  pinMode(pinFrontLeft, INPUT);
  pinMode(pinFrontCentre, INPUT);
  pinMode(pinFrontRight, INPUT);
  pinMode(pinRearRight, INPUT);
  pinMode(pinRearLeft, INPUT);
  
  //servo - steering
  servoSteering.attach(servoPinSteering);
  
  //distance sensors (right and left).  need to add these to a state machine call
  rightDistance = getRightDistance();
  delay(250);
  leftDistance = getLeftDistance();

  //delay
  delay(1000);

  //servo - reverse camera
  servoReverse.attach(servoPinReverse);
  lookLeft(); //look over left shoulder (rear camera/sonic sensor)
  lookRight(); //look over right shoulder (rear camera/sonic sensor)
  lookStraightBack(); //look straight back 
  delay(1000);
  
  //motor
  motor.setSpeed(carSpeeds[carGearFwd]); 
  //run the car forward until something good or bad happens
  driveMotor();
}

//checks to see if there is an object in front of the IR sensors (in front of car)
//HIGH - means no obstacle in view //LOW - means there is an obstacle in view. HIGH == 1, LOW == 0
//this method only checks the front 3 IR sensors when moving forward
int getIsObstacleInFront()
{
  isObjectInFront = 1;
  iRList[0] = digitalRead(pinFrontLeft);
  iRList[1] = digitalRead(pinFrontCentre);
  iRList[2] = digitalRead(pinFrontRight);
  //iRList[3] = digitalRead(pinRightRight);
  //iRList[4] = digitalRead(pinRearLeft);
  for (int counter = 0; counter < 3; counter++)
  {
    if(iRList[counter] == 0)
    {
      isObjectInFront = 0;
    }
  }
  return isObjectInFront;
}

//checks to see if there is an object in front of the IR sensors (behind car)
//HIGH - means no obstacle in view //LOW - means there is an obstacle in view. HIGH == 1, LOW == 0
//this method only checks the back 2 IR sensors when moving reverse
int getIsObstacleBehind()
{
  isObjectBehind = 1;
  //iRList[0] = digitalRead(pinFrontLeft);
  //iRList[1] = digitalRead(pinFrontCentre);
  //iRList[2] = digitalRead(pinFrontRight);
  iRList[3] = digitalRead(pinRearRight);
  iRList[4] = digitalRead(pinRearLeft);
  for (int counter = 3; counter < 5; counter++)
  {
    if(iRList[counter] == 0)
    {
      isObjectBehind = 0;
    }
  }
  return isObjectBehind;
}

//get the distance from the wall (Right side of car)
float getRightDistance()
{
  rightCurrentDistance = float(hcsr04Right.ping_median(2));
  return rightCurrentDistance;
}

//get the distance from the wall (Left side of car)
float getLeftDistance()
{
  leftCurrentDistance = float(hcsr04Left.ping_median(2));
  return leftCurrentDistance;
}

//get the distance from the wall using the rear camera/sensor
float getRearDistance()
{
  return float(hcsr04MiddleRear.ping_median(2));
}

//drive the motor
void driveMotor()
{
  motor.run(FORWARD);
}

void reverseMotor()
{
  motor.run(BACKWARD);
}

//stop the motor
void stopMotor()
{
  motor.run(RELEASE);
}

//turn right
bool turnRight()
{
  currentAngle = servoSteering.read();
  float moveAngle = (currentAngle - turnIncrement);
  if (moveAngle > minAngle)
  {
    servoSteering.write(moveAngle);
    delay(20); //let motor take time to get to the new position
    return true;
  }
  else
  {
    return false;
  }
}

//turn left
bool turnLeft()
{
  currentAngle = servoSteering.read();
  float moveAngle = (currentAngle + turnIncrement);
  if (moveAngle < maxAngle)
  {
    servoSteering.write(moveAngle);
    delay(20); //let motor take time to get to the new position
    return true;
  }
  else
  {
    return false;
  }
}

bool lookLeft()
{
  servoReverse.write(lookLeftAngle);
  delay(lookDelay);
  return true;
}

bool lookRight()
{
  servoReverse.write(lookRightAngle);
  delay(lookDelay);
  return true;
}

bool lookStraightBack()
{
  servoReverse.write(lookStraightBackAngle);
  delay(lookDelay);
  return true;
}

bool isMoving()
{
  float isMovingDistance = 0.0;
  //isMovingDistance1 = float(hcsr04MiddleRear.ping_median(2));
  isMovingDistance1 = hcsr04MiddleRear.ping_cm();
  delay(15);
  isMovingDistance2 = hcsr04MiddleRear.ping_cm();
  isMovingDistance = isMovingDistance2 - isMovingDistance1;
  Serial.print(isMovingDistance);
  Serial.print("\n");
  if (isMovingDistance > 1.0)
  {
    return true;   
  }
  else
  {
    return false;
  }
}

//check blind spots and then turn around
void turnAround()
{
  motor.setSpeed(carSpeeds[carGearRev]);
  
  lookLeft();
  backLeftDistance = float(hcsr04MiddleRear.ping_median(2));
  
  lookRight();
  backRightDistance = float(hcsr04MiddleRear.ping_median(2));
  
  lookStraightBack();
  
  if(backLeftDistance > backRightDistance) //turn around clockwise (because there is more space behind back left - when reversing)
  {
    lookLeft(); //look over left shoulder
    do { //turn wheels left
      //execution code is in while condition.  need to refactor
    } while (turnLeft());
    delay(turnAroundDelay);
    
    reverseMotor(); //start backing up until collision detection
    do {
    } while (getIsObstacleBehind() == 1);
    stopMotor();
    delay(turnAroundDelay);
    
    do { //turn wheels right
      //execution code is in condition.  need to refactor
    } while (turnRight()); //turn right
    delay(turnAroundDelay);
    
    driveMotor(); //drive forward
    do {
      rightCurrentDistance = getRightDistance();
    } while (getIsObstacleInFront() == 1);
    stopMotor();
    delay(turnAroundDelay);

    do { //turn wheels left
      //execution code is in while condition.  need to refactor
    } while (turnLeft());
    delay(turnAroundDelay);

    reverseMotor(); //start backing up until collision detection
    do { 
    } while (getIsObstacleBehind() == 1);
    stopMotor();
    delay(turnAroundDelay);
  }
  
  else if(backRightDistance > backLeftDistance) //turn around counter clockwise
  {
    servoReverse.write(lookRightAngle); //look over right shoulder
    delay(turnAroundDelay);

    do { //turn wheels right
      //execution code is in while condition.  need to refactor
    } while (turnRight());
    delay(turnAroundDelay);
    
    reverseMotor(); //start backing up until collision detection
    do {
    } while (getIsObstacleBehind() == 1);
    stopMotor();
    delay(turnAroundDelay);
    
    do { //turn wheels left
      //execution code is in condition.  need to refactor
    } while (turnLeft()); //turn left
    delay(turnAroundDelay);
    
    driveMotor(); //drive forward
    do {
      leftCurrentDistance = getLeftDistance();
    } while (getIsObstacleInFront() == 1);
    stopMotor();
    delay(turnAroundDelay);

    do { //turn wheels right
      //execution code is in while condition.  need to refactor
    } while (turnRight());
    delay(turnAroundDelay);

    reverseMotor(); //start backing up until collision detection
    do {
    } while (getIsObstacleBehind() == 1);
    stopMotor();
    delay(turnAroundDelay);
  }
  else
  {
    //reverseMotor();
    //delay(2000);
  }
  servoReverse.write(lookStraightBackAngle); //look straight back (rear camera/sonic sensor)
  delay(500);
  servoSteering.write(initialAngle);
  delay(500);
  driveMotor();
}

//gear down
void gearDown()
{
  if(carSpeeds[carGearFwd] > 3)
  {
    carGearFwd = carGearFwd-1;
    motor.setSpeed(carSpeeds[carGearFwd]);
  }
}

//gear up
void gearUp()
{
  if(carSpeeds[carGearFwd] < 35)
  {
    carGearFwd = carGearFwd+1;
    motor.setSpeed(carSpeeds[carGearFwd]);
  }
}

//main method into the program
void loop() 
{
  if(isMoving() == false)
  {
    gearUp(); 
  }
  
  //need to add these to a state machine call
  rightCurrentDistance = getRightDistance(); //get the current distance (right side)
  leftCurrentDistance = getLeftDistance(); //get the current distance (left side)
  distanceDelta = abs(rightCurrentDistance - leftCurrentDistance); //absolute value of the difference between the left and right sides
  //check IR sensors for an object
  if (getIsObstacleInFront() == 0) //object is in front of car
  {
    //stop car and start turn-around process
    stopMotor();
    turnAround();
  }
  else if(distanceDelta < 60.0)
  {
    //do nothing for now
  }
  else
  {
    if (leftCurrentDistance > rightCurrentDistance)
    {
      turnLeft();
    }
    else if (rightCurrentDistance > leftCurrentDistance)
    {
      turnRight();
    }
    else
    {
      //do nothing for now      
    }
    leftDistance = leftCurrentDistance;
    rightDistance = rightCurrentDistance;
  }
}
