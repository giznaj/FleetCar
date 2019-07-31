/*
  FLEET CAR (TBD)
  POC
*/

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

//max distance for all sensors
#define maxDistance 200 //200 centimeters = 6.6 feet
//front right sensor
#define echoPin1 30 // microphone (listening for ping)
#define trigPin1 31 // speaker (making ping sound)
//front left sensor
#define echoPin2 32 // microphone (listening for ping)
#define trigPin2 33 // speaker (making ping sound)
//middle front sensor
#define echoPin3 34 // microphone (listening for ping)
#define trigPin3 35 // speaker (making ping sound)
//middle rear sensor
#define echoPin4 36 // microphone (listening for ping)
#define trigPin4 37 // speaker (making ping sound)
//turning servo (turning wheels)
#define servoPinSteering 10 // servo control pin
//looking servo (rotating back sensor)
#define servoPinReverse 9 // reverse camera servo

AF_DCMotor motor(1);
Servo servoSteering;
Servo servoReverse;
NewPing hcsr04Right(trigPin1, echoPin1, maxDistance);
NewPing hcsr04Left(trigPin2, echoPin2, maxDistance);
NewPing hcsr04MiddleFront(trigPin3, echoPin3, maxDistance);
NewPing hcsr04MiddleRear(trigPin4, echoPin4, maxDistance);

//front end wheels/servo
float initialAngle = 90.0; //initial angle of the direction the wheels face (straight ahead)
float minAngle = 69.0; //lowest angle we'll let the servo turn left (so we don't break the steering column)
float maxAngle = 111.0; //highest angle we'll let the servo turn right (so we don't break the steering column)
float frontMiddleDistance; //(current) distance from the object directly in front of the vehicle
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

//back camera
float lookLeftAngle = 25.0; //the angle the rear camera points when looking left (as driver looks over the left shoulder)
float lookRightAngle = 155.0; //the angle the rear camera points when looking right (as driver looks over the right shoulder)
float lookStraightBackAngle = 82.0; //the angle the rear camera points when it is look directly backwards
float reverseDistance = 0.0; //(current) distance from the wall behind the car while backing up (turning around)
float backLeftDistance = 0.0; //distance to object when looking back over left shoulder
float backRightDistance = 0.0; //distance to object when looking back over right shoulder

//dc motor - speeds
int carSpeeds[] = {32, 64, 96, 128, 192, 255}; // 1/8, 1/4, 1/3, 1/2, 3/4, 4/4

//global vars (both front and back sensors)
const float frontCollisionDistance = 1350.0;
const float rearCollisionDistance = 500.0;

void setup()
{
  //delay
  delay(3000);

  //servo - steering
  servoSteering.attach(servoPinSteering);
  
  //distance sensors (right and left)
  //need to add these to a state machine call
  rightDistance = getRightDistance();
  delay(250);
  leftDistance = getLeftDistance();

  //serial port
  //Serial.begin(9600);
  
  //delay
  delay(1000);

  //servo - reverse camera
  servoReverse.attach(servoPinReverse);
  lookLeft(); //look over left shoulder (rear camera/sonic sensor)
  lookRight(); //look over right shoulder (rear camera/sonic sensor)
  lookStraightBack(); //look straight back 
  delay(1000);
  
  //motor
  motor.setSpeed(carSpeeds[3]); 
  //motor.run(RELEASE);
  
  //run the car forward until something good or bad happens
  driveMotor();
}

//get the distance from the wall (Right side of car)
float getFrontDistance()
{
  frontMiddleDistance = float(hcsr04MiddleFront.ping_median(2));
  return frontMiddleDistance;
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
    delay(35); //let motor take time to get to the new position
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
    delay(35); //let motor take time to get to the new position
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
  delay(600);
  return true;
}

bool lookRight()
{
  servoReverse.write(lookRightAngle);
  delay(600);
  return true;
}

bool lookStraightBack()
{
  servoReverse.write(lookStraightBackAngle);
  delay(600);
  return true;
}

//check blind spots and then turn around
void turnAround()
{
  lookLeft();
  backLeftDistance = float(hcsr04MiddleRear.ping_median(2));
  
  lookRight();
  backRightDistance = float(hcsr04MiddleRear.ping_median(2));
  
  lookStraightBack();
  if(backLeftDistance > backRightDistance) //turn around clockwise (because you can backup to your left with more space)
  {
    lookLeft(); //look over left shoulder

    do { //turn wheels left
      //execution code is in while condition.  need to refactor
    } while (turnLeft());
    delay(500);
    
    reverseMotor(); //start backing up until collision detection
    do {
      backLeftDistance = getRearDistance();
    } while (backLeftDistance > rearCollisionDistance);
    stopMotor();
    delay(500);
    
    do { //turn wheels right
      //execution code is in condition.  need to refactor
    } while (turnRight()); //turn right
    delay(500);
    
    driveMotor(); //drive forward
    do {
      rightCurrentDistance = getRightDistance();
    } while (rightCurrentDistance > frontCollisionDistance);
    stopMotor();
    delay(500);

    do { //turn wheels left
      //execution code is in while condition.  need to refactor
    } while (turnLeft());
    delay(500);

    reverseMotor(); //start backing up until collision detection
    do { 
      backLeftDistance = getRearDistance();
    } while (backLeftDistance > rearCollisionDistance);
    stopMotor();
    delay(500);
  }
  
  else if(backRightDistance > backLeftDistance) //turn around counter clockwise
  {
    servoReverse.write(lookRightAngle); //look over right shoulder
    delay(500);

    do { //turn wheels right
      //execution code is in while condition.  need to refactor
    } while (turnRight());
    delay(500);
    
    reverseMotor(); //start backing up until collision detection
    do {
      //getRearDistance();
    } while (getRearDistance() > rearCollisionDistance);
    stopMotor();
    delay(500);
    
    do { //turn wheels left
      //execution code is in condition.  need to refactor
    } while (turnLeft()); //turn left
    delay(500);
    
    driveMotor(); //drive forward
    do {
      leftCurrentDistance = getLeftDistance();
    } while (leftCurrentDistance > frontCollisionDistance);
    stopMotor();
    delay(500);

    do { //turn wheels right
      //execution code is in while condition.  need to refactor
    } while (turnRight());
    delay(500);

    reverseMotor(); //start backing up until collision detection
    do {
      //getRearDistance();
    } while (getRearDistance() > rearCollisionDistance);
    stopMotor();
    delay(500);
  }
  else
  {
    //reverseMotor();
    //delay(2000);
  }
  servoReverse.write(82); //look straight back (rear camera/sonic sensor)
  delay(500);
  servoSteering.write(initialAngle);
  delay(500);
  driveMotor();
}

//drive straight
void driveStraight()
{
  currentAngle = servoSteering.read();
  servoSteering.write(initialAngle);
  delay(45); //let motor take time to get to the new position
}

void loop()
{
  //GET THIS DISTANCE FIRST (all 3 front sensors)
  if(getFrontDistance() < frontCollisionDistance)
  {
    //stop car and start turn-around process
    stopMotor();
    turnAround();
  }
  else
  {
    //driveMotor();
  }
  
  //need to add these to a state machine call
  rightCurrentDistance = getRightDistance(); //get the current distance (right side)
  leftCurrentDistance = getLeftDistance(); //get the current distance (left side)
  distanceDelta = abs(rightCurrentDistance - leftCurrentDistance); //absolute value of the difference between the left and right sides
  distanceTotal = rightCurrentDistance + leftCurrentDistance; //sum of the left and right distance
  
  rightAbsoluteValue = abs(rightCurrentDistance - rightDistance);
  leftAbsoluteValue = abs(leftCurrentDistance - leftDistance);
  if ((rightAbsoluteValue < distanceTolerance) && (leftAbsoluteValue < distanceTolerance))
  {
    //do nothing.  distance didn't change on both sides so DO NOT TURN THE STEERING WHEELS.  You're probably good.
  }
  else if(distanceDelta < 125.0)
  {
    //do nothing.  the difference in distance from walls on both sides is close.  Don't change it if it aint broke.
  }
  else
  {
    if (leftCurrentDistance > rightCurrentDistance)
    {
      turnLeft();
    }
    else
    {
      turnRight();
    }
    leftDistance = leftCurrentDistance;
    rightDistance = rightCurrentDistance;
  }
  delay(65);
}
