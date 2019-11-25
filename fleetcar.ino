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
#define pinLeftLeft 40 //front left IR sensor
#define pinLeftCentre 42 //front centre IR sensor
#define pinRightCentre 44 //front right centre IR sensor
#define pinRightRight 46 //right right IR sensor
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
int isObjectThere = 1;

//back camera
float lookLeftAngle = 25.0; //the angle the rear camera points when looking left (as driver looks over the left shoulder)
float lookRightAngle = 155.0; //the angle the rear camera points when looking right (as driver looks over the right shoulder)
float lookStraightBackAngle = 82.0; //the angle the rear camera points when it is look directly backwards
float reverseDistance = 0.0; //(current) distance from the wall behind the car while backing up (turning around)
float backLeftDistance = 0.0; //distance to object when looking back over left shoulder
float backRightDistance = 0.0; //distance to object when looking back over right shoulder

//dc motor - speeds
int carSpeeds[16] = {16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255};
int carGearFwd = 10; 
int carGearRev = 10;
int carGear = 10;
int gearCounter = 0;
int iRList[4] = {1, 1, 1, 1}; //HIGH = no object, LOW = object 1 = HIGH, 0 = LOW

//global vars (both front and back sensors)
const float rearCollisionDistance = 900.0;
const float frontCollisionDistance = 300.0;

void setup()
{
  Serial.begin(9600);
  Serial.print("MTO AI Fleet Car");
  Serial.println();
  
  //delay
  delay(3000);

  //obstacle detection IR
  pinMode(pinLeftLeft, INPUT);
  pinMode(pinLeftCentre, INPUT);
  pinMode(pinRightCentre, INPUT);
  pinMode(pinRightRight, INPUT);
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

//checks to see if there is an object in front of the IR sensors
//HIGH - means no obstacle in view //LOW - means there is an obstacle in view. HIGH == 1, LOW == 0
int getIsObstacleThere()
{
  isObjectThere = 1;
  iRList[0] = digitalRead(pinLeftLeft);
  iRList[1] = digitalRead(pinLeftCentre);
  iRList[2] = digitalRead(pinRightCentre);
  iRList[3] = digitalRead(pinRightRight);
  iRList[4] = digitalRead(pinRearLeft);
  for (int counter = 0; counter < 5; counter++)
  {
    //Serial.print(iRList[counter]);
    //Serial.println();
    if(iRList[counter] == 0)
    {
      isObjectThere = 0;
    }
  }
  return isObjectThere;
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
  motor.setSpeed(carSpeeds[carGearRev]);
  
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
    } while (getIsObstacleThere() == 1);
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
    } while (getIsObstacleThere() == 1);
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

//gear down
void gearDown()
{
  if(carGearFwd > 8)
  {
    carGearFwd = carGearFwd-1;
    motor.setSpeed(carSpeeds[carGearFwd]);
    delay(15);
  }
}

void loop()
{
  if(gearCounter == 5)
  {
    gearDown();
    gearCounter = 0;
  }
  //need to add these to a state machine call
  rightCurrentDistance = getRightDistance(); //get the current distance (right side)
  leftCurrentDistance = getLeftDistance(); //get the current distance (left side)
  distanceDelta = abs(rightCurrentDistance - leftCurrentDistance); //absolute value of the difference between the left and right sides
  //check IR sensors for an object
  if (getIsObstacleThere() == 0 ) //object is in front of car
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
    delay(15);
  }
  gearCounter = gearCounter+1;
}
