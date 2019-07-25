/*
  FLEET CAR (TBD)
  POC
*/

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

#define maxDistance 200
//Right
#define echoPin1 30 // microphone (listening for ping)
#define trigPin1 31 // speaker (making ping sound)
//Left
#define echoPin2 32 // microphone (listening for ping)
#define trigPin2 33 // speaker (making ping sound)
//Middle Front
#define echoPin3 34 // microphone (listening for ping)
#define trigPin3 35 // speaker (making ping sound)
//Turning Servo
#define servoPinSteering 10 // servo control pin
//Reverse Servo
#define servoPinReverse 9 // reverse camera servo

AF_DCMotor motor(1);
Servo servoSteering;
Servo servoReverse;
NewPing hcsr04Right(trigPin1, echoPin1, maxDistance);
NewPing hcsr04Left(trigPin2, echoPin2, maxDistance);
NewPing hcsr04MiddleFront(trigPin3, echoPin3, 3000);
float initialAngle = 90.0; // initial angle of the direction the wheels face (straight ahead)
float minAngle = 65.0; // lowest angle we'll let the servo turn left (so we don't break the steering column)
float maxAngle = 115.0; // highest angle we'll let the servo turn right (so we don't break the steering column)
float frontMiddleDistance; // (current) distance from the object directly in front of the vehicle
float rightDistance = 0.0; // (previous) distance from wall/object to front right wheel
float leftDistance = 0.0; // (previous) distance from wall/object to front left wheel
float rightCurrentDistance = 0.0; // (current) distance from wall/object to front left wheel
float leftCurrentDistance = 0.0; // (current) distance from wall/object to front left wheel
float currentAngle = 0.0; // current angle (when being checked before it is probably moved)
float backLeftDistance = 0.0; // distance to object when looking back over left shoulder
float backRightDistance = 0.0; // distance to object when looking back over right shoulder
const float turnIncrement = 5.0; // the angle amount the steering wheel will turn left or right
const float deltaThreshold = 60.0; // the distance to set the signicance or willingness to turn.  How much more or less one side has other the other.  This can be changed to increase performance - CAREFUL!
float distanceTotal = 0.0; // total distance (sum of right and left distances)
float distanceTolerance = 15.0; // the distance to account for noise (ping echo distance).  Delta between distance and current distance is valid or an error. (the lower the number, more aggressive steering)
float distanceDelta = 0.0; // The difference between the distances of both sides (i.e.  left tire is 90 and right tire is 110.  the delta is the abs(20) |20|.
int rightAbsoluteValue = 0;
int leftAbsoluteValue = 0;

void setup()
{
  //delay
  delay(5000);

  //servo - steering
  servoSteering.attach(servoPinSteering);
  delay(500);
  servoSteering.write(initialAngle);
  delay(500);

  //distance sensors (right and left)
  //need to add these to a state machine call
  rightDistance = getRightDistance();
  leftDistance = getLeftDistance();

  //serial port
  Serial.begin(9600);
  //Serial.println(rightDistance);
  //Serial.println(leftDistance);
  
  //delay
  delay(2000);

  //servo - reverse camera
  servoReverse.attach(servoPinReverse);
  delay(500);
  servoReverse.write(30);
  delay(500);
  servoReverse.write(150);
  delay(500);
  servoReverse.write(85);

  //delay
  delay(1000);
  
  //motor
  motor.setSpeed(90);
  motor.run(RELEASE);
  motor.run(FORWARD); // Run the car forward until something good or bad happens

  // delay
  delay(1000);
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
  //String text = "right distance ";
  //Serial.println(text + rightDistance);
  return rightCurrentDistance;
}

//get the distance from the wall (Left side of car)
float getLeftDistance()
{
  leftCurrentDistance = float(hcsr04Left.ping_median(2));
  //String text = "left distance ";
  //Serial.println(text + leftDistance);
  return leftCurrentDistance;
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

// stop the motor
void stopMotor()
{
  motor.run(RELEASE);
}

// turn right
void turnRight()
{
  currentAngle = servoSteering.read();
  float moveAngle = (currentAngle - turnIncrement);
  //Serial.println(currentAngle);
  //Serial.println(moveAngle);
  if (moveAngle > 69)
  {
    servoSteering.write(moveAngle);
    delay(35); //let motor take time to get to the new position
  }
}

//turn left
void turnLeft()
{
  currentAngle = servoSteering.read();
  float moveAngle = (currentAngle + turnIncrement);
  //Serial.println(currentAngle);
  //Serial.println(moveAngle);
  if (moveAngle < 111)
  {
    servoSteering.write(moveAngle);
    delay(35); //let motor take time to get to the new position
  }
}

//check blind spots
void turnAround()
{
  delay(500);
  servoReverse.write(20);
  backLeftDistance = float(hcsr04Left.ping_median(2));
  delay(500);
  servoReverse.write(160);
  backRightDistance = float(hcsr04Left.ping_median(2));
  delay(500);
  servoReverse.write(85);
  delay(500);
  if(backLeftDistance > backRightDistance)
  {
    turnLeft();   
  }
  else
  {
    turnRight();
  }
}

//drive straight
void driveStraight()
{
  currentAngle = servoSteering.read();
  servoSteering.write(initialAngle);
  delay(45); // let motor take time to get to the new position
}

void loop()
{
  //GET THIS DISTANCE FIRST
  if(getFrontDistance() < 1000)
  {
    //start turn-around process
    stopMotor();
    turnAround();
  }
  else
  {
    driveMotor();
  }
  
  //need to add these to a state machine call
  rightCurrentDistance = getRightDistance(); // get the current distance (right side)
  //Serial.println(rightCurrentDistance);
  leftCurrentDistance = getLeftDistance(); // get the current distance (left side)
  //Serial.println(leftCurrentDistance);
  distanceDelta = abs(rightCurrentDistance - leftCurrentDistance); // absolute value of the difference between the left and right sides
  distanceTotal = rightCurrentDistance + leftCurrentDistance; // sum of the left and right distance
  
  //if (distanceTotal < 1600) // get rid of noise.  large distances when the agent is not on a track (how far we look out to the sides)
  rightAbsoluteValue = abs(rightCurrentDistance - rightDistance);
  leftAbsoluteValue = abs(leftCurrentDistance - leftDistance);
  Serial.println(rightAbsoluteValue);
  Serial.println(leftAbsoluteValue);
  if ((rightAbsoluteValue < distanceTolerance) && (leftAbsoluteValue < distanceTolerance))
  {
    //do nothing.  distance didn't change so DO NOT TURN THE STEERING WHEELS.  You're probably good.
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