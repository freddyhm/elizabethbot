#include "Servo.h" 

#define SERIAL_PORT 9600 

#define PIN_SERVO      2

#define PIN_CAR_DIRECTION_RIGHT 3
#define PIN_CAR_DIRECTION_LEFT  4
#define PIN_CAR_MOTOR_PWM_RIGHT 5
#define PIN_CAR_MOTOR_PWM_LEFT  6
#define CAR_SPEED_RIGHT 100
#define CAR_SPEED_LEFT 100
#define CAR_TURN_SPEED 100

#define FRONT_ANGLE 45
#define LEFT_ANGLE 90
#define RIGHT_ANGLE 135

boolean moveCar = true;

#define PIN_SONIC_TRIG    7    
#define PIN_SONIC_ECHO    8  
#define MAX_SONIC_DISTANCE    300   
#define SONIC_TIMEOUT   (MAX_SONIC_DISTANCE * 60) 
#define SONIC_SOUND_VELOCITY  340      

#define DELAY_DURATION 1000
#define INIT_SERVO_ANGLE 45

Servo servo;             
byte servoOffset = 0;    // calibrate servo
u8 distance[4];
u8 currentAngle;
u8 distance_single;

void setup() {

  Serial.begin(SERIAL_PORT);

  setCarPins();
  setSonicSensorPins();
  setServo();
}

void setCarPins()
{
  pinMode(PIN_CAR_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_CAR_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_CAR_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_CAR_MOTOR_PWM_RIGHT, OUTPUT);
}

void setSonicSensorPins()
{
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT); 
}

void setServo()
{

  servo.attach(PIN_SERVO);

  int initialAngle = 90 + servoOffset;
  servo.write(initialAngle);  
}

void loop() 
{
  checkSonarAngleForObjects(FRONT_ANGLE);
  checkSonarAngleForObjects(LEFT_ANGLE);
  checkSonarAngleForObjects(RIGHT_ANGLE));
}

void checkSonarAngleForObjects(u8 angle)
{
  changeSonarAngle(angle);

  if (isObjectDetected())
  {
    if (angle == FRONT_ANGLE)
    {
      turnCarRight();
      print("Car has moved right with angle: ", angle);
    } 
    else if (currentAngle == RIGHT_ANGLE || currentAngle == LEFT_ANGLE)
    {
      turnCarLeft();
      print("Car has moved left with angle: ", angle);
    } 
  }
  else
  {
    print("Car has moved forward with angle: ", angle);
    moveCarForward();
  }
}

void turnCarRight()
{
  motorRun(CAR_TURN_SPEED, -CAR_TURN_SPEED);
}

void turnCarLeft()
{
  motorRun(-CAR_TURN_SPEED, CAR_TURN_SPEED);
}

void moveCarForward()
{
  motorRun(CAR_SPEED_LEFT, CAR_SPEED_RIGHT);
}

void changeSonarAngle(u8 newAngle)
{
  servo.write(newAngle);
  delay(DELAY_DURATION);
}

void printSonarValue(const char* direction, u8 distance)
{
  print(direction, distance);
}

void printCurrentAngle()
{
  currentAngle = servo.read();
  print("Current Angle:", servo.read());
}

bool isObjectDetected()
{
  distance_single = getSonar();
  
  if(distance_single < 40)
  {
    print("Object detected: ", distance_single);

    return true;
  }

  return false;
}

void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;

  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }

  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }
   
  digitalWrite(PIN_CAR_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_CAR_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_CAR_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_CAR_MOTOR_PWM_RIGHT, speedr);
}

float getSonar() {
  
  unsigned long pingTime;

  float distance;
  
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(100);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time

  if (pingTime != 0)
    distance = (float)pingTime * SONIC_SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_SONIC_DISTANCE;
  
  return distance; // return the distance value
}

void print(const char* parameter,int value)
{
  Serial.print(parameter);
  Serial.print(value);
  Serial.print("\n");
}
