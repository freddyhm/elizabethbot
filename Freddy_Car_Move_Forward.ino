/**********************************************************************
  Product     : Freenove 4WD Car for UNO
  Auther      : www.freenove.com
  Modification: 2019/08/03
**********************************************************************/

#include "Servo.h" 

#define PIN_SERVO      2       //define servo pin

#define PIN_DIRECTION_RIGHT 3
#define PIN_DIRECTION_LEFT  4
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_MOTOR_PWM_LEFT  6
#define SPEED_RIGHT 100
#define SPEED_LEFT 100
#define TURN_SPEED 100

boolean moveCar = true;

// sonic
#define PIN_SONIC_TRIG    7    //define Trig pin
#define PIN_SONIC_ECHO    8    //define Echo pin
#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60) // calculate timeout 
#define SOUND_VELOCITY  340  //soundVelocity: 340m/s
#define NO_OBJECT_NEARBY        

Servo servo;             //create servo object
byte servoOffset = 0;    //change the value to Calibrate servo
u8 distance[4];          //define an arry with type u8(same to unsigned char)
u8 currentAngle;

void setup() {

  // car
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  // sonic
  Serial.begin(9600);
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode

  // servo
  servo.attach(PIN_SERVO);        //initialize servo 
  servo.write(90 + servoOffset);  // change servoOffset to Calibrate servo
}

void loop() {

  servo.write(45);
  delay(1000);
  
  currentAngle = servo.read();

  Serial.print("Current Angle:");
  Serial.print(servo.read());
  Serial.print("\n");

  if (currentAngle == 45)
  {
    distance[0] = getSonar();   
    detectObject(distance[0], 'r');
  
    Serial.print("R:");

    Serial.print(distance[0], 'r');
    Serial.print('\n');  
  }

  servo.write(90);
  delay(1000);

  currentAngle = servo.read();

  Serial.print("Current Angle:");
  Serial.print(servo.read());
  Serial.print("\n");

  if (currentAngle == 90)
  {
    distance[1] = getSonar();
    detectObject(distance[1], 'm');
  
    Serial.print("M:");
    Serial.print(distance[1]);
    Serial.print('\n');   
  }

  servo.write(135);
  delay(1000);

  currentAngle = servo.read();

  Serial.print("Current Angle:");
  Serial.print(servo.read());
  Serial.print("\n");

  if (currentAngle == 135)
  {
    distance[2] = getSonar();
    detectObject(distance[2], 'l');
  
    Serial.print("L:");
    Serial.print(distance[2]);
    Serial.print('\n');   
  }

  servo.write(90);
  delay(1000);

  currentAngle = servo.read();
 
  Serial.print("Current Angle:");
  Serial.print(servo.read());
  Serial.print("\n");

  if (currentAngle == 90)
  {
    distance[3] = getSonar();
    detectObject(distance[3], 'm'); 
  
    Serial.print("M2:");
    Serial.print(distance[3]);
    Serial.print('\n');   
  }
}

void detectObject(u8 distance, char direction) {
  if(distance < 40 )
  {

    Serial.print("object detected in direction:");
    Serial.print(direction);

    Serial.print('\n');

    motorRun(0, 0);

    delay(1000);

    if (direction == 'l' || direction == 'm')
    {
      motorRun(TURN_SPEED, -TURN_SPEED);

    } else if (direction == 'r')
    {
      motorRun(-TURN_SPEED, TURN_SPEED);
    }

    delay(1000);
    

    //motorRun(SPEED_LEFT, SPEED_RIGHT);
  }
  else
  {
    motorRun(SPEED_LEFT, SPEED_RIGHT);
    //motorRun(TURN_SPEED, -TURN_SPEED);
  }
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
  
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(100);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}
