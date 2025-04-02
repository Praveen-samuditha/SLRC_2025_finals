#include <Arduino.h>
#include <L298N.h>
#include <Servo.h>
#include <string.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <math.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// A====> left
// B====> Right

#define PWMA 4
#define PWMB 5
#define AIN1 22
#define AIN2 23
#define BIN1 24
#define BIN2 25

// TCS3200 (HW-067) required pin definitions
#define S0 27  // Frequency scaling
#define S2 26  // Filter select
#define S3 29  // Filter select
#define OUT 28 // Frequency output
#define YELLOW_LED 30  // Yellow LED indicator
#define WHITE_LED 31   // White LED indicator

unsigned long redMinWhite, redMaxWhite, greenMinWhite, greenMaxWhite, blueMinWhite, blueMaxWhite;
unsigned long redMinYellow, redMaxYellow, greenMinYellow, greenMaxYellow, blueMinYellow, blueMaxYellow;

unsigned long redFreq, greenFreq, blueFreq;

// Create servo objects for each joint
Servo baseServo;      // Base rotation (0-360° capable)
Servo shoulderServo;  // Shoulder joint (0-180°)
Servo elbowServo;    // Elbow joint (0-180°)
Servo gripperServo;  // Gripper (0-180°)
Servo rightServo;
Servo leftServo;


// Pin definitions
const int BASE_PIN = 7;
const int SHOULDER_PIN = 11;
const int ELBOW_PIN = 8;
const int GRIPPER_PIN = 9;

const int right_pin = 10;
const int left_pin = 6;

int ini_right = 0 ;
int ini_left = 180 ;





// Variables to store servo positions
int basePos = 120;     // Initial position for base (0-360°)
int shoulderPos = 0;  // Initial position for shoulder (0-180°)
int elbowPos = 180;   // Initial position for elbow (0-180°)
int gripperPos = 0;   // Gripper fully closed initially (0-180°)
int ball_count=0;

// for IR_array
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A7, A6, A5, A4, A3, A2, A1, A0}; // Analog pins for the remaining sensors

int IR_weight[SensorCount] = {-50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];
int sensorValues_w[SensorCount];

// PID control parameters
float Kp = 5; // Proportional term
float Ki = 0; // Integral term
float Kd = 5; // Derivative term
float sum;
int increment_count = 1;

long ldistance;
long rdistance;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 200;

String junction = "a";

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft();
void turnRight();
void noLine();
String colordetect();
void Task1();
void moveStraightPID();
void moveTurnRightPID();
void moveTurnLeftPID();
void againcheck(int *sensorValues, int count);
void backward_B1();
void backward_B2();
void backward_B3();
void small_backward();
void moveForward(int *sensorValues, int count);
void encoderISR_A();
void encoderISR_B();
void colour_line_follow();
void line_follow(int *sensorValues);
bool isImmediateTurnL(int *sensorValues);
bool isImmediateTurnR(int *sensorValues);
bool isImmediateTurnT(int *sensorValues);
void encoder_backward();
int Green_White_Detect();
void colomcheck();
void move_for_grabbing(int count);
//void turnRight45();
//void turnLeft45();
void turnRightStart();
void half_rotation();
void PID_Linefollow(float pidValue);
void smoothMoveServo(Servo &servo, int &currentPos, int targetPos);
String ball_colour();
void calibrateColors();
void readColors();
void turnLeftNew();

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

#define ENCODER_A1 2 // Encoder A channel (Interrupt pin)
#define ENCODER_B1 3 // Encoder B channel (Interrupt pin)

// Encoder counts
volatile long encoderCountA = 0; // Encoder count for Motor A
volatile long encoderCountB = 0; // Encoder count for Motor B
const long targetCounts = 190; // Desired encoder counts for 90 degree turns
const long targetCounts_rotation = 360; // Desired encoder counts for 180 degree turns
const int count_moveForward = 85;
const int count_moveForward_w = 125;

// PID parameters for encoders
float errorenco = 0;
float previousErrorenco = 0;
float integralenco = 0;
float derivativeenco = 0;

// Target speed for motors
int baseSpeed = 200; // Base speed for both motors (0 to 255)///////// for Line Navigation 115

int flag = 0;
int plus = 0;
bool is_plus=false;


int green_detect = 1; // if 1 --> green, 0 --> white     ********************************************************
int green_temp_array[5]= {2,0,1,1,2};
char temp_yellow_white[5]={'W','Y','W','Y','Y'};
int tem_pos=0;

bool check_again = false;
int Tdetect=0;
int endline=0;




int Array_size;

// for Line Navigation
int No_lines = 0;
int modulus;
bool Code[20] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int first_black_line = 0;

void setup() {
  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(53, OUTPUT); // led indicate in virtual box

  // Encoder pins setup
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);

  // give pinmode to defined leds

  // give initial values to servo motors

  // Attach servos to their respective pins
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  leftServo.attach(left_pin);
  rightServo.attach(right_pin);



  pinMode(13,OUTPUT);

  digitalWrite(13,LOW);

  // if (tcs.begin()==false){
  //   digitalWrite(13,HIGH);

  // }    

  pinMode(S0, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Set pin modes for LEDs
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);

  // Initial state: LEDs off
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(WHITE_LED, LOW);

  digitalWrite(S0, HIGH); // LOW to HIGHT

  Serial.begin(9600);

  

  // Set initial positions
  baseServo.write(basePos);         // Base: 0-360°
  shoulderServo.write(shoulderPos); // Others: 0-180°
  elbowServo.write(elbowPos);
  gripperServo.write(gripperPos);

  leftServo.write(ini_left);
  rightServo.write(ini_right);

  calibrateColors();

}

void loop() {
  // Call the PID function to keep the robot moving straight
  //moveStraightPID();

  // Add a small delay to control the PID loop frequency
  //delay(10);
  //colour_line_follow();
  Task1();
  //move_for_grabbing(100);
}

void calibrateColors() {
  // Calibrate White
  Serial.println("Place a WHITE object in front of the sensor and press any key...");
  digitalWrite(WHITE_LED, HIGH);  // Indicate white calibration
  digitalWrite(YELLOW_LED, LOW);

  delay(5000);
  Serial.read();  // Clear buffer
  digitalWrite(WHITE_LED, LOW);  // Turn off LED during reading

  readColors();
  redMinWhite = redFreq - 10; redMaxWhite = redFreq + 10;
  greenMinWhite = greenFreq - 10; greenMaxWhite = greenFreq + 10;
  blueMinWhite = blueFreq - 10; blueMaxWhite = blueFreq + 10;

  Serial.print("White Calibrated - Red: ");
  Serial.print(redFreq);
  Serial.print(" Green: ");
  Serial.print(greenFreq);
  Serial.print(" Blue: ");
  Serial.println(blueFreq);

  delay(1000);  // Brief pause

  // Calibrate Yellow
  Serial.println("Place a YELLOW object in front of the sensor and press any key...");
  digitalWrite(YELLOW_LED, HIGH);  // Indicate yellow calibration
  digitalWrite(WHITE_LED, LOW);

  delay(5000);

  Serial.read();  // Clear buffer
  digitalWrite(YELLOW_LED, LOW);  // Turn off LED during reading

  readColors();
  redMinYellow = redFreq - 10; redMaxYellow = redFreq + 10;
  greenMinYellow = greenFreq - 10; greenMaxYellow = greenFreq + 10;
  blueMinYellow = blueFreq - 10; blueMaxYellow = blueFreq + 10;

  Serial.print("Yellow Calibrated - Red: ");
  Serial.print(redFreq);
  Serial.print(" Green: ");
  Serial.print(greenFreq);
  Serial.print(" Blue: ");
  Serial.println(blueFreq);
}






void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 80 ? 0 : 1; // Assuming higher values indicate no line (sensor readings > 135) black ====> return 0 (no line)
  }
}

void line_follow(int *sensorValues) {
  if (isImmediateTurnL(sensorValues)) {
    motor1.stop();
    motor2.stop();
    delay(5);
    againcheck(sensorValues, count_moveForward);
  } else if (isImmediateTurnR(sensorValues)) {
    motor1.stop();
    motor2.stop();
    delay(5);
    againcheck(sensorValues, count_moveForward);
  } else if (isImmediateTurnT(sensorValues)) {
    motor1.stop();
    motor2.stop();
    delay(5);
    againcheck(sensorValues, count_moveForward);
  } else {
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  // Loop through all sensors
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) { // Assuming 1 indicates line detected
      position += IR_weight[i];
      onLine++;
    }
  }

  // If no line is detected by any sensor, use the last known error value
  if (onLine == 0) {
    error = -previousError;
  } else {
    position /= onLine;
    error = 0 - position;
  }
  for (int i = 49; i > 0; i--) {
    errorArray[i] = errorArray[i - 1];
  }
  errorArray[0] = error;
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);

  return pidValue;
}

void motor_drive(float left, float right) {
    int absRight = abs(right); // Absolute value for right speed
    int absLeft = abs(left); // Absolute value for left speed
  
    if (left > 0) {
      motor2.setSpeed(absLeft);
      motor2.forward();
    } else {
      motor2.setSpeed(absLeft);
      motor2.backward();
    }
  
    if (right > 0) {
      motor1.setSpeed(absRight);
      motor1.forward();
    } else {
      motor1.setSpeed(absRight);
      motor1.backward();
    }
  }

void PID_Linefollow(float pidValue) {
  lsp = lfspeed - pidValue;
  rsp = lfspeed + pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = 255;
  }
  motor_drive(lsp, rsp);
}



bool isImmediateTurnL(int *sensorValues) {
  bool turnLeft = (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) || (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1);
  return turnLeft;
}

bool isImmediateTurnR(int *sensorValues) {
  bool turnRight = (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 0) || (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0);
  return turnRight;
}

bool isImmediateTurnT(int *sensorValues) {
  bool turnT = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return turnT;
}

void againcheck(int *sensorValues, int count) {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 15 && encoderCountB < 15) {
    moveStraightPID();
  }
  motor1.stop();
  motor2.stop();
  delay(5);
  readSensors(sensorValues);
  if (isImmediateTurnL(sensorValues)) {
    junction = "L";
    motor1.stop();
    motor2.stop();
    delay(5);
    moveForward(sensorValues, count);
  } else if (isImmediateTurnR(sensorValues)) {
    junction = "R";
    motor1.stop();
    motor2.stop();
    delay(5);
    moveForward(sensorValues, count_moveForward);
  } else if (isImmediateTurnT(sensorValues)) {
    junction = "T";
    motor1.stop();
    motor2.stop();
    delay(5);
    moveForward(sensorValues, count_moveForward);
  } else {
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
}

void backward_B1() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 450 && encoderCountB < 450) {
    encoder_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void backward_B2() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 765 && encoderCountB < 765) {
    encoder_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void small_backward() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 170 && encoderCountB < 170) {
    encoder_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void backward_B3() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 1090 && encoderCountB < 1090) {
    encoder_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void moveForward(int *sensorValues, int count) {
  readSensors(sensorValues);
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < count && encoderCountB < count) {
    moveStraightPID();
  }
  motor1.stop();
  motor2.stop();
  delay(50);
  readSensors(sensorValues);

  if (junction == "L") {
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2) {
      junction = "LL";
    } else if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6) {
      junction = "TB";
    } else {
      junction = "LT";
    }
  }
  if (junction == "R") {
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2) {
      junction = "RR";
    } else if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6) {
      junction = "TB";
    } else {
      junction = "RT";
    }
  }
  if (junction == "T") {
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2) {
      junction = "TT";
    } else if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6) {
      junction = "TB";
    } else {
      junction = "TP";
    }
  }
}

void turnRight() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < targetCounts && encoderCountB < targetCounts) {
    moveTurnRightPID();
  }
  motor1.stop();
  motor2.stop();
}

void turnLeft() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountB < targetCounts && encoderCountA < targetCounts) {
    moveTurnLeftPID();
  }
  motor1.stop();
  motor2.stop();
}

void turnLeftNew() {
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountB < 185 && encoderCountA < 185) {
    moveTurnLeftPID();
  }
  motor1.stop();
  motor2.stop();
}


// void turnLeft45() {
//   encoderCountA = 0;
//   encoderCountB = 0;

//   while (encoderCountB < 95 && encoderCountA < 95) {
//     moveTurnLeftPID();
//   }
//   motor1.stop();
//   motor2.stop();
// }

// void turnRight45() {
//   encoderCountA = 0;
//   encoderCountB = 0;

//   while (encoderCountB < 95 && encoderCountA < 95) {
//     moveTurnRightPID();
//   }
//   motor1.stop();
//   motor2.stop();
// }

void turnRightStart() {
  turnRight();

  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 150 && encoderCountB < 150) {
    moveStraightPID();
  }
  motor1.stop();
  motor2.stop();
  delay(5);

  turnLeft();
}

void encoderISR_A() {
  encoderCountA++;
}

void encoderISR_B() {
  encoderCountB++;
}

void colour_line_follow() {
  readSensors(sensorValues);
  line_follow(sensorValues);

  if (junction == "LL") {
    turnLeft();
  } else if (junction == "LT") {
    line_follow(sensorValues);
  } else if (junction == "RR") {
    turnRight();
  } else if (junction == "RT") {
    line_follow(sensorValues);
  } else if (junction == "TP") {
    is_plus = true;
    //delay(1000);
    //green_detect = Green_White_Detect();      *******************************************************
    //Serial.println(plus);


    //Serial.println(green_detect);
  } else if (junction == "TT") {
      Tdetect = 1;
  } else if (junction == "TB") {
    motor1.stop();
    motor2.stop();
    //delay(3000);
    increment_count++;
  }
  junction = "a";
}

void moveStraightPID() {
  errorenco = encoderCountA - encoderCountB;

  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  int motor1Speed = baseSpeed - correction;
  int motor2Speed = baseSpeed + correction;

  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  motor1.setSpeed(motor1Speed);
  motor1.forward();
  motor2.setSpeed(motor2Speed);
  motor2.forward();

  previousErrorenco = errorenco;
}

// D53-blue , D51
String colordetect(){
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (b > r){
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    return "Blue" ;
  } 
  if (b < r){
    //digitalWrite(51,HIGH);
    //digitalWrite(53,LOW);
    return "Red";
  }
  else{
    return "non";
  }
}
String Blue_Red_NOTdetect(){
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("red:");
  Serial.print(r);
  Serial.print(", blue:");
  Serial.println(b);

  
  if ( r> 150 || 45 < b){ //(26<r && 26<b && r<200 && b<200){
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    if (r>b){
      Serial.print("red");
      return "r";
    }
    else{
      Serial.print("blue");
      return "b";
    }

  }
  else{
    return "non";
  }


}



void colordetect1(){
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (b > r){
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    //Serial.print("Red    :");     //32---- 37
    //Serial.print(r);
    //Serial.print("   Blue    :");    //59--- 64
    //Serial.println(b);
  } 
  if (b < r){
    //digitalWrite(51,HIGH);
    //digitalWrite(53,LOW);
    //Serial.print("Red    :");     //102
    //Serial.print(r);
    //Serial.print("   Blue    :");   //50
    //Serial.println(b);
  }
}
// Black-----> r=20   b=18
// White-----> r=310  b=360


// const int size_1b=2;
// const String Turn_1b[size_1b]={"TP","TT"};
// const String Actions_1b[size_1b]={"R","B"};
// const String LED_1b[size_1b]={"N","S"};


// const int size_1r=10;
// const String Turn_1r[size_1r]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
// const String Actions_1r[size_1r]={"180","L","L","F","F","180","R","R","R","B"};
// const String LED_1r[size_1r]={"N","N","N","S","N","O","N","N","N","S"};

/*
void Identify_action(String Action){

  if (Action=="180"){half_rotation();}

  if (Action=="L"){turnLeft();}

  if (Action=="R"){turnRight();}

  if (Action=="B"){
    backward_B1();
    while(Blue_Red_NOTdetect()){
      //colordetect1();
      encoder_backward();
    }
    motor1.stop();
    motor2.stop();
    delay(3000);
    increment_count++;
    digitalWrite(53,LOW);
    half_rotation();
    moveForward(sensorValues,count_moveForward);
    
  }
    if (Action=="FF"){
    while(Blue_Red_NOTdetect()){
      //colordetect1();
      moveStraightPID();
    }
    motor1.stop();
    motor2.stop();
    delay(3000);
    digitalWrite(53,LOW);
    increment_count++;

    
  }


  if (Action=="S"){
    motor1.stop();
    motor2.stop();
    delay(3000);
    }

  if (Action=="F"){line_follow(sensorValues);}

  if (Action=="B1"){backward_B1();}

  if (Action=="B2"){backward_B2();}

  if (Action=="B3"){backward_B3();}


}
*/
void half_rotation() {
  // Perform a sharp left turn by 
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while ((encoderCountB < targetCounts_rotation) && (encoderCountA < targetCounts_rotation)) {
    //Serial.println(encoderCountA,encoderCountB);

    moveTurnRightPID();
  
    // motor2.setSpeed(120);
    // motor2.forward();
    // motor1.setSpeed(120);
    // motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB);
}

void half_rotation_L() {
  // Perform a sharp left turn by 
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while ((encoderCountB < targetCounts_rotation) && (encoderCountA < targetCounts_rotation)) {
    //Serial.println(encoderCountA,encoderCountB);

    moveTurnLeftPID();
  
    // motor2.setSpeed(120);
    // motor2.forward();
    // motor1.setSpeed(120);
    // motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB);
}
/*
bool Identify_Strip(int* sensorValues){
  readSensors(sensorValues);
  if ((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7])>6){
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 1;     
    
  }else{
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 0;
  }
}

*/




// Robo should go backward until the box should increment the increment count and go to next task
void encoder_backward(){

    // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.backward();
  motor2.setSpeed(motor2Speed);
  motor2.backward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}     

//////////////////

/*
bool Wall_detect(){
  digitalWrite(trig1, LOW);

  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  
  long t1 = pulseIn(echo1, HIGH);
  
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);

  
  long t2 = pulseIn(echo2, HIGH);

  long cm1 = t1/29/2;
  long cm2 = t2/29/2;

  //Serial.print(cm1);
  //Serial.print("\t");
  //Serial.print(cm2);
  //Serial.println();

  // if((2<cm1) && (cm1<70) || (2<cm2) && (cm2<70)){
  //     digitalWrite(49,HIGH);
  //     delay(1000);
  //     digitalWrite(49,LOW);
  // }

  //delay(100);
  return ((2<cm1) && (cm1<90) || (2<cm2) && (cm2<90) );
}

*/







void LED_Switching_Juc(String LEDstate){
  // S---> Switch On
  // O---> Switch Off
  // N---> Nothing
  if (LEDstate=="S"){
    // Turn on LED
    //digitalWrite(53,HIGH);
  }
  if (LEDstate=="O"){
    // Turn Off LED
    //digitalWrite(53,LOW);
  }
}

// const int size_1b=2;
// const String Turn_1b[size_1b]={"TP","TT"};
// const String Actions_1b[size_1b]={"R","B"};
// const String LED_1b[size_1b]={"N","S"};


// const int size_1r=10;
// const String Turn_1r[size_1r]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
// const String Actions_1r[size_1r]={"180","L","L","F","F","180","R","R","R","B"};
// const String LED_1r[size_1r]={"N","N","N","S","N","O","N","N","N","S"};
/*
void find_path(String Turn[], String Actions[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        Identify_action(Actions[i]);
        junction="a";
        break;
      }
      
    }
  }
}



void find_path_full_array(String Turn[], String Actions[],String LED[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        LED_Switching_Juc(LED[i]);
        Identify_action(Actions[i]);
        junction="a";
        break;
      }
      
    }
  }
}
*/
// for check Blue wall open or not
//bool Check_Blue_Wall(int )        <======== have to find out how Ultra sonic works

String Colour;

String Arranging_order(String Colour) {
  if (Colour == "Blue") {
      return "Ascending";
  } else if (Colour == "Red") {
      return "Descending";
  } else {
      return "Unknown";
  }
}

//  Box_Detection function should feed Ultra Sound input


/*
bool Box_Detection(){

  return 0;

}


void Box_lift(){}

void Box_drop(){}


void Box_Arrange(){
  Colour="Blue";
  if (Arranging_order(Colour)="Ascending"){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);

      if (junction=="TP"){
        turnLeft();
      }
      
      junction="a";

      if (Box_Detection()){
        motor1.stop();
        motor2.stop();
        if  (Hight_Measure()==1){
          delay(7000);
          Box_lift();
          half_rotation();


          // box=3, location=A
          const String Turn_3A[3]={"TP","TP","TP"};
          const String Action_3A[3]={"L","F","R"};

          //find_path_Box_Arrangement(Turn_3A,Action_3A);
          Box_drop();

          while(true){
            readSensors(sensorValues);
            line_follow(sensorValues);

            if  (junction=="TP"){
              turnLeft();

              while(true){
                readSensors(sensorValues);
                line_follow(sensorValues);

                if  (junction=="TP"){
                  turnRight();

                }

                if (Box_Detection()){
                  motor1.stop();
                  motor2.stop();
                  if  (Hight_Measure()==1){
                    delay(7000);
                    Box_lift();
                    half_rotation();

                    //const String Turn_1B[2]={"TP","TP"};
                    //const String Turn_1B[2]={"R","L"};

                    //find_path_Box_Arrangement(Turn_1B,Action_1B);
                    Box_drop();
                    while (true){
                      readSensors(sensorValues);
                      line_follow(sensorValues);
                      
                      junction="a";
                    }
          
        
                  }

                }
              junction="a";
              }
            }

            junction="a";

          }
        }

        
        

      }


    
  }

  }
  if (Arranging_order(Colour)="Descending"){


  }
}

void Mech_Arm(String Drop){

}

int Hight_Measure(){
  // there are 3 different angles for hight measurements 
  bool range1;//1
  bool range2;//2
  bool range3;//3
  if (range1){
    return 1;
  }
    if (range2){
    return 2;
  }
    if (range3){
    return 3;
  }
}


//String Box_Arrangement_Turns[];


void find_path_Box_Arrangement(String Turn[], String Actions[]){


  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        Identify_action(Actions[i]);
        break;
        //Mech_Arm(Drop[i]);
      }
      if (junction=="TB"){        // white Box***** DON't INVERT
        break;

      }
      junction="a";
    }
  }

}

*/
void moveTurnRightPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.forward();
  motor2.setSpeed(motor2Speed);
  motor2.backward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}

void moveTurnLeftPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.backward();
  motor2.setSpeed(motor2Speed);
  motor2.forward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}



void Task1(){  
  if(flag== 0){
    turnRightStart();
    flag = 1;
  }
  colomcheck();

  while (ball_count==5 && plus==0){



        //Serial.println("hee");
        if (endline == 0){
          colour_line_follow();
          
        }
        
        if(Tdetect == 1){
          turnRight();
    
          encoderCountA=0;
          encoderCountB=0;
          while((encoderCountA < 170) && (encoderCountB < 170)){
            //delay(5000);
            readSensors(sensorValues);
            float pidValue = calculatePID(sensorValues);
            PID_Linefollow(pidValue);
          }
          
          turnLeftNew();
    
    
          Tdetect = 0;
          endline = 1;
    
        }
        if (endline == 1){
          readSensors(sensorValues);
          while(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2){
            moveStraightPID();
            readSensors(sensorValues);
    
          }
          encoderCountA = 0;
          encoderCountB = 0;
    
          while (encoderCountA < 80 && encoderCountB < 80) {
            moveStraightPID();
          }
          motor1.stop();
          motor2.stop();
          delay(5);
          
    
          turnLeftNew();
    
          encoderCountA = 0;
          encoderCountB = 0;
    
          while (encoderCountA < 80 && encoderCountB < 80) {
            moveStraightPID();
          }
          motor1.stop();
          motor2.stop();
          delay(5);
          delay(5000);
    
    
    
          // dropping
          rightServo.write(90);  // Move to 0 degrees
          delay(1000);       // Wait 1 second
    
    
          encoderCountA = 0;
          encoderCountB = 0;
    
          while (encoderCountA < 180 && encoderCountB < 180) {
            moveStraightPID();
          }
          motor1.stop();
          motor2.stop();
          delay(5);
    
          turnLeftNew();
          turnLeftNew();
    
          encoderCountA = 0;
          encoderCountB = 0;
    
          while (encoderCountA < 700 && encoderCountB < 700) {
            moveStraightPID();
          }
          motor1.stop();
          motor2.stop();
          delay(5); 
          leftServo.write(90); // Move to 90 degrees
          delay(1000);  
          while(true){
            motor1.stop();
            motor2.stop();
            delay(5); 
          }     
          
    
        }






  }  
    /*motor1.stop();
    motor2.stop();
    delay(1000);*/

}
  
int Green_White_Detect(){
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  //Serial.println(g);
  if (35<g && g<230){ // then green
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    //digitalWrite(13, HIGH);
    return 1 ;
  } 
  else{// white
    //digitalWrite(13, LOW);
    return 0;
  }
}

void readColors() {
  unsigned long redSum = 0, greenSum = 0, blueSum = 0;
  const int samples = 5;  // Average over 5 readings

  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    redSum += pulseIn(OUT, LOW, 10000);  // 10ms timeout

    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    greenSum += pulseIn(OUT, LOW, 10000);

    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    blueSum += pulseIn(OUT, LOW, 10000);

    delay(10);  // Small delay between samples
  }

  redFreq = redSum / samples;
  greenFreq = greenSum / samples;
  blueFreq = blueSum / samples;
}

String ball_colour(){
  
  readColors();
  if (redFreq>redMinYellow && blueFreq>blueMinYellow){
    //Serial.println("Yellow");
    return "Yellow";

  }
  else {
    //Serial.println("White");
    return "White";

  }
}

  
void move_for_grabbing(){
  encoderCountA=0;
  encoderCountB=0;
  while((encoderCountA < 65) && (encoderCountB < 65)){
    //delay(5000);
    readSensors(sensorValues);
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
 
  motor1.stop();
  motor2.stop();

  smoothMoveServo(baseServo,basePos,10);
  delay(500);
  smoothMoveServo(shoulderServo,shoulderPos,50);
  delay(500);
  smoothMoveServo(elbowServo,elbowPos,55);
  delay(500);
  smoothMoveServo(shoulderServo,shoulderPos,75);
  delay(500);
  smoothMoveServo(elbowServo,elbowPos,50);
  delay(500);
  String colour_ = ball_colour();       
  smoothMoveServo(gripperServo,gripperPos,150);
  delay(500);
  //String colour_ = ball_colour();
  smoothMoveServo(elbowServo,elbowPos,180);
  delay(500);
  smoothMoveServo(shoulderServo,shoulderPos,0);
  delay(500);
  if (colour_=="Yellow"){   //(temp_yellow_white[tem_pos]=='Y'){
    smoothMoveServo(baseServo,basePos,110);
    digitalWrite(30, HIGH);
    delay(500); 
    digitalWrite(30, LOW);

  }else{
    smoothMoveServo(baseServo,basePos,150);
    digitalWrite(31, HIGH);
    delay(500);
    digitalWrite(31, LOW);
  }
  delay(500);
  smoothMoveServo(gripperServo,gripperPos,0);



  //grabing code
  delay(500);

  encoderCountA=0;
  encoderCountB=0;
  while((encoderCountA < 30) && (encoderCountB < 30)){
    encoder_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(500);
  //half_rotation();
  

  ball_count++;

  tem_pos++;
  
}


void smoothMoveServo(Servo &servo, int &currentPos, int targetPos) {
  // Constrain target to the servo's specific range
  targetPos = constrain(targetPos, 0, 180);

  // Calculate the direction and total steps
  int delta = targetPos - currentPos;
  int steps = abs(delta) / 1;
  int direction = (delta > 0) ? 1 : -1;

  // Move incrementally toward the target
  for (int i = 0; i <= steps; i++) {
    currentPos += direction * 1;
    if (direction > 0 && currentPos > targetPos) currentPos = targetPos;
    if (direction < 0 && currentPos < targetPos) currentPos = targetPos;
    
    servo.write(currentPos);
    delay(15);
  }

  // Ensure final position is exact
  currentPos = targetPos;
  servo.write(currentPos);
}

// void Grab


void colomcheck(){
  colour_line_follow();
  if(is_plus ==true && plus == 0 && green_temp_array[tem_pos]==0) { //green_detect == 1  ){
    //turnRight45();
    turnRight();
    move_for_grabbing();
    turnLeft();
    //turnLeft45();
    plus = 0;
    is_plus =false;
  }
  else if (is_plus ==true  && plus == 0 && green_temp_array[tem_pos]!=0){ //&& green_detect == 0
    delay(500);
    turnRight();
    plus+=1;
    is_plus =false;
    
  }
  else if (is_plus ==true  && plus == 1 && green_temp_array[tem_pos]==1){ //&& green_detect == 1
    //turnLeft45();
    move_for_grabbing();
    
    // graping code
    //turnLeft45();
    //turnLeft();
    half_rotation_L();
    plus = 4;
    is_plus =false;     
  }
  else if (is_plus ==true  && plus == 1 && green_temp_array[tem_pos]!=1){ //&& green_detect == 0
    //turnRight();
    delay(500);
    plus+=1;
    is_plus =false;    
  }
  else if (is_plus ==true  && plus == 2 && green_temp_array[tem_pos]==2){ //&& green_detect == 1
    //turnLeft45();
    move_for_grabbing();
    
    // graping code
    //turnLeft45();
    //turnLeft();
    half_rotation_L();
    plus = 3;
    is_plus =false;     
  }
  else if (is_plus ==true  && plus == 2 && green_temp_array[tem_pos]!=2){ //&& green_detect == 0
    delay(500);
    half_rotation();
    

    check_again = true;

    plus = 3;
    is_plus =false;     
  }
  else if (is_plus ==true && plus == 3){
    delay(500);
    plus=4;
    is_plus =false;    
  }
  else if (is_plus ==true && plus==4){
    delay(500);
    turnRight();
    
    if (check_again == true){
      small_backward();
      check_again = false;
    }
    plus=0;
    is_plus=false;
  }
  

}


