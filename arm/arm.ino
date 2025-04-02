#include <Servo.h>

// Create servo objects for each joint
Servo baseServo;      // Base rotation (0-360° capable)
Servo shoulderServo;  // Shoulder joint (0-180°)
Servo elbowServo;    // Elbow joint (0-180°)
Servo gripperServo;  // Gripper (0-180°)

// Pin definitions
const int BASE_PIN = 7;
const int SHOULDER_PIN = 11;
const int ELBOW_PIN = 8;
const int GRIPPER_PIN = 9;

// Variables to store servo positions
int basePos = 120;     // Initial position for base (0-360°)
int shoulderPos = 0;  // Initial position for shoulder (0-180°)
int elbowPos = 180;   // Initial position for elbow (0-180°)
int gripperPos = 0;   // Gripper fully closed initially (0-180°)

void setup() {
  // Attach servos to their respective pins
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);
  
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set initial positions
  baseServo.write(basePos);         // Base: 0-360°
  shoulderServo.write(shoulderPos); // Others: 0-180°
  elbowServo.write(elbowPos);
  gripperServo.write(gripperPos);
  
  Serial.println("4DOF Robot Arm Ready");
  Serial.println("Commands: B(base 0-360), S(shoulder 0-180), E(elbow 0-180), G(gripper 0-180) followed by angle");
}

// Smooth movement function with range-specific constraints
void smoothMoveServo(Servo &servo, int &currentPos, int targetPos, int minAngle, int maxAngle, int stepDelay = 15, int stepSize = 1) {
  // Constrain target to the servo's specific range
  targetPos = constrain(targetPos, minAngle, maxAngle);
  
  // Calculate the direction and total steps
  int delta = targetPos - currentPos;
  int steps = abs(delta) / stepSize;
  int direction = (delta > 0) ? 1 : -1;

  // Move incrementally toward the target
  for (int i = 0; i <= steps; i++) {
    currentPos += direction * stepSize;
    if (direction > 0 && currentPos > targetPos) currentPos = targetPos;
    if (direction < 0 && currentPos < targetPos) currentPos = targetPos;
    
    servo.write(currentPos);
    delay(stepDelay);
  }
  
  // Ensure final position is exact
  currentPos = targetPos;
  servo.write(currentPos);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    int angle = Serial.parseInt();
    
    switch (command) {
      case 'B':  // Base control (0-360°)
        smoothMoveServo(baseServo, basePos, angle, 0, 360);  // Extended range for base
        Serial.print("Base set to: ");
        Serial.println(basePos);
        break;
        
      case 'S':  // Shoulder control (0-180°)
        smoothMoveServo(shoulderServo, shoulderPos, angle, 0, 180);
        Serial.print("Shoulder set to: ");
        Serial.println(shoulderPos);
        break;
        
      case 'E':  // Elbow control (0-180°)
        smoothMoveServo(elbowServo, elbowPos, angle, 0, 180);
        Serial.print("Elbow set to: ");
        Serial.println(elbowPos);
        break;
        
      case 'G':  // Gripper control (0-180°)
        smoothMoveServo(gripperServo, gripperPos, angle, 0, 180);
        Serial.print("Gripper set to: ");
        Serial.println(gripperPos);
        break;
    }
    
    // Clear serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}