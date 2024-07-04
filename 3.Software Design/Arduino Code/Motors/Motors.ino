#include <SoftwareSerial.h>
#include "MeSmartServo.h"
#include "MeMegaPi.h"


// Pin for Pump
const int PumpPin = A10;

// Definir los pines para el driver A4988
const int dirPin = 10;
const int stepPin = 11;

// Definir el número de pasos por revolución (ajusta según tu motor)
const int stepsPerRevolution = 200;

// Velocidad del motor (ajustar para controlar la velocidad)
const int motorSpeed = 1000; // microsegundos de retraso entre pasos

// Variables and Constants definition for Servo
const char Error_Servo1 = -10;
const char Error_Servo2 = 30;
const char Error_Servo3 = 0;
const bool Debug = true;
long loopTime = 0;

// Function Prototyping
void MoveArmTo(int angle1, int angle2, int angle3, unsigned char speed=15);
void rotateAngle(float angle, bool clockwise);

// Initialize the Servo Class
MeSmartServo mysmartservo(PORT5); // Works with RX2 & TX2 in the arduino MEGA (Pins 16-17)

int StepDelay=500;                      // Amount of miliseconds between each step of the acceleration
int currentSpeed[4] = {0, 0, 0, 0};     // Current speed for each motor
int RobotMotors[4][4] = {               // Array of motor pin configurations
    {48, 50, 12, 1},   // IN1/IN2 | IN3/IN4 | ENABLE | ID
    {52, A8, 44, 3},
    {49, 51, 13, 2},
    {53, A9, 45, 4}
};

// Function Prototyping
void Robot_AccelTo(int motorTargetSpeeds[4], float accelTime=3.0);
void Robot_LinealAccel(int direction, int globalTargetSpeed);

void Motor_Accel(int motorIndex, int MotorPins[3], int targetSpeed, int AccelSteps=1);
void Motor_SetSpeed(int MotorPins[3], int actspeed);
void Motor_Break(int MotorPins[3]);

bool CompareSpeedArrays(int arr1[], int arr2[]);



void setup() {
    // Configurar los pines como salidas
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(PumpPin, OUTPUT);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            pinMode(RobotMotors[i][j], OUTPUT);
        }
    }
    
    Serial.begin(9600);

    for (int i = 0; i < 4; i++) {
        Motor_Break(RobotMotors[i]); // Stop motors
    }

    // Servo setup
    mysmartservo.begin(115200);
    delay(5);
    mysmartservo.assignDevIdRequest();
    delay(50);

    loopTime = millis();
    MoveArmTo(0, 0, 0);
    delay(500);
    rotateAngle(600, true); // Rotate 90 degrees counterclockwise
    delay(500);
    rotateAngle(600, false); // Rotate 90 degrees clockwise
    delay(500);
    MoveArmTo(-90, 15, 45);
    delay(5000);
}


void loop() {

    digitalWrite(PumpPin, LOW);
    MoveArmTo(0, 0, 0);
    rotateAngle(600, true); // Rotate 90 degrees counterclockwise

    MoveArmTo(0, -45, -45);
    rotateAngle(400, false);  // Rotate 90 degrees clockwise
    digitalWrite(PumpPin, HIGH);
    delay(500);

    rotateAngle(400, true);  // Rotate 90 degrees clockwise
    delay(500);

    MoveArmTo(-65, 90, -90);
    MoveArmTo(60, 45, 90);
    MoveArmTo(0, -45, -45);

    delay(500);
    rotateAngle(200, false); // Rotate 90 degrees counterclockwise
    digitalWrite(PumpPin, LOW);
    delay(1000);

    rotateAngle(400, true); // Rotate 90 degrees counterclockwise
    MoveArmTo(0, 0, 0);
   
    rotateAngle(600, false); // Rotate 90 degrees counterclockwise
    MoveArmTo(-90, 15, 45);
    delay(5000);

    int turnForward[4] = {200, 200, 200, 200};
    Robot_AccelTo(turnForward);
    delay(500); // Wait for a while
    int turnBackward[4] = {-200, -200, -200, -200};
    Robot_AccelTo(turnBackward);
    delay(250); // Wait for a while
    Robot_Break();
    delay(2000);
}



// Robot General Functions

void Robot_AccelTo(int motorTargetSpeeds[4], float accelTime) {
    int motorAccelSteps[4]; // Array to hold accelerations for each motor

    // Calculate the acceleration steps for each motor
    for (int i = 0; i < 4; i++) {
        motorAccelSteps[i] = abs(motorTargetSpeeds[i] - currentSpeed[i])/(accelTime*1000/StepDelay);
        Serial.print(motorAccelSteps[i]);
        Serial.print(" | ");
        Serial.println(motorTargetSpeeds[i]);
    }

    // Accelerate all motors simultaneously
    while (!CompareSpeedArrays(currentSpeed,motorTargetSpeeds))
    {
        for (int i = 0; i < 4; i++) {
            Motor_Accel(i, RobotMotors[i], motorTargetSpeeds[i], motorAccelSteps[i]);
        }

        delay(StepDelay); // Delay to make acceleration smooth
    }
}


void Robot_LinealAccel(int direction, int globalTargetSpeed) {
    int motorSpeedTarget[4]; // Array to hold target speeds for each motor

    unsigned char targetSpeed = map(globalTargetSpeed, 0, 100, 0, 255);

    // Calculate the target speeds for each motor based on globalTargetSpeed and direction
    motorSpeedTarget[0] = round(targetSpeed*cos(radians(direction-135)));
    motorSpeedTarget[1] = -1*round(targetSpeed*cos(radians(direction-45)));
    motorSpeedTarget[2] = -1*motorSpeedTarget[0];
    motorSpeedTarget[3] = -1*motorSpeedTarget[1];

    Robot_AccelTo(motorSpeedTarget);
}


void Robot_TurnAccel(int turningSpeed) {
    int motorTurnSpeedTarget[4]; // Array to hold target speeds for each motor

    unsigned char turnTargetSpeed = map(turningSpeed, 0, 100, 0, 255);

    // Calculate the turning target speeds for each motor
    for(int i = 0; i < 4; i++) {
        motorTurnSpeedTarget[i] = turnTargetSpeed;
    }

    Robot_AccelTo(motorTurnSpeedTarget);
}


void Robot_Break() {
    int stop[4] = {0,0,0,0};
    Robot_AccelTo(stop);

    for(int i = 0; i < 4; i++) {
        Motor_Break(RobotMotors[i]);
    }

    delay(100);
}



//Individual Motor Functions

void Motor_Accel(int motorIndex, int MotorPins[3], int targetSpeed, int AccelSteps) {
    // Accelerate the motor to the target speed
    if (currentSpeed[motorIndex] < targetSpeed) {
        currentSpeed[motorIndex] += AccelSteps;
        if (currentSpeed[motorIndex] > targetSpeed) {
            currentSpeed[motorIndex] = targetSpeed;
        }
    } else if (currentSpeed[motorIndex] > targetSpeed) {
        currentSpeed[motorIndex] -= AccelSteps;
        if (currentSpeed[motorIndex] < targetSpeed) {
            currentSpeed[motorIndex] = targetSpeed;
        }
    }

    // Handle small speeds to avoid stalling
    if (currentSpeed[motorIndex] >= -25 && currentSpeed[motorIndex] <= 25 && targetSpeed != 0) {
        if (abs(currentSpeed[motorIndex]) == AccelSteps) {
            currentSpeed[motorIndex] = 26 * targetSpeed / abs(targetSpeed);
        } else {
            currentSpeed[motorIndex] = 0;
        }
    } else if (currentSpeed[motorIndex] >= -25 && currentSpeed[motorIndex] <= 25 && targetSpeed == 0) {
        currentSpeed[motorIndex] = 0;
    }

    Motor_SetSpeed(MotorPins, currentSpeed[motorIndex]); // Set motor speed
}


void Motor_SetSpeed(int MotorPins[3], int actspeed) {
    // Set motor direction and speed
    if (actspeed > 0) {
        digitalWrite(MotorPins[0], LOW);
        digitalWrite(MotorPins[1], HIGH);
        actspeed = max(actspeed, 25);
    } else if (actspeed < 0) {
        digitalWrite(MotorPins[0], HIGH);
        digitalWrite(MotorPins[1], LOW);
    } else {
        Motor_Break(MotorPins);
    }

    if (actspeed >= -20 && actspeed <= 20) {
        actspeed = 0; // Protect motor from burning out without turning
    }

    Serial.print("Motor ");
    Serial.print(MotorPins[3]);
    Serial.print(", Speed = ");
    Serial.println(actspeed);

    analogWrite(MotorPins[2], max(-255, min(255, abs(actspeed)))); // Set motor speed using PWM
}


void Motor_Break(int MotorPins[3]) {
    // Stop the motor by setting all pins low
    digitalWrite(MotorPins[0], LOW);
    digitalWrite(MotorPins[1], LOW);
    analogWrite(MotorPins[2], 0);

    if (StepDelay < 100) {  // Make sure the safety stop time takes place
        delay(100);
    }
}

// Other Functions

bool CompareSpeedArrays(int arr1[], int arr2[]) {
    // Return True if they are equal
    for (int i = 0; i < 4; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}

// Function to make it easy to manipulate the arm by passing the angles of the three servos 
void MoveArmTo(int angle1, int angle2, int angle3, unsigned char speed) {
  mysmartservo.moveTo(1, angle1 + Error_Servo1, speed);
  mysmartservo.moveTo(2, angle2 + Error_Servo2, speed);
  mysmartservo.moveTo(3, angle3 + Error_Servo3, speed);

  // Wait for the arm to move and show data if debugging
  loopTime = millis();
  while (millis() - loopTime < 2000) {
    if (Debug) {
      for (int i = 1; i < 4; i++) {
        Serial.print((String)"SERVO " + i);
        Serial.print((String)" || Angle: " + mysmartservo.getAngleRequest(i));
        Serial.print((String)" | Speed: " + mysmartservo.getSpeedRequest(i));
        Serial.print((String)" | Voltage: " + mysmartservo.getVoltageRequest(i));
        Serial.print((String)" | Temp: " + mysmartservo.getTempRequest(i));
        Serial.println((String)" | Current: " + mysmartservo.getCurrentRequest(i));
      }
      Serial.println();
    }
  }
}

// Function to rotate the stepper motor by a specific angle
void rotateAngle(float angle, bool clockwise) {
  // Calculate the number of steps required
  int steps = (int)((angle / 360.0) * stepsPerRevolution);

  // Set the motor direction
  if (clockwise) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Move the motor the calculated number of steps
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorSpeed);
  }

  // Print the angle and direction to the serial monitor
  Serial.print("Rotated ");
  Serial.print(angle);
  Serial.print(" degrees ");
  if (clockwise) {
    Serial.println("clockwise.");
  } else {
    Serial.println("counterclockwise.");
  }
}