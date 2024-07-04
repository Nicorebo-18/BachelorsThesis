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

// Setup of the program
void setup() {
  // Configurar los pines como salidas
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(PumpPin, OUTPUT);

  // Iniciar la comunicación serial
  Serial.begin(9600);
  
  // Servo setup
  mysmartservo.begin(115200);
  delay(5);
  mysmartservo.assignDevIdRequest();
  delay(50);

  loopTime = millis();
  MoveArmTo(0, 0, 0);
  delay(3000);
}

// Main Program
void loop() {

  digitalWrite(PumpPin, LOW);
  MoveArmTo(0, 0, 0);
  rotateAngle(600, true); // Rotate 90 degrees counterclockwise
  //delay(1000);

  MoveArmTo(0, -45, -45);
  rotateAngle(300, false);  // Rotate 90 degrees clockwise
  digitalWrite(PumpPin, HIGH);
  //delay(1000);

  // Move the servo arm
  MoveArmTo(-65, 90, -90);
  //delay(1000);

  // Move the stepper motor
  rotateAngle(300, true);  // Rotate 90 degrees clockwise
  MoveArmTo(60, 45, 90);
  //delay(1000);
  
  rotateAngle(300, true); // Rotate 90 degrees counterclockwise
  MoveArmTo(0, -45, -45);
  rotateAngle(600, false); // Rotate 90 degrees counterclockwise
  delay(1000);
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
