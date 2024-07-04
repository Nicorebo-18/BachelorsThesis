#include <LiquidCrystal_I2C.h>
#include <SharpDistSensor.h>
#include <Wire.h>
#include "Adafruit_VCNL4010.h"
#include <ezButton.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD address to 0x27 for a 20 chars and 4 lines display

// Ultrasonic sensors setup
const int trigPins[] = {2, 3, 4, 5};  // Front, Right, Back, Left
const int echoPins[] = {6, 7, 8, 9};  // Front, Right, Back, Left
long durations[4];
float distances[4];

// Stepper Motor Pins
const int dirPin = 10;
const int stepPin = 11;

// Infrared sensor setup
const byte sensorPin = A4;
const byte medianFilterWindowSize = 5;
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

// VCNL4010 Proximity Sensor Setup
#define TCAADDR 0x70  // I2C address of the TCA9548A
Adafruit_VCNL4010 vcnl;
bool sensorPresent[4];  // Array to store the presence of sensors
const uint8_t channels[] = {0, 1, 3, 4};  // Channels used for VCNL4010

// Define Color Sensor RGB Pins
int s0 = 30, s1 = 31, s2 = 32, s3 = 33;
const int out = 19;
volatile int flag = 0;
volatile byte counter = 0;
volatile byte countR = 0, countG = 0, countB = 0;
unsigned long previousMillisRGB = 0; // stores last time sensor was read
const long intervalRGB = 10; // interval at which to read sensor (10ms)

// Corner switches
const int numSwitches = 8;
ezButton button1(22);
ezButton button2(23);
ezButton button3(24);
ezButton button4(25);
ezButton button5(26);
ezButton button6(27);
ezButton button7(28);
ezButton button8(29);

// Variable to track time for switching displays
unsigned long previousMillis = 0;
const long interval = 10000;  // Interval for switching (10 seconds)
unsigned int displayIndex = 0;  // Index to keep track of which display to show



void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}







void setup() {
  // Initialize the LCD
  lcd.init();  // Initialize the LCD
  lcd.backlight();  // Turn on the LCD backlight

  // Initialize Stepper Motor
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  // Initialize the ultrasonic sensors
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Initialize ezButton objects and set debounce times
  button1.setDebounceTime(50);  // set debounce time to 50 milliseconds
  button2.setDebounceTime(50);
  button3.setDebounceTime(50);
  button4.setDebounceTime(50);
  button5.setDebounceTime(50);
  button6.setDebounceTime(50);
  button7.setDebounceTime(50);
  button8.setDebounceTime(50);

  // Initialize TCS (Color Sensor) 
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  // Attach interrupt to out pin (external interrupt 4 for pin 18 on Mega)
  attachInterrupt(digitalPinToInterrupt(out), ISR_INTO, CHANGE);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the VCNL4010 sensors
  Wire.begin();
  Serial.println("VCNL4010 test");
  for (uint8_t i = 0; i < 4; i++) {
    tcaSelect(channels[i]);
    if (!vcnl.begin()) {
      Serial.print("Sensor not found on channel ");
      Serial.println(channels[i]);
      sensorPresent[i] = false;
    } else {
      Serial.print("Found VCNL4010 on channel ");
      Serial.println(channels[i]);
      sensorPresent[i] = true;
    }
  }

  // Display a welcome message
  lcd.setCursor(0, 0);
  lcd.print("BACHELORS THESIS");
  lcd.setCursor(0, 1);
  lcd.print("NICOLAS  REBOLLO");
  delay(2000);  // Display the welcome message
  lcd.clear();  // Clear the LCD screen
}





void loop() {
  // Get the current time
  unsigned long currentMillis = millis();
  
  // Check if it's time to switch the display
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the previous time
    displayIndex = (displayIndex + 1) % 5;  // Increment display index (0 to 4)
    lcd.clear();  // Clear the LCD screen
  }

  // Display content based on the current index
  if(displayIndex == 0) {
    // Display distances from ultrasonic sensors
    for (int i = 0; i < 4; i++) {
      // Send a trigger pulse
      digitalWrite(trigPins[i], LOW);
      delayMicroseconds(2);
      digitalWrite(trigPins[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPins[i], LOW);

      // Read the echo pulse duration
      durations[i] = pulseIn(echoPins[i], HIGH);

      // Calculate the distance in centimeters and round to the nearest whole number
      distances[i] = round(durations[i] * 0.034 / 2);

      // Print the distance measured by each sensor
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distances[i]);
      Serial.println(" cm");

      // Display the distance on the LCD
      int row = i / 2;  // Determine the row (0 or 1)
      int col = (i % 2) * 9;  // Determine the column (0 or 9)
      lcd.setCursor(col, row);
      lcd.print("US");
      lcd.print(i + 1);
      lcd.print(":");
      lcd.print("   ");  // Clear the previous number
      lcd.setCursor(col + 4, row);
      lcd.print((int)distances[i]);  // Cast to int to avoid decimal places
    }
      
  } else if(displayIndex == 1) {
    // Display the distance from the infrared sensor
    unsigned int irDistance = sensor.getDist();
    float formattedIrDistance = irDistance / 10.0;  // Format the distance

    // Print distance to Serial
    Serial.println(irDistance);
    
    // Update the LCD with the distance
    lcd.setCursor(0, 0);
    lcd.print("IR Distance:");
    lcd.setCursor(0, 1);
    lcd.print("        ");  // Clear the previous number
    lcd.setCursor(0, 1);
    lcd.print(formattedIrDistance, 1);  // Print with one decimal place
    lcd.print(" cm");

  } else if(displayIndex == 2) {
    // Display the proximity readings from the VCNL4010 sensors
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(sensorPresent[i]);
    
      // Display the proximity value on the LCD
      int row = i / 2;  // Determine the row (0 or 1)
      int col = (i % 2) * 9;  // Determine the column (0 or 9)
      lcd.setCursor(col, row);
      lcd.print("Prx");
      lcd.print(i + 1);
      lcd.print(":");
    
      if (sensorPresent[i]) {
        tcaSelect(channels[i]);
        int proximity = vcnl.readProximity();
        int proximityValue = (proximity > 2500) ? 1 : 0;
    
        // Print the proximity value to Serial
        Serial.print("Channel ");
        Serial.print(channels[i]);
        Serial.print(" - Proximity: ");
        Serial.println(proximityValue);
        lcd.print(proximityValue);
      } else {
        lcd.print("-");
      }
    }

  } else if(displayIndex == 3) {

    button1.loop();  // MUST call the loop() function first
    button2.loop(); 
    button3.loop(); 
    button4.loop(); 
    button5.loop(); 
    button6.loop(); 
    button7.loop(); 
    button8.loop(); 


    for (int i = 0; i < numSwitches; i++) {
      int state = 0;
      switch (i) {
        case 0:
          state = button1.getState();
          break;
        case 1:
          state = button2.getState();
          break;
        case 2:
          state = button3.getState();
          break;
        case 3:
          state = button4.getState();
          break;
        case 4:
          state = button5.getState();
          break;
        case 5:
          state = button6.getState();
          break;
        case 6:
          state = button7.getState();
          break;
        case 7:
          state = button8.getState();
          break;
      }
      // Print state on LCD
      int row = i % 2;  // Determine the row (0 or 1)
      int col = (i / 2) * 4;  // Determine the column (0 or 9)
      lcd.setCursor(col, row);
      lcd.print(i + 1); // Print switch number
      lcd.print(":");
      lcd.print(!state);

      // Print information on Serial for debugging
      Serial.print(i + 1);
      Serial.print(":");
      Serial.println(!state);
    }

  } else if(displayIndex == 4) {

    if (currentMillis - previousMillisRGB >= intervalRGB) {
      previousMillisRGB = currentMillis;
      updateColorSensor();
    }

  } else {
    Serial.print("No mode Availiable, displayIndex: ");
    Serial.println(displayIndex);
    lcd.setCursor(6, 0);
    lcd.print("No mode");
    lcd.setCursor(5, 1);
    lcd.print("availiable");  // Clear the previous number
  }

  delay(100);  // Short delay to avoid rapid LCD updates
}







void updateColorSensor() {
  flag++;

  if (flag == 1) {
    countR = counter;
    Serial.print("Red = ");
    Serial.println(countR, DEC);
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("R:");
    lcd.print("  "); // Clear previous values
    lcd.setCursor(3, 0);
    lcd.print(countR);

  } else if (flag == 2) {
    countG = counter;
    Serial.print("Green = ");
    Serial.println(countG, DEC);
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
    lcd.setCursor(8, 0);
    lcd.print("G:");
    lcd.print("  "); // Clear previous values
    lcd.setCursor(11, 0);
    lcd.print(countG);
  } else if (flag == 3) {
    countB = counter;
    Serial.print("Blue = ");
    Serial.println(countB, DEC);
    Serial.println();
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    lcd.setCursor(0, 1);
    lcd.print("B:");
    lcd.print("  "); // Clear previous values
    lcd.setCursor(3, 1);
    lcd.print(countB);
  } else if (flag == 4) {
    flag = 0;
  }

  counter = 0;
}

void ISR_INTO() {
  counter++;
}