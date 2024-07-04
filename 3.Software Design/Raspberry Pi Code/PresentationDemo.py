#########################################################
#                                                       #
# Raspberry Pi Robot Control Code                       #
# This code demonstrates the functionality of the       #
# robot's upper part, including the camera, servo motor #
# MPU4060, and serial communication with Arduino.       #
#                                                       #
#########################################################

# ------------- Imports --------------
from matplotlib.pyplot import gray
import cv2
import datetime as dt
import numpy as np
import math
import RPi.GPIO as GPIO
import time
import threading
import serial

# ----------- Pin Definition -------------
# Set GPIO numbering mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(11, GPIO.OUT)
servo_cam = GPIO.PWM(11, 50)  # pin 11 for servo1, pulse 50Hz

# ------------- Functions ----------------

def Servo_Move(angle):
    global camera_angle
    camera_offset_angle = -10  # To correct an assembly error
    correctedangle = max(min((angle + camera_offset_angle), 180), 0)
    camera_angle = max(min((angle), 180), 0)
    servo_cam.ChangeDutyCycle(2 + (correctedangle / 18))
    time.sleep(0.5)
    servo_cam.ChangeDutyCycle(0)
    print("Moving Servo to " + str(correctedangle) + " | Duty Cycle: " + str(2 + (correctedangle / 18)))

# Initialize serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

# ---------------- Program Setup ----------------
# Initialize time variables
t = dt.datetime.now()
t_totaltime = dt.datetime.now()

# Setup the camera capture
cap = cv2.VideoCapture(0)
camera_width = 640
camera_height = 480
cap.set(3, camera_width)  # Set width
cap.set(4, camera_height) # Set height

# Define the initial angle for the camera
camera_angle = 90
moveServo_Process = threading.Thread(target=Servo_Move, args=(camera_angle,))
moveServo_Process.start()

# ------------------------ Main Program ------------------------
try:
    while True:
        # Wait for serial connection with Arduino
        while not ser.isOpen():
            time.sleep(1)

        # Move the camera up and down
        Servo_Move(135)  # Look 45 degrees up
        time.sleep(1)
        Servo_Move(45)   # Look 45 degrees down

        # Send MPU4060 data and check for Aruco marker for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10:
            # Send MPU4060 data (example data, replace with actual sensor reading code)
            ser.write(b"MPU4060 data\n")
            time.sleep(1)

            # Check for Aruco marker (placeholder, replace with actual detection code)
            aruco_detected = False  # Replace with actual detection logic
            if aruco_detected:
                ser.write(b"Yes\n")
            else:
                ser.write(b"No\n")

finally:
    # Release the capture and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
    
    # Clean things up at the end
    servo_cam.stop()
    GPIO.cleanup()
    ser.close()
