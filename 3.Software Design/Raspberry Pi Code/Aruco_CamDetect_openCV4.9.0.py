#########################################################
#                                                       #
# Get real world pose from Aruco marker                 #
# Sources: https://www.youtube.com/watch?v=cIVZRuVdv1o  #
#          https://github.com/JarneVdP/Bachelorproef    #
#                                                       #
#########################################################

# ------------- Imports --------------
from matplotlib.pyplot import gray
import numpy as np
import cv2
import math
import time
import datetime as dt


# ------------- Functions ----------------

def isRotationMatrix(R):
    '''Checks if a matrix is a valid rotation matrix. 
    Source: https://learnopencv.com/rotation-matrix-to-euler-angles/'''

    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)

    return n < 1e-6




def rotationMatrixToEulerAngles(R):
    '''Calculates rotation matrix to euler angles
    The result is the same as the MATLAB function rotm2eul, but the order of the euler angles (x and z) are swapped.'''
    
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])

    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# Estimates the pose of single markers
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):

    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
      - corners, ids, rejectedImgPoints = detector.detectMarkers(image)

    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix

    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''

    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []

    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)

    return rvecs, tvecs, trash






# ---------------- Program Setup ----------------

# Initialize time variables
t = dt.datetime.now()
t_totaltime = dt.datetime.now()

# Load camera calibration data
with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

# Define the aruco dictionary and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
parameters.adaptiveThreshConstant = 10
parameters.adaptiveThreshWinSizeMax = 151
parameters.adaptiveThreshWinSizeMin = 5
parameters.adaptiveThreshWinSizeStep = 2

# Setup the camera capture
cap = cv2.VideoCapture(0)
marker_size = 5        
camera_width = 640
camera_height = 480
camera_frame_rate = 40

cap.set(3, camera_width)  # Set width
cap.set(4, camera_height) # Set height
cap.set(5, camera_frame_rate) # Set frame rate

# Initialize variables for data transmission
send_x = 0
send_y = 0
send_heading = 0
prevTime = time.time()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


# ------------------------ Main Program ------------------------

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers in the frame
    corners, ids, rejected = detector.detectMarkers(gray)
    
    # Calculate the total time delta
    delta_totaltime = dt.datetime.now() - t_totaltime

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners)

        for i in range(len(ids)):
            # Get the center point of the tag
            center = corners[i][0]
            M = cv2.moments(center)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Write the coordinates of the center of the tag
            cv2.putText(frame, " X:"+ str(cX/10) + ", Y:" + str(cY/10), (cX, cY), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 100, 20), 2)

            rvec_list_all, tvec_list_all, _objPoints = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec = rvec_list_all[i]
            tvec = tvec_list_all[i]
            
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)


        rvec_flipped = rvec*-1
        tvec_flipped = tvec*-1
        rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
        realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
        
        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
        
        # realworld_tvec[0] : x coordinate
        # realworld_tvec[1] : y coordinate
        
        
        distance = math.sqrt(realworld_tvec[0]**2 + realworld_tvec[1]**2)
        angle = math.degrees(math.atan2(realworld_tvec[0], realworld_tvec[1]))
        
        ##goal
        ##sample is to the right of the robot -> y is positive
        ##sample is to the left of the robot -> y is negative
        #
        #if math.degrees(yaw) > 0.0 and math.degrees(yaw) < 90.0:
        #    send_x = realworld_tvec[1]
        #    send_y = -realworld_tvec[0] 
        #    send_heading = math.degrees(yaw)
        #if math.degrees(yaw) > 90.0 and math.degrees(yaw) < 180.0:
        #    send_x = -realworld_tvec[1]
        #    send_y = realworld_tvec[0] 
        #    send_heading = math.degrees(yaw) 
        #if math.degrees(yaw) < 0.0 and math.degrees(yaw) > -90.0:
        #    send_x = realworld_tvec[0]
        #    send_y = realworld_tvec[1] 
        #    send_heading = math.degrees(yaw)
        #if math.degrees(yaw) < -90.0 and math.degrees(yaw) > -180.0:
        #    send_x = -realworld_tvec[1]
        #    send_y = realworld_tvec[0] 
        #    send_heading = math.degrees(yaw) 
        #for i in range(len(ids)):
        #    # get the center point of the tag
        #    center = corners[i][0]
        #    M = cv2.moments(center)
        #    cX = int(M["m10"] / M["m00"])
        #    cY = int(M["m01"] / M["m00"])
        #    # writes the coordinates of the center of the tag
        #    #cv2.putText(frame,"x:"+ str(cX) + ", y:" + str(cY), (cX, cY), cv2.FONT_HERSHEY_TRIPLEX, 0.7,
        #    #            (0, 255, 0), 2) #center cX + 40, cY - 40, we use 50 mm markers ,(cX-25, cY-25)

        tvec_str = "ID=%s, x=%4.0f, y=%4.0f, dist=%4.0f, angle=%4.0f, time=%4.0f secs"%(ids, realworld_tvec[1], realworld_tvec[0], distance, math.degrees(yaw), delta_totaltime.seconds)
    else:
        tvec_str = "ID=[None], x= --, y= --, dist= --, angle=--, time=%4.0f secs"%(delta_totaltime.seconds)
    
    cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255), 2)
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()