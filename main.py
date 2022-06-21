# Standard imports
import cv2
import numpy as np

import serial
import time
from datetime import datetime

#all in mm
Y_SPEED = 1
Z_SPEED = 5

FOV = 103 # field of view (degrees)
CM_360 = 65 # cm/360 degrees

TOP_BAR_HEIGHT = 50 # pixels from the top to remove from the frame (crops out the top bar)


currentPos = (0,0)

FOV *= np.pi / 180.0  # convert from deg to rad
CM_360 /= 2 * np.pi  # convert from cm/360 deg to cm/rad

# TODO: make sure coordinate systems are matched between opencv (test by shifting keypoints) and motor system

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))

# currently is x:(-50,50) y(-25,10)
def inBoundary(pos):
    return pos[1] > 10 and pos[1] < 160

def command(ser, command):
    start_time = datetime.now()
    ser.write(str.encode(command)) 
    print(command)

def translateMotors(v):
    global currentPos

    currentPos = np.add(currentPos, v)
    print(currentPos)

    if inBoundary(currentPos):
        command(ser, "G0 Y" + str(-currentPos[0]) + " Z" + str(currentPos[1]) + "\r\n")
    else:
        print("point out of bounds")
        currentPos = np.subtract(currentPos, v)
        return

def initSerial():
    global ser

    ser = serial.Serial('/dev/ttyUSB0', 115200)

    command(ser, "G21\r\n")
    time.sleep(1)
    command(ser, "G90\r\n")
    time.sleep(1)
    command(ser, "M203 X150 Y150 Z300\r\n")
    time.sleep(1)
    command(ser, "M92 Y17 Z37\r\n")
    time.sleep(1)
    command(ser, "G28 Z0\r\n")

    input() #modify to wait for response in the future

    translateMotors((0,100))

    input()

# return the next keypoint coordinates
def getNextKeypoint(keypoints, frame):
    i = 0
    minDist = 9999
    minIndex = 0
    for p in keypoints:
        x = p.pt[0] - frame.shape[1] / 2.0
        y = p.pt[1] - frame.shape[0] / 2.0
        distance = np.sqrt(x*x + y*y)
        print(distance)
        if distance < minDist:
            minDist = distance
            minIndex = i
        i += 1

    if minIndex >= len(keypoints):
        return

    return keypoints[minIndex].pt


def findKeypoints(frame):
    # crop frame by removing top 80 pixels
    frame = frame[TOP_BAR_HEIGHT:, :, :]

    # Convert BGR to HSV
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # threshold using color range
    lower_blue = np.array([0,90,90])
    upper_blue = np.array([110,255,255])

    frame = cv2.inRange(frame, lower_blue, upper_blue)

    # detect circular blobs from the thresholded frame
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 1250
    params.maxArea = 4000
    params.filterByConvexity = True
    params.minConvexity = 0.8

    detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(frame)

    # shift keypoint down by TOP_BAR_HEIGHT px
    for k in keypoints:
        k.pt = (k.pt[0], k.pt[1] + TOP_BAR_HEIGHT)

    return keypoints

# converts a keypoint to the mouse translation in mm required to reach that keypoint
def getMouseTranslation(keypoint, frame):
    frame_width = frame.shape[1]
    frame_height = frame.shape[0]

    x_offset = keypoint.pt[0] - frame_width / 2.0
    y_offset = keypoint.pt[1] - frame_height / 2.0

    distance_to_frame = frame_width / (2 * np.tan(FOV / 2.0))
    
    thetax = np.arctan(x_offset / distance_to_frame)
    thetay = np.arctan(y_offset / distance_to_frame)

    x_translation = CM_360 * thetax * 10 # mm distance
    y_translation = CM_360 * thetay * 10 # mm distance

    return (x_translation, y_translation)


# Read image
cam = cv2.VideoCapture(0)
initSerial()
if not inBoundary(currentPos):
        print("point out of bounds")

while True:
        ret, frame = cam.read()
        canvas = frame.copy()

        if not ret:
            break

        keypoints = findKeypoints(frame)

        if len(keypoints) > 0:
            nextKeypoint = getNextKeypoint(keypoints)
            print(nextKeypoint)

            translateMotors(getMouseTranslation(nextKeypoint))
            input()
            
        canvas = cv2.drawKeypoints(canvas, keypoints,np.array([]), (0,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow('canvas', canvas)

        input()
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
cam.release()
cv2.destroyAllWindows()