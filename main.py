# Standard imports
import cv2
import numpy as np;

import serial
import time
from datetime import datetime

#all in mm
#might be adjusted after rollers are added
topMotorPos = (0, 160) # top motor (X)
leftMotorPos = (-368.3, -165) # left motor (Y)
rightMotorPos = (368.3, -165) # right motor (Z)
dist = 70.5 # attachment point distance
pulleyCir = 55 * np.pi

currentPos = (0,0)

currentTopMotor = 0
currentLeftMotor = 0
currentRightMotor = 0

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))

# currently is x:(-50,50) y(-25,10)
def inBoundry(pos):
    return (pos[0] < 20 and pos[0] > -20 and pos[1] > -10 and pos[1] < 7)

def command(ser, command):
  start_time = datetime.now()
  ser.write(str.encode(command)) 
#   time.sleep(1)
  print(command)
  while True:
    break
    line = ser.readline()
    print(line)

    if line == 'b\'ok 0\\r\\n\'':
      break

#figures out the coordinates of each anchor point
def recalcPos():
    if inBoundry(currentPos):
        xyOffset = dist / np.sqrt(2)
        global botLeft
        botLeft = (currentPos[0] - xyOffset, currentPos[1] - xyOffset)
        global botRight 
        botRight = (currentPos[0] + xyOffset, currentPos[1] - xyOffset)
        global top
        top = (currentPos[0], currentPos[1] + dist)
        return True
    else:
        print("point out of bounds")
        return False

def movePos(v):
    global currentPos
    currentPos = np.add(currentPos, v)
    print(currentPos)
    if not recalcPos():
        currentPos = np.subtract(currentPos, v)

def moveMotors():
    lenX = distance(topMotorPos, top)
    lenY = distance(leftMotorPos, botLeft)
    lenZ = distance(rightMotorPos, botRight)
    global currentTopMotor 
    currentTopMotor = lenX - initTop
    global currentLeftMotor 
    currentLeftMotor = lenY - initLeft
    global currentRightMotor
    currentRightMotor = lenZ - initRight
    command(ser, "G0 X" + str(-currentTopMotor/10) + " Y" + str(-currentLeftMotor/10) + " Z" + str(-currentRightMotor/10) + "\r\n")

def initSerial():
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(4)
    command(ser, "M17\r\n")
    command(ser, "G21\r\n")
    command(ser, "G90\r\n")
    command(ser, "M92 X200 Y200 Z200\r\n")
    command(ser, "M203 X30000 Y30000 Z30000\r\n")
    command(ser, "M204 X17200 Y17200 Z17200\r\n")

# Read image
cam = cv2.VideoCapture(0)
# finds the vector the crosshair needs to move to reach the closest target
def findMove(keypoints):
    centerX = 315
    centerY = 177
    i = 0
    minDist = 9999
    minIndex = 0
    for p in keypoints:
        x = p.pt[0] - centerX
        y = p.pt[1] - centerY
        distance = np.sqrt(x*x + y*y)
        print(distance)
        if distance < minDist:
            minDist = distance
            minIndex = i
        i = i+1
    if minIndex >= len(keypoints):
        return
    target = keypoints[minIndex]
    return (target.pt[0] - centerX, -1 * (target.pt[1] - centerY))

initSerial()
recalcPos()
initRight = distance(rightMotorPos, botRight)
initLeft = distance(leftMotorPos, botLeft)
initTop = distance(topMotorPos, top)
movePos((0,0))
moveMotors()
input()
while(1):
        ret, frame = cam.read()
        canvas = frame.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if not ret:
            break

        


        lower = (70,30,120)  #130,150,80
        upper = (120,255,255) #250,250,120
        mask = cv2.inRange(frame, lower, upper)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByCircularity = True
        params.minCircularity = 0.7
        # params.filterByArea = True

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(mask)
        move = findMove(keypoints)
        print('vector: ', move)
        if move != None:
            movePos((move[0]/100, move[1]/100))
            moveMotors()
        canvas = cv2.drawKeypoints(canvas, keypoints,np.array([]), (0,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # cv2.imshow('frame',frame)
        cv2.imshow('canvas',canvas)
        cv2.imshow('mask',mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
cam.release()
cv2.destroyAllWindows()