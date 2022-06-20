# Standard imports
import cv2
import numpy as np;

import serial
import time
from datetime import datetime

#all in mm
ySpeed = 1
zSpeed = 5
kP =0.25

currentPos = (0,0)

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))

# currently is x:(-50,50) y(-25,10)
def inBoundry(pos):
    return pos[1] > 10 and pos[1] < 160

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
        return
    command(ser, "G0 Y" + str(-currentPos[0]) + " Z" + str(currentPos[1]) + "\r\n")

def initSerial():
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    command(ser, "G21\r\n")
    time.sleep(1)
    command(ser, "G90\r\n")
    time.sleep(1)
    command(ser, "M203 X150 Y150 Z150\r\n")
    time.sleep(1)
    command(ser, "M92 Y17 Z37\r\n")
    time.sleep(2)
    command(ser, "M203 Y" + str(1.7) + " Z" + str(37) + "\r\n")
    time.sleep(2)
    command(ser, "G28 Z0\r\n")
    input()
    movePos((0,70))
    time.sleep(5)

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
    global target
    target = keypoints[minIndex]
    return (target.pt[0] - centerX, -1 * (target.pt[1] - centerY))

initSerial()
recalcPos()
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
            movePos((move[0]*kP, move[1]*kP))
            input()
        canvas = cv2.drawKeypoints(canvas, keypoints,np.array([]), (0,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # cv2.imshow('frame',frame)
        cv2.imshow('canvas',canvas)
        cv2.imshow('mask',mask)
        input()
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
cam.release()
cv2.destroyAllWindows()