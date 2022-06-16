import serial
import time
from datetime import datetime
import numpy as np

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
    return (pos[0] < 50 and pos[0] > -50 and pos[1] > -25 and pos[1] < 10)

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
    else:
        print("point out of bounds")

def movePos(v):
    global currentPos
    currentPos = np.add(currentPos, v)
    print(currentPos)
    recalcPos()

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
    command(ser, "G0 X" + str(-currentTopMotor) + " Y" + str(currentLeftMotor) + " Z" + str(currentRightMotor) + "\r\n")

def initSerial():
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(4)
    command(ser, "G21\r\n")
    command(ser, "G90\r\n")
    command(ser, "M92 X20 Y20 Z20\r\n")
    command(ser, "M203 X30000 Y30000 Z30000\r\n")
    command(ser, "M204 X17200 Y17200 Z17200\r\n")

initSerial()
recalcPos()
initRight = distance(rightMotorPos, botRight)
initLeft = distance(leftMotorPos, botLeft)
initTop = distance(topMotorPos, top)
while 1:
    input('x coord:')
    movePos((10,0))
    moveMotors()
    movePos((0,5))
    moveMotors()
    movePos((-10,0))
    moveMotors()
    movePos((0,-5))
    moveMotors()


