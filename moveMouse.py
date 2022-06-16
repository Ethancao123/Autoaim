import serial
import time
from datetime import datetime
import numpy as np
import motors

#all in mm
#might be adjusted after rollers are added
topMotorPos = (0, 152.4) # top motor (X)
leftMotorPos = (-368.3, -152.4) # left motor (Y)
rightMotorPos = (368.3, -152.4) # right motor (Z)
dist = 70.5 # attachment point distance
pulleyCir = 60 * np.pi

currentPos = (0,0)

currentTopMotor = 0
currentLeftMotor = 0
currentRightMotor = 0

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])^2 + (p1[1]-p2[1])^2)

# currently is x:(-75,75) y(-85,25)
def inBoundry(pos):
    return (pos[0] < 75 && pos[0] > -75 && pos[1] > -85 && pos[1] < 25)

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
    currentPos = np.add(currentPos, v)
    recalcPos()

def moveMotors():
    lenX = distance(topMotorPos, top)
    lenY = distance(leftMotorPos, botLeft)
    lenZ = distance(rightMotorPos, botRight)
    global currentTopMotor 
    currentTopMotor = lenX
    global currentLeftMotor 
    currentLeftMotor = lenY
    global currentRightMotor
    currentRightMotor = lenZ
    command(ser, "G0 X" + lenX + " Y" + lenY " Z" + lenZ + "\r\n")

def initSerial():
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(4)
    command(ser, "G21\r\n")
    command(ser, "G90\r\n")
    command(ser, "M92 X60 Y60 Z60\r\n")
    command(ser, "M203 X30000 Y30000 Z30000\r\n")
    command(ser, "M204 X17200 Y17200 Z17200\r\n")

recalcPos()
lenBotLeft = distance(leftMotorPos, botLeft)
lenBotRight = distance(rightMotorPos, botRight)
lenTop = distance(topMotorPos, top)
while 1:
    movePos((int(input('x coord:')),0))
    moveMotors()


