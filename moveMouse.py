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

#figures out the coordinates of each anchor point
def recalcPos():
    xyOffset = dist / np.sqrt(2)
    global botLeft
    botLeft = (currentPos[0] - xyOffset, currentPos[1] - xyOffset)
    global botRight 
    botRight = (currentPos[0] + xyOffset, currentPos[1] - xyOffset)
    global top
    top = (currentPos[0], currentPos[1] + dist)

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
    motors.move(lenX, lenY, lenZ)


recalcPos()
lenBotLeft = distance(leftMotorPos, botLeft)
lenBotRight = distance(rightMotorPos, botRight)
lenTop = distance(topMotorPos, top)
while 1:
    movePos(vector)
    moveMotors()


