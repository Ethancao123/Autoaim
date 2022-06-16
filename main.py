# Standard imports
import cv2
import numpy as np;

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
        canvas = cv2.drawKeypoints(canvas, keypoints,np.array([]), (0,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # cv2.imshow('frame',frame)
        cv2.imshow('canvas',canvas)
        cv2.imshow('mask',mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
cam.release()
cv2.destroyAllWindows()