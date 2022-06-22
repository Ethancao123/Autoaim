import cv2
import numpy as np

# read the video
cap = cv2.VideoCapture('vid.mp4')

PLAY_MODE = 1 # 0 for paused, 1 for playing

TOP_BAR_HEIGHT = 80


def find_keypoints(frame):
    # crop frame by removing top 80 pixels
    frame = frame[TOP_BAR_HEIGHT:, :, :]

    # Convert BGR to HSV
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([0,90,90])
    upper_blue = np.array([110,255,255])
    # Threshold the HSV image to get only blue colors
    frame = cv2.inRange(frame, lower_blue, upper_blue)

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

    for k in keypoints:
        k.pt = (k.pt[0], k.pt[1] + TOP_BAR_HEIGHT)

    return keypoints


# iterate over cap frame by frame
if __name__ == '__main__':
    while True:
        # read the frame
        ret, frame = cap.read()

        # if frame is empty, break
        if not ret:
            break

        keypoints = find_keypoints(frame) #.size, .pt

        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # show the frame
        cv2.imshow('frame', frame)

        # wait for a key to be pressed
        if cv2.waitKey(PLAY_MODE) & 0xFF == ord('q'):
            break

