import cv2
import numpy as np
# import mss
import d3dshot
import time
import pyautogui
import keyboard
import pytesseract

pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'



PLAY_MODE = 1 # 0 for paused, 1 for playing

TOP_BAR_HEIGHT = 80
SIDE_CROPPING = 200
DOWNSAMPLER = 4

prev_score = 0

LOWER_BLUE = np.array([0,90,90])
UPPER_BLUE = np.array([110,255,255])


def findKeypoints(frame, detector):
    frame = frame[int(TOP_BAR_HEIGHT / DOWNSAMPLER):, int(SIDE_CROPPING / DOWNSAMPLER):int((frame.shape[1] - SIDE_CROPPING) / DOWNSAMPLER), :]

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    # Threshold the HSV image to get only blue colors
    frame = cv2.inRange(frame, LOWER_BLUE, UPPER_BLUE)

    keypoints = detector.detect(frame)

    for k in keypoints:
        k.pt = (k.pt[0] + int(SIDE_CROPPING / DOWNSAMPLER), k.pt[1] + int(TOP_BAR_HEIGHT / DOWNSAMPLER))

    return keypoints

# iterate over cap frame by frame
if __name__ == '__main__':
    d = d3dshot.create(capture_output="numpy", frame_buffer_size=1)
    d.display = d.displays[0]

    monitor_width = pyautogui.size()[0]
    monitor_height = pyautogui.size()[1]

    cv2.namedWindow('frame')
    cv2.moveWindow('frame', -monitor_width, 0)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = ((0.015 * monitor_width / DOWNSAMPLER) ** 2) * np.pi
    params.maxArea =((0.028 * monitor_width / DOWNSAMPLER) ** 2) * np.pi
    params.filterByConvexity = True
    params.minConvexity = 0.8

    detector = cv2.SimpleBlobDetector_create(params)

    np_array_type = type(np.array([]))

    start_time = time.time()
    cnt = 0
    d.capture()
    while True:
        frame = d.get_latest_frame()

        if type(frame) == np_array_type:           
            # keypoint detection
            # scaled_frame = cv2.resize(frame, dsize=(int(frame.shape[1] / DOWNSAMPLER), int(frame.shape[0] / DOWNSAMPLER)), interpolation=cv2.INTER_CUBIC) 
            # display_frame = cv2.cvtColor(scaled_frame, cv2.COLOR_BGR2RGB)
            # keypoints = findKeypoints(scaled_frame, detector, frame.shape[1])
            # display_frame = cv2.drawKeypoints(display_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # cv2.imshow('frame', frame) # show the frame

            # shot detection
            frame = frame[40:120, 830:1000]
            frame = cv2.resize(frame, dsize=(int(frame.shape[1] / DOWNSAMPLER), int(frame.shape[0] / DOWNSAMPLER)))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            _, frame = cv2.threshold(frame, 235, 255, cv2.THRESH_BINARY)

            score = None

            # try:
            #     rawScoreStr = pytesseract.image_to_string(frame)
            #     score = int(rawScoreStr)
            # except:
            #     pass

            # if score and abs(score - prev_score) < 1000 and score > prev_score:
            #         print(score)
            #         prev_score = score
        else:
            print("no frame")

        if cnt == 10:
            print("FPS:", cnt / (time.time() - start_time))
            cnt = 0
            start_time = time.time()
        cnt += 1
            

        # wait for a key to be pressed
        if (cv2.waitKey(PLAY_MODE) & 0xFF == ord('q')) or keyboard.is_pressed('q'):
            d.stop()
            break

