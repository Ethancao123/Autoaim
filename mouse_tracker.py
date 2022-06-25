import pyautogui
import cv2
import pytesseract
import time


cap = cv2.VideoCapture('vid.mp4')
play_mode = 1 # 0 for paused, 1 for playing
score = 0
prev_score = 0

mouse_pos = []

# start_time = time.time()
# x = 1 # displays the frame rate every 1 second
# counter = 0

while True:
    ret, frame = cap.read()
    original_frame = frame

    play_mode = 1

    if not ret:
        break

    # grayscale the frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame = frame[30:55, 475:565]

    _, frame = cv2.threshold(frame, 235, 255, cv2.THRESH_BINARY)

    score = None

    try:
        rawScoreStr = pytesseract.image_to_string(frame)
        # print(rawScoreStr)
        score = int(rawScoreStr)
    except:
        pass

    if score and abs(score - prev_score) < 1000 and score != prev_score:
        # given that the length of the score and the prev_score is the same
        # if only one digit has changed from the prev_score to score,
        # then there is a high chance that the OCR has misread the score
        misread = False
        changedDigits = 0
        scoreStr = str(score)
        prevScoreStr = str(prev_score)
        if len(scoreStr) == len(prevScoreStr):
            for i in range(len(scoreStr)):
                if prevScoreStr[i] != scoreStr[i]:
                    changedDigits += 1
            if changedDigits == 1:
                misread = True

        if score > prev_score and not misread:
            shots.append(score - prev_score)
            # play_mode = 0
            prev_score = score


    cv2.imshow('frame', original_frame)

    # counter += 1
    # if (time.time() - start_time) > x:
    #     print("FPS: ", round(counter / (time.time() - start_time), 2))
    #     counter = 0
    #     start_time = time.time()


    if cv2.waitKey(play_mode) & 0xFF == ord('q'):
        break


# while true
# store mouse pos in arr
# if mouse is within a keypoint that existed last frame but doesnt exist this frame
#      push mouse_pos_arr to pathways_arr
#      clear mouse_pos_arr


# given that im viewing the shots at anglex (A, B), how do I know if,
# when viewing shots from angle (C, D), if the shots are in the same
# positions on the plane?

# - you know that all 3 shots will always be visible
# - you know that, if a successful shot has been hit across 2 frames,
#   only 1 of the shots will have a different position

# 1.
# - take the left-most shot

# how does ball size change with distance