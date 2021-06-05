#!/usr/bin/python3
import numpy as np
import csv
import cv2
import os.path
import sys
import pandas as pd
from detection_functions import*
import pathlib

current_dir = pathlib.Path(__file__).parent.absolute()

# ----------------------------- Control Variables -------------------------
# filename = 'feb14_try2_gopro'
# extension = ".LRV"

# filename = 'tester'
# extension = ".MP4"

# filename = 'output'
# extension = ".mp4"

# filename = 'tester_realsense'
# extension = ".mp4"

filename = 'controlled_yaw_try1'
extension = ".LRV"

find_blink_frame = False
tune_reference_detection = False

detect_tape = False

if find_blink_frame:

    blink_frame = 0

    detect_tape = False
    tune_reference_detection = False

start_frame = 0

# --------------------------------init open and save files-------------------

# determing if a settings file is available, otherwise creating new settings
if not find_blink_frame:

    if os.path.isfile(str(current_dir.parents[0]) + '/visual_angle_data/{0}_settings.csv'.format(filename)):
        print('settings file found')

        existing_settings = pd.read_csv(
            str(current_dir.parents[0]) + r'/visual_angle_data/{0}_settings.csv'.format(filename), sep=",")

        existing_settings = existing_settings.set_index("setting", drop=False)

        blink_frame = existing_settings.loc['blink_frame', :][1]
    else:
        print("no file has been found, creating new settings file")
        existing_settings = None

else:
    existing_settings = None

cap = cv2.VideoCapture(
    str(current_dir.parents[0]) + '/videos/{0}{1}'.format(filename, extension))

cv2.namedWindow('red threshold image', cv2.WINDOW_NORMAL)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)

# initializing data filename
data_file = open(
    str(current_dir.parents[0]) + '/visual_angle_data/{0}_visual_angle_data.csv'.format(filename), mode='w')
angle_data_writer = csv.writer(
    data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
angle_data_writer.writerow(
    ['frame', 'angle', 'frame rate'])

# initialize setting file
settings_file = open(
    str(current_dir.parents[0]) + '/visual_angle_data/{0}_settings.csv'.format(filename), mode='w')
settings_writer = csv.writer(
    settings_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
settings_writer.writerow(
    ['setting', "value", "h", "s", "v"])
settings_writer.writerow(
    ['blink_frame', blink_frame, 0, 0, 0])

# --------capture first frame for detection tuning of reference and drone------------

cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

ok, first_frame = cap.read()
if not ok:
    print('playback not working, check file name and extension')
    sys.exit()

# inverting image to counter wrapping problem of red in HSV
first_frame = 255 - first_frame

drone_detection_thres = np.array([[80, 70, 70], [95, 255, 255]])

drone_thres_lower, drone_thres_upper = DetectionTuning(
    first_frame, drone_detection_thres)
createVideoTrackbars('image', drone_thres_lower, drone_thres_upper)

cv2.imshow("image", first_frame)

# Detect the tape on the floor and determine angle
if detect_tape:
    roi = cv2.selectROI(first_frame)
    roi = first_frame[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]
    tape_angle, lower, upper = referenceDetection(
        roi, existing_settings, tune_reference_detection)

    # writting filter threshold settings to file
    settings_writer.writerow(
        ["lower_tape_filter", lower[0], lower[1], lower[2]])
    settings_writer.writerow(
        ["upper_tape_filter", upper[0], upper[1], upper[2]])
else:
    tape_angle = 0

first_threshold_red_image = detectRed(first_frame)
_, temp_angle = angleDetection(first_threshold_red_image, first_frame)
drone_angle = temp_angle

play = True
wrapped = False
not_ok_counter = 0

while(cap.isOpened):
    while(cap.get(cv2.CAP_PROP_POS_FRAMES) <
          cap.get(cv2.CAP_PROP_FRAME_COUNT)):

        ok, frame = cap.read()

        if ok is False:
            not_ok_counter = not_ok_counter + 1
            print("frame not recieved correctly")
            if not_ok_counter == 10:
                break
        else:
            kernel = np.ones((5, 5), np.uint8)

            # countering noise of realsense camera
            cv2.dilate(frame, kernel, iterations=1)
            cv2.erode(frame, kernel, iterations=1)
            frame = 255 - frame

            threshold_red_image = detectRed(frame)

            previous_angle = temp_angle
            threshold_red_image, temp_angle = angleDetection(
                threshold_red_image, frame)

            if temp_angle == np.nan:
                wrapped = False
                # print("wrapping reset")

            # Angle wrapping at 0 degrees detection and relevant angle change
            # TODO angle detection has changed, is wrapping still a problem?
            elif abs(previous_angle - temp_angle) > 170 and previous_angle is not np.nan and temp_angle is not np.nan:
                wrapped = not wrapped

            if wrapped:
                drone_angle = 180 + temp_angle
            else:
                drone_angle = temp_angle

            drone_angle = drone_angle - tape_angle

            # saving frame number and angle to csv filename
            angle_data_writer.writerow(
                [cap.get(cv2.CAP_PROP_POS_FRAMES), drone_angle, cv2.CAP_PROP_FPS])

            cv2.putText(frame, "frame: " + str(cap.get(cv2.CAP_PROP_POS_FRAMES)),
                        (200, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 3)
            cv2.putText(frame, "angle: " + str(round(drone_angle, 2)), (200, 300),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3)

            cv2.imshow("image", frame)
            cv2.imshow('red threshold image', threshold_red_image)

        # Video play and pause controls
        next_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
        current_frame = next_frame - 1
        previous_frame = current_frame - 1

        if next_frame == blink_frame - 1:
            cap.set(cv2.CAP_PROP_POS_FRAMES, next_frame + 21)

        key = cv2.waitKey(play)

        if key == ord('q'):
            print("video is stopped")
            break
        elif key == ord('n'):
            continue
        elif key == ord('b'):
            if previous_frame >= 0:
                cap.set(cv2.CAP_PROP_POS_FRAMES, previous_frame)
        elif key == ord('p'):
            play = not play
            print("video has been paused")

    cap.release()
    cv2.destroyAllWindows()
    print("video has finished")
    break

data_file.close()
settings_file.close()
