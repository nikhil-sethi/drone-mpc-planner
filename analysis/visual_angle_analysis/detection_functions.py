import numpy as np
import csv
import cv2


def detectRed(frame):

    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')

    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    result = frame.copy()
    # frame_inv = 255-frame
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower, upper)

    red_image = cv2.bitwise_and(result, result, mask=mask)
    red_image = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)

    _, threshold_red_image = cv2.threshold(
        red_image, 10, 255, cv2.THRESH_BINARY)
    threshold_red_image = cv2.erode(
        threshold_red_image, (10, 10), iterations=1)
    threshold_red_image = cv2.dilate(threshold_red_image, None, iterations=2)

    return threshold_red_image


def angleDetection(backImage, frame):
    contours, hierarchy = cv2.findContours(
        backImage.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    top_corner = []
    bottom_corner = []
    longside = 0
    longside_corner1 = []
    longside_corner2 = []

    if len(contours) == 0:
        print('no red has been detected')
        return np.zeros(backImage.shape, np.uint8), 180
    else:
        mask = np.zeros(backImage.shape, np.uint8)
        for i in contours:
            if cv2.contourArea(i) < 600:
                cv2.drawContours(mask, [i], 0, (255, 255, 255, 255), -1)

        # largest_blob contains the largest structure in the threshold image
        largest_blob = cv2.subtract(backImage, mask)
        points = cv2.findNonZero(largest_blob)

        if points is None:
            print('no sizeable red has been detected')
            return np.zeros(backImage.shape, np.uint8), np.nan

        rect = cv2.minAreaRect(points)
        box1 = cv2.boxPoints(rect)
        box_corners = np.int0(box1)
        # print(box_corners)
        cv2.drawContours(frame, [box_corners], 0, (0, 0, 255), 2)

        # rect[1] contains width and height
        if rect[1][0] > rect[1][1]:
            longside = np.floor(rect[1][0])
        elif rect[1][0] < rect[1][1]:
            longside = np.floor(rect[1][1])

        for corner in box_corners:
            for other_corner in box_corners:
                distance = np.floor(np.sqrt(
                    (corner[0] - other_corner[0])**2 + (corner[1] - other_corner[1])**2))
                # print("distance = {0}, longside = {1}".format(
                # distance, longside))
                if np.abs(distance - longside) < 5:
                    longside_corner1 = corner
                    longside_corner2 = other_corner

        # print("longside_corner1: {0}, longside_corner2: {1}".format(
            # longside_corner1, longside_corner2))
        if longside_corner1[1] >= longside_corner2[1]:
            top_corner = longside_corner1
            bottom_corner = longside_corner2
        elif longside_corner1[1] < longside_corner2[1]:
            top_corner = longside_corner2
            bottom_corner = longside_corner1
        # print('top_corner: {0}, bottom_corner: {1}'.format(
            # top_corner, bottom_corner))

        delta_x = top_corner[0] - bottom_corner[0]
        delta_y = top_corner[1] - bottom_corner[1]

        # calculate angle in degrees
        angle = np.arctan(delta_x / delta_y) * (180 / np.pi)

        return largest_blob, angle


def DetectionTuning(image, default_thres, lower=None, upper=None):

    cv2.namedWindow('tuning', cv2.WINDOW_NORMAL)

    def nothing(x):
        pass

    # create trackbars for color change
    # Hue is from 0-179 for Opencv
    cv2.createTrackbar('HMin', 'tuning', 0, 179, nothing)
    cv2.createTrackbar('SMin', 'tuning', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'tuning', 0, 255, nothing)
    cv2.createTrackbar('HMax', 'tuning', 0, 179, nothing)
    cv2.createTrackbar('SMax', 'tuning', 0, 255, nothing)
    cv2.createTrackbar('VMax', 'tuning', 0, 255, nothing)

    # Set default value for MAX HSV trackbars.
    if lower is None and upper is None:
        cv2.setTrackbarPos('HMin', 'tuning', default_thres[0][0])
        cv2.setTrackbarPos('SMin', 'tuning', default_thres[0][1])
        cv2.setTrackbarPos('VMin', 'tuning', default_thres[0][2])
        cv2.setTrackbarPos('HMax', 'tuning', default_thres[1][0])
        cv2.setTrackbarPos('SMax', 'tuning', default_thres[1][1])
        cv2.setTrackbarPos('VMax', 'tuning', default_thres[1][2])
    else:
        cv2.setTrackbarPos('HMin', 'tuning', lower[0])
        cv2.setTrackbarPos('SMin', 'tuning', lower[1])
        cv2.setTrackbarPos('VMin', 'tuning', lower[2])
        cv2.setTrackbarPos('HMax', 'tuning', upper[0])
        cv2.setTrackbarPos('SMax', 'tuning', upper[1])
        cv2.setTrackbarPos('VMax', 'tuning', upper[2])

    # Initialize to check if HSV min/max value changes

    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0

    output = image
    wait_time = 33

    while(1):

        # get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'tuning')
        sMin = cv2.getTrackbarPos('SMin', 'tuning')
        vMin = cv2.getTrackbarPos('VMin', 'tuning')

        hMax = cv2.getTrackbarPos('HMax', 'tuning')
        sMax = cv2.getTrackbarPos('SMax', 'tuning')
        vMax = cv2.getTrackbarPos('VMax', 'tuning')

        # Set minimum and max HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)

        # Print if there is a change in HSV value
        if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax)):
            print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
                hMin, sMin, vMin, hMax, sMax, vMax))
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display output image
        cv2.imshow('tuning', output)

        # Wait longer to prevent freeze for videos.
        if cv2.waitKey(wait_time) & 0xFF == ord('d'):
            break
    cv2.destroyWindow('tuning')
    return np.array([hMin, sMin, vMin]), np.array([hMax, sMax, vMax])


def createVideoTrackbars(frame, lower=None, upper=None):
    def nothing(x):
        pass
    # Hue is from 0-179 for Opencv
    cv2.createTrackbar('HMin', frame, 0, 179, nothing)
    cv2.createTrackbar('SMin', frame, 0, 255, nothing)
    cv2.createTrackbar('VMin', frame, 0, 255, nothing)
    cv2.createTrackbar('HMax', frame, 0, 179, nothing)
    cv2.createTrackbar('SMax', frame, 0, 255, nothing)
    cv2.createTrackbar('VMax', frame, 0, 255, nothing)

    # Set default value for MAX HSV trackbars.

    if lower is None and upper is None:
        cv2.setTrackbarPos('HMin', frame, 80)
        cv2.setTrackbarPos('SMin', frame, 60)
        cv2.setTrackbarPos('VMin', frame, 60)
        cv2.setTrackbarPos('HMax', frame, 100)
        cv2.setTrackbarPos('SMax', frame, 255)
        cv2.setTrackbarPos('VMax', frame, 255)
    else:
        cv2.setTrackbarPos('HMin', frame, lower[0])
        cv2.setTrackbarPos('SMin', frame, lower[1])
        cv2.setTrackbarPos('VMin', frame, lower[2])
        cv2.setTrackbarPos('HMax', frame, upper[0])
        cv2.setTrackbarPos('SMax', frame, upper[1])
        cv2.setTrackbarPos('VMax', frame, upper[2])


def referenceDetection(frame, existing_settings, tune_reference_detection):

    if existing_settings is not None and (existing_settings.index == 'lower_tape_filter').any() and (existing_settings.index == 'upper_tape_filter').any():
        lower_setting = existing_settings.loc['lower_tape_filter', :]
        upper_setting = existing_settings.loc['upper_tape_filter', :]
        lower = np.array(
            [lower_setting[1], lower_setting[2], lower_setting[3]])
        upper = np.array(
            [upper_setting[1], upper_setting[2], upper_setting[3]])
        print('lower: {0}, upper {1}'.format(lower, upper))
    else:
        lower = np.array([0, 0, 150])
        upper = np.array([179, 255, 255])

    default_thres = None

    if tune_reference_detection:
        lower, upper = DetectionTuning(frame, default_thres, lower, upper)

    cv2.namedWindow("tape detection", cv2.WINDOW_NORMAL)
    result = frame.copy()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower, upper)

    tape_image = cv2.bitwise_and(result, result, mask=mask)
    tape_image = cv2.cvtColor(tape_image, cv2.COLOR_BGR2GRAY)

    _, threshold_tape_image = cv2.threshold(
        tape_image, 10, 255, cv2.THRESH_BINARY)

    _, tape_angle = angleDetection(threshold_tape_image, frame)

    cv2.putText(frame, str(round(tape_angle, 2)), (10, 30),
                cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
    cv2.imshow("tape detection", frame)

    return tape_angle, lower, upper
