#!/usr/bin/python3

import numpy as np
import pandas as pd
import time
import mplcursors
import pathlib

from matplotlib import pyplot as plt

current_dir = pathlib.Path(__file__).parent.absolute()

filename = "controlled_yaw_try1"

# feb14_try1_gopro_save flash time
# flash_time = 15.82

# feb14_try2_gopro_save flash time
# flash_time = 7.64

# controlled_yaw_try1 flash time
flash_time = 7.52

# controlled_yaw_try2 flash time
# flash_time = 5.01


frame_period = 1 / 60

visual_angle_data = pd.read_csv(
    str(current_dir.parents[0]) + r"/visual_angle_data/{0}_visual_angle_data.csv".format(filename))
pats_data = pd.read_csv(str(current_dir.parents[0]) + r"/pats_yaw_data/{0}_pats_log.csv".format(filename), sep=";")

existing_settings = pd.read_csv(str(current_dir.parents[0]) + r'/visual_angle_data/{0}_settings.csv'.format(filename), sep=",")
existing_settings = existing_settings.set_index("setting", drop=False)
blink_frame = existing_settings.loc['blink_frame', :][1]

time_elapsed = flash_time + \
    (visual_angle_data["frame"] - blink_frame) * frame_period

visual_angle_data['angle'] = visual_angle_data['angle'].replace(180, np.nan)

normalized_angle = visual_angle_data['angle'] / max(visual_angle_data['angle'])
normalized_yaw = pats_data[' yaw'] / max(pats_data[' yaw'])

f1 = plt.figure(1)
plt.title(filename + ": Drone Angle During landing procedure (normalized)")
plt.xlabel("time(s)")
plt.ylabel("angle (degrees)")
plt.plot(time_elapsed, normalized_angle)
plt.plot(pats_data['elapsed'], normalized_yaw)


f2 = plt.figure(2)
plt.title(filename + ": Drone Angle During landing procedure")
plt.xlabel("time(s)")
plt.ylabel("angle (degrees)")
plt.plot(time_elapsed, visual_angle_data['angle'])
plt.plot(pats_data['elapsed'], pats_data[' yaw'])

mplcursors.cursor(hover=True)

plt.show()
