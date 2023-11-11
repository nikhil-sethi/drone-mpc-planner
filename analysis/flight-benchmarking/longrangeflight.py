#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

from cleanwhitespaces import cleanWhitespaces
from tuningflight import read_tuningflight
from control import init_step_data, eval_control, KEY_ISOVERSHOOTED, KEY_OVERSHOOTERROR, KEY_FIRSTPASSERROR, KEY_FIRSTPASSTIME, KEY_TIME, KEY_STEPDURATION
from flightstates import init_flightstate_data, eval_current_flightstate, KEY_TAKEOFFTIME

KEY_INTERSECTIONERROR_LR = 'intersection_error_long_range'
KEY_INTERSECTIONTIME_LR = 'intersection_time_long_range'
KEY_OVERSHOOT_LR = 'overshoot_longrange'
KEY_MINIMALEROR_LR = 'minimal_error_longrange'
KEY_TIMEMINIMALERROR_LR = 'time_minimal_error_longrange'


def read_longrangeflight(drone_filepath, target):
    longrange_data = {}
    flightstates_data = {}

    drone_data_string = cleanWhitespaces(drone_filepath)
    drone_data = pd.read_csv(drone_data_string, sep=';')

    n_samples_drone = np.size(drone_data, 0)
    step_found = False
    steps_data = []
    step = init_step_data()
    flightstates_data = init_flightstate_data()

    closest_distance = np.inf
    time_at_closest_distance = np.nan
    log_target = np.array([drone_data['target_pos_x'][0], drone_data['target_pos_y'][0], drone_data['target_pos_z'][0]])

    for i in range(n_samples_drone):
        prev_logtarget = log_target
        log_target = np.array([drone_data['target_pos_x'][i], drone_data['target_pos_y'][i], drone_data['target_pos_z'][i]])
        pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
        logerr = log_target - pos
        err = np.linalg.norm(target - pos)
        time = drone_data['elapsed'][i]
        nav_state = drone_data['nav_state'][i]
        auto_throttle = drone_data['autoThrottle'][i]

        [step_found, step, steps_data] = eval_control(prev_logtarget, log_target, pos, logerr, time, step_found, nav_state, step, steps_data)
        flightstates_data = eval_current_flightstate(flightstates_data, time, nav_state, auto_throttle)

        if err < closest_distance:
            closest_distance = err
            time_at_closest_distance = time

    longrange_data[KEY_MINIMALEROR_LR] = closest_distance
    longrange_data[KEY_TIMEMINIMALERROR_LR] = time_at_closest_distance - flightstates_data[KEY_TAKEOFFTIME]
    if(len(steps_data) >= 2 and steps_data[1][KEY_ISOVERSHOOTED]):
        longrange_data[KEY_OVERSHOOT_LR] = steps_data[1][KEY_OVERSHOOTERROR]
        longrange_data[KEY_INTERSECTIONERROR_LR] = steps_data[1][KEY_FIRSTPASSERROR]
        step_begin = steps_data[0][KEY_TIME] + steps_data[0][KEY_STEPDURATION]
        abs_intersection_time = step_begin + steps_data[1][KEY_FIRSTPASSTIME]
        longrange_data[KEY_INTERSECTIONTIME_LR] = abs_intersection_time - flightstates_data[KEY_TAKEOFFTIME]

    return longrange_data, flightstates_data


def longrange_evaldata(longrange_data):
    lr_evaldat = {}
    for key in longrange_data:
        if(not np.isnan(longrange_data[key]) and not np.isinf(longrange_data[key])):
            lr_evaldat[key] = longrange_data[key]

    return lr_evaldat


def longrange_evaldata_units():
    rt = {}
    rt[KEY_MINIMALEROR_LR] = 'm'
    rt[KEY_TIMEMINIMALERROR_LR] = 's'
    rt[KEY_OVERSHOOT_LR] = 'm'
    rt[KEY_INTERSECTIONERROR_LR] = 'm'
    rt[KEY_INTERSECTIONTIME_LR] = 's'

    return rt


if __name__ == "__main__":
    import os
    import sys
    import time
    if(len(sys.argv) >= 2):
        folderpath = str(sys.argv[1])
    else:
        folderpath = os.path.expanduser('~/code/pats/pc/build-vscode/logging/')

    drone_filepath = folderpath + 'log.csv'
    print('\nEvaluate: ', folderpath)

    tic = time.time()
    print(read_longrangeflight(drone_filepath, np.array([2, -1, -3])))
    toc = time.time()
