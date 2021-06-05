#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

KEY_TIME = 'time'
KEY_STEPDURATION = 'step_duration'
KEY_STEPAMPLITUDE = 'step_amplitude'
KEY_STEPDIRECTION = 'step_direction'
KEY_OVERSHOOTERROR = 'overshoot_error'
KEY_FIRSTPASSERROR = 'firstpass_error'
KEY_FIRSTPASSTIME = 'firstpass_time'
KET_TRAJECTORYERROR = 'trajectory_error'
KEY_ISOVERSHOOTED = 'is_overshooted'


def init_step_data():
    return {KEY_TIME: np.nan,
            KEY_STEPDIRECTION: np.array([np.nan, np.nan, np.nan]),
            KEY_STEPDIRECTION: np.nan,
            KEY_STEPAMPLITUDE: np.nan,
            KEY_FIRSTPASSERROR: np.nan,
            KEY_FIRSTPASSTIME: np.nan,
            KEY_OVERSHOOTERROR: np.nan,
            KET_TRAJECTORYERROR: np.nan,
            KEY_ISOVERSHOOTED: False}


def is_setpoint_changed(prev_target, target):
    """Return as first argument true if the setpoint has changed, else false.
            Return as second argument the number of dimension which have changed."""
    if(not any(np.isnan(prev_target)) and not any(np.isnan(target))):
        ndim_target_changed = 3 - np.count_nonzero(abs(target - prev_target) < 0.01)
    else:
        ndim_target_changed = 0

    return ndim_target_changed != 0


def overshoot_error(time, step_direction, error):
    overshooterror = np.dot(-step_direction, error)
    if overshooterror < 0:
        overshooterror = 0
    return overshooterror


def trajectory_error(step_direction, error):
    traj_err = np.linalg.norm(np.cross(error, step_direction)) / np.linalg.norm(step_direction)
    return traj_err


def eval_control(prev_target, target, pos, err, time, step_found, nav_state, step, steps_list):
    setpoint_changed = is_setpoint_changed(prev_target, target)
    if setpoint_changed:
        if step_found and nav_state != 22:
            step[KEY_STEPDURATION] = time - step[KEY_TIME]
            if step[KEY_STEPDURATION] > 0.3:
                steps_list.append(step)
        else:
            step_found = True

        step = init_step_data()
        step[KEY_TIME] = time
        step[KEY_STEPAMPLITUDE] = np.linalg.norm(target - prev_target)
        step[KEY_STEPDIRECTION] = (target - prev_target) / step[KEY_STEPAMPLITUDE]
    else:
        if step_found:
            overshooterror = overshoot_error(time, step[KEY_STEPDIRECTION], err)
            trajectoryerror = trajectory_error(step[KEY_STEPDIRECTION], err)
            if overshooterror > step[KEY_OVERSHOOTERROR] or np.isnan(step[KEY_OVERSHOOTERROR]):
                step[KEY_OVERSHOOTERROR] = overshooterror
                if overshooterror > 0 and not step[KEY_ISOVERSHOOTED]:
                    step[KEY_ISOVERSHOOTED] = True
                    step[KEY_FIRSTPASSERROR] = trajectoryerror
                    step[KEY_FIRSTPASSTIME] = time - step[KEY_TIME]

            if trajectoryerror > step[KET_TRAJECTORYERROR] or np.isnan(step[KET_TRAJECTORYERROR]):
                step[KET_TRAJECTORYERROR] = trajectoryerror

    return [step_found, step, steps_list]


KEY_FIRSTPASSTIME_UNIFIED = 'firstpass_time_unified'


def control_evaldata(step_stats):
    overshoot = []
    firstpasserror = []
    firstpasstime = []
    firstpass_time_unified = []
    trajectory_error = []
    for step in step_stats:
        if not np.isnan(step[KEY_OVERSHOOTERROR]):
            overshoot.append(step[KEY_OVERSHOOTERROR])
        if not np.isnan(step[KEY_FIRSTPASSERROR]):
            firstpasserror.append(step[KEY_FIRSTPASSERROR])
        if not np.isnan(step[KEY_FIRSTPASSTIME]):
            firstpasstime.append(step[KEY_FIRSTPASSTIME])
        if not np.isnan(step[KEY_FIRSTPASSTIME]) and not np.isnan(step[KEY_STEPAMPLITUDE]):
            firstpass_time_unified.append(step[KEY_FIRSTPASSTIME] / step[KEY_STEPAMPLITUDE])
        if not np.isnan(step[KET_TRAJECTORYERROR]):
            trajectory_error.append(step[KET_TRAJECTORYERROR])

    control_evaldata = {}
    if overshoot:
        control_evaldata[KEY_OVERSHOOTERROR] = overshoot
    if firstpasserror:
        control_evaldata[KEY_FIRSTPASSERROR] = firstpasserror
    if firstpasstime:
        control_evaldata[KEY_FIRSTPASSTIME] = firstpasstime
    if firstpass_time_unified:
        control_evaldata[KEY_FIRSTPASSTIME_UNIFIED] = firstpass_time_unified
    if trajectory_error:
        control_evaldata[KET_TRAJECTORYERROR] = trajectory_error

    return control_evaldata


def control_evaldata_units():
    control_evaldata_units = {}
    control_evaldata_units[KEY_OVERSHOOTERROR] = 'm'
    control_evaldata_units[KEY_FIRSTPASSERROR] = 'm'
    control_evaldata_units[KEY_FIRSTPASSTIME] = 's'
    control_evaldata_units[KEY_FIRSTPASSTIME_UNIFIED] = 's/m'
    control_evaldata_units[KET_TRAJECTORYERROR] = 'm'

    return control_evaldata_units


if __name__ == "__main__":
    pass
