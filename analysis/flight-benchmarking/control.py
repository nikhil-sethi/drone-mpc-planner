#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

key_time = 'time'
key_stepduration = 'step_duration'
key_stepamplitude = 'step_amplitude'
key_stepdirection = 'step_direction'
key_overshooterror = 'overshoot_error'
key_firstpasserror = 'firstpass_error'
key_firstpasstime = 'firstpass_time'
key_trajectoryerror = 'trajectory_error'
key_isovershooted = 'is_overshooted'

def init_step_data():
	return {key_time: np.nan, 
		key_stepdirection: np.array([np.nan, np.nan, np.nan]), 
		key_stepdirection: np.nan,
		key_stepamplitude: np.nan, 
		key_firstpasserror: np.nan,
		key_firstpasstime: np.nan,
		key_overshooterror: np.nan,
		key_trajectoryerror: np.nan,
		key_isovershooted: False }


def is_setpoint_changed(prev_target, target):
	"""Return as first argument true if the setpoint has changed, else false.
		Return as second argument the number of dimension which have changed."""
	if(not any(np.isnan(prev_target)) and not any(np.isnan(target))):
		ndim_target_changed = 3- np.count_nonzero(abs(target-prev_target)<0.01)
	else:
		ndim_target_changed = 0

	return ndim_target_changed!=0

def overshoot_error(time, step_direction, error):
	overshooterror = np.dot(-step_direction, error)
	if(overshooterror<0):
		overshooterror = 0
	return overshooterror

def trajectory_error(step_direction, error):
	traj_err = np.linalg.norm(np.cross(error, step_direction)) / np.linalg.norm(step_direction) 
	return traj_err

def eval_control(prev_target, target, pos, err, time, step_found, nav_state, step, steps_list):
	setpoint_changed = is_setpoint_changed(prev_target, target)	
	if(setpoint_changed):
		if step_found and nav_state!=22:
			step[key_stepduration] = time - step[key_time]
			if(step[key_stepduration]>0.3):
				steps_list.append(step)
		else:
			step_found = True

		step = init_step_data()
		step[key_time] = time
		step[key_stepamplitude] = np.linalg.norm(target - prev_target)
		step[key_stepdirection] = (target - prev_target) / step[key_stepamplitude]
	else:
		if(step_found):
			overshooterror = overshoot_error(time, step[key_stepdirection], err)
			trajectoryerror = trajectory_error(step[key_stepdirection], err)
			if(overshooterror>step[key_overshooterror] or np.isnan(step[key_overshooterror])):
				step[key_overshooterror] = overshooterror
				if(overshooterror>0 and step[key_isovershooted] == False):
					step[key_isovershooted] = True
					step[key_firstpasserror] = trajectoryerror
					step[key_firstpasstime] = time - step[key_time]

			if(trajectoryerror>step[key_trajectoryerror] or np.isnan(step[key_trajectoryerror])):
				step[key_trajectoryerror] = trajectoryerror

	return [step_found, step, steps_list]

key_firstpasstime_unified = 'firstpass_time_unified'

def control_evaldata(step_stats):
	overshoot = []
	firstpasserror = []
	firstpasstime = []
	firstpass_time_unified = []
	trajectory_error = []
	for step in step_stats:
		if(not np.isnan(step[key_overshooterror])):
			overshoot.append(step[key_overshooterror])
		if(not np.isnan(step[key_firstpasserror])):
			firstpasserror.append(step[key_firstpasserror])
		if(not np.isnan(step[key_firstpasstime])):
			firstpasstime.append(step[key_firstpasstime])
		if(not np.isnan(step[key_firstpasstime]) and not np.isnan(step[key_stepamplitude])):
			firstpass_time_unified.append(step[key_firstpasstime]/step[key_stepamplitude])
		if(not np.isnan(step[key_trajectoryerror])):
			trajectory_error.append(step[key_trajectoryerror])

	control_evaldata = {}
	if(len(overshoot)>0):
		control_evaldata[key_overshooterror] = overshoot
	if(len(firstpasserror)>0):
		control_evaldata[key_firstpasserror] = firstpasserror
	if(len(firstpasstime)>0):
		control_evaldata[key_firstpasstime] = firstpasstime
	if(len(firstpass_time_unified)>0):
		control_evaldata[key_firstpasstime_unified] = firstpass_time_unified
	if(len(trajectory_error)>0):
		control_evaldata[key_trajectoryerror] = trajectory_error

	return control_evaldata

def control_evaldata_units():
	control_evaldata_units = {}
	control_evaldata_units[key_overshooterror] = 'm'
	control_evaldata_units[key_firstpasserror] = 'm'
	control_evaldata_units[key_firstpasstime] = 's'
	control_evaldata_units[key_firstpasstime_unified] = 's/m'
	control_evaldata_units[key_trajectoryerror] = 'm'

	return control_evaldata_units

if __name__ == "__main__":
	pass
