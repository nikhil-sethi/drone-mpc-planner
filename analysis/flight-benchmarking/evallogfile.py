#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pandas as pd 
import numpy as np
import os
import pwd
import time
from evalcontrol import eval_control, init_step_data, generate_eval_variables_control
from evalflightstates import eval_current_flightstate, init_flightstate_data, generate_eval_variables_flightstates
from evaldata import calc_stat_varibales, plot_stats
from cleanwhitespaces import cleanWhitespaces

FPS = 60.
x = 0
y = 1
z = 2


def eval_logfile(drone_filepath):
	drone_data_string = cleanWhitespaces(drone_filepath)
	drone_data = pd.read_csv(drone_data_string, sep=';')

	n_samples_drone = np.size(drone_data,0)
	step_stats = []
	target = np.array([drone_data['target_pos_x'][0], drone_data['target_pos_y'][0], drone_data['target_pos_z'][0]])
	step_found = False
	step = init_step_data()
	flight = init_flightstate_data()

	for i in range(n_samples_drone):
		prev_target = target
		target = np.array([drone_data['target_pos_x'][i], drone_data['target_pos_y'][i], drone_data['target_pos_z'][i]])
		pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
		err = target - pos
		time = drone_data['elapsed'][i]
		nav_state = drone_data['nav_state'][i]
		auto_throttle = drone_data['autoThrottle'][i]

		[step_found, step, step_stats] = eval_control(prev_target,target, pos, err, time, step_found, nav_state, step, step_stats)		
		flight = eval_current_flightstate(flight, time, nav_state, auto_throttle)
	return step_stats, flight


if __name__ == "__main__":
	tic = time.time()
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/version-test/exampleTuning/')
		
	drone_filepath = folderpath + 'log.csv'
	print('\nEvaluate: ', folderpath)
	
	step_stats, flight = eval_logfile(drone_filepath)
	toc = time.time()
	print('Found '+str(len(step_stats))+' step responses.')
	# for step_stat in step_stats:
	# 	print(step_stat)
	# print(flight)	
	control_stats, control_stats_units = generate_eval_variables_control(step_stats)
	control_eval_stats = calc_stat_varibales(control_stats)
	plot_stats(control_eval_stats, control_stats_units, 'md')

	state_stats, state_stats_unit = generate_eval_variables_states([flight])
	print(state_stats)
	print(state_stats_unit)

	print('\nEval time: '+str(toc-tic))
	
