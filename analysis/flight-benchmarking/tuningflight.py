#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pandas as pd
import numpy as np
import os
import pwd
import time
from control import eval_control, init_step_data, control_evaldata
from flightstates import eval_current_flightstate, init_flightstate_data, flightstates_evaldata
from evaldata_generic import calc_stat_varibales, plot_stats
from cleanwhitespaces import cleanWhitespaces

FPS = 60.
x = 0
y = 1
z = 2

def read_tuningflight(drone_filepath):
	steps_data = []
	try:
		drone_data_string = cleanWhitespaces(drone_filepath)
		print('read_tuningflight:', drone_filepath)
		drone_data = pd.read_csv(drone_data_string, sep=';')

		n_samples_drone = np.size(drone_data,0)
		if(n_samples_drone>0):
			target = np.array([drone_data['target_pos_x'][0], drone_data['target_pos_y'][0], drone_data['target_pos_z'][0]])
			step_found = False
			step = init_step_data()
			flightstates_data = init_flightstate_data()

			for i in range(n_samples_drone):
				prev_target = target
				target = np.array([drone_data['target_pos_x'][i], drone_data['target_pos_y'][i], drone_data['target_pos_z'][i]])
				pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
				err = target - pos
				time = drone_data['elapsed'][i]
				nav_state = drone_data['nav_state'][i]
				auto_throttle = drone_data['autoThrottle'][i]

				[step_found, step, steps_data] = eval_control(prev_target,target, pos, err, time, step_found, nav_state, step, steps_data)
				flightstates_data = eval_current_flightstate(flightstates_data, time, nav_state, auto_throttle)
			return steps_data, flightstates_data
		else:
			return [], {}
	except Exception as e:
		print(e)
		return [], {}


if __name__ == "__main__":
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/logging/')

	drone_filepath = folderpath + 'log.csv'
	print('\nEvaluate: ', folderpath)

	tic = time.time()
	steps, flightstates_data = read_tuningflight(drone_filepath)
	toc = time.time()

	print('Found '+str(len(steps))+' step responses.')
	print(flightstates_data)
	print('\nEval time: '+str(toc-tic))
