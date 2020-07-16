#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
from flightstates import init_flightstate_data, eval_current_flightstate, KEY_TAKEOFFTIME
from cleanwhitespaces import cleanWhitespaces


def concatenate_insectlogs(folderpath):
	insect_data = pd.DataFrame()
	for file in os.listdir(folderpath):
		if file.endswith('.csv') and file.startswith('log_itrk'):
			data_string = cleanWhitespaces(folderpath+'/'+file)
			dataframe = pd.read_csv(data_string, sep=';')
			insect_data = pd.concat([insect_data, dataframe], ignore_index=True)

	replayinsect_data = pd.DataFrame()
	n_replaylogs = 0
	for file in os.listdir(folderpath):
		if file.endswith('.csv') and file.startswith('log_rtrk'):
			data_string = cleanWhitespaces(folderpath+'/'+file)
			dataframe = pd.read_csv(data_string, sep=';')
			replayinsect_data = pd.concat([replayinsect_data, dataframe], ignore_index=True)
			n_replaylogs += 1

	key_nlostframes = 'n_frames_lost_insect'
	key_posxtarget = 'posX_insect'
	key_posytarget = 'posY_insect'
	key_posztarget = 'posZ_insect'
	if n_replaylogs > 0:
		key_nlostframes = 'n_frames_lost_replay'
		key_posxtarget = 'posX_replay'
		key_posytarget = 'posY_replay'
		key_posztarget = 'posZ_replay'
		return replayinsect_data.sort_values(by=['RS_ID']).reset_index(), key_nlostframes, key_posxtarget, key_posytarget, key_posztarget

	return insect_data.sort_values(by=['RS_ID']).reset_index(), key_nlostframes, key_posxtarget, key_posytarget, key_posztarget


KEY_MINIMALERROR = 'minimal_error'
KEY_TIME_MINIMALERROR = 'time_minimal_error'
KET_MINIMALERROR_UNTRACKED = 'minimal_error_untracked'
KEY_TIME_MINIMALERROR_UNTRACKED = 'time_minimal_error_untracked'
KEY_INSECTTRACKING = 'insect_tracking'
def read_hunt(folderpath):
	drone_filepath = folderpath+'/log.csv'
	drone_data_string = cleanWhitespaces(drone_filepath)
	drone_data = pd.read_csv(drone_data_string, sep=';')

	insect_data, key_nlostframes, key_posxtarget, key_posytarget, key_posztarget = concatenate_insectlogs(folderpath)

	n_samples_drone = np.size(drone_data,0)
	n_samples_insect = np.size(insect_data, 0)

	closest_distance = np.inf
	time_at_closest_distance = np.nan
	closest_distance_untracked = np.inf
	time_at_closest_distance_untracked = np.nan
	samples_insect_tracked = 0
	flightstates_data = init_flightstate_data()

	for i in range(n_samples_insect):
		if insect_data[key_nlostframes][i] == 0:
			samples_insect_tracked += 1

	first_valid = np.max([np.where(drone_data['valid'] == 1)[0][0] - 2, 1])
	for i in range(first_valid, n_samples_drone):
		time = drone_data['elapsed'][i]
		nav_state = drone_data['nav_state'][i]
		auto_throttle = drone_data['autoThrottle'][i]
		flightstates_data = eval_current_flightstate(flightstates_data, time, nav_state, auto_throttle)

		try:
			insect_idx = insect_data[insect_data['RS_ID'] == drone_data['RS_ID'][i]].index[0]
			target = np.array([insect_data[key_posxtarget][insect_idx], insect_data[key_posytarget][insect_idx], insect_data[key_posztarget][insect_idx]])

			if drone_data['valid'][i] == 1:
				pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
				error = np.linalg.norm(target-pos)

				if error < closest_distance_untracked:
					# print(str(insect_data['RS_ID'][insect_idx])+':'+str(error)+'@'+str(drone_data['elapsed'][i]))
					closest_distance_untracked = error
					time_at_closest_distance_untracked = drone_data['elapsed'][i]
				if insect_data[key_nlostframes][insect_idx] == 0:
					if error < closest_distance:
						closest_distance = error
						time_at_closest_distance = drone_data['elapsed'][i]

		except IndexError:
			pass

	hunt_data = {}
	hunt_data[KEY_MINIMALERROR] = closest_distance
	hunt_data[KEY_TIME_MINIMALERROR] = time_at_closest_distance - flightstates_data[KEY_TAKEOFFTIME]
	hunt_data[KET_MINIMALERROR_UNTRACKED] = closest_distance_untracked
	hunt_data[KEY_TIME_MINIMALERROR_UNTRACKED] = time_at_closest_distance_untracked - flightstates_data[KEY_TAKEOFFTIME]
	hunt_data[KEY_INSECTTRACKING] = samples_insect_tracked/n_samples_insect*100.
	return flightstates_data, hunt_data

def hunt_evaldata(hunt_data):
	hunt_evaldat = {}
	for key in hunt_data:
		if(not np.isnan(hunt_data[key]) and not np.isinf(hunt_data[key])):
			hunt_evaldat[key] = hunt_data[key]

	return hunt_evaldat

def hunt_evaldata_units():
	hunt_evaldata_units = {}
	hunt_evaldata_units[KEY_MINIMALERROR] = 'm'
	hunt_evaldata_units[KEY_TIME_MINIMALERROR] = 's'
	hunt_evaldata_units[KET_MINIMALERROR_UNTRACKED] = 'm'
	hunt_evaldata_units[KEY_TIME_MINIMALERROR_UNTRACKED] = 's'
	hunt_evaldata_units[KEY_INSECTTRACKING] = '%'

	return hunt_evaldata_units


if __name__ == "__main__":
	import time as t
	folderpath = '/home/pats/pc/pc/build-vscode/logging'
	tic = t.time()
	flight_data, hunt_data = read_hunt(folderpath)
	toc = t.time()
	print('Evaluation time:', toc-tic)
	print(hunt_evaldata(hunt_data))

