#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd 
from flightstates import init_flightstate_data, eval_current_flightstate, key_takeofftime 
from cleanwhitespaces import cleanWhitespaces


def concatenate_insectlogs(folderpath):
	insect_data = pd.DataFrame()
	for file in os.listdir(folderpath):
		if file.endswith('.csv') and file.startswith('log_itrk'):
			data_string = cleanWhitespaces(folderpath+'/'+file)	
			dataframe = pd.read_csv(data_string, sep=';')
			insect_data = pd.concat([insect_data, dataframe], ignore_index=True)
	return insect_data.sort_values(by=['RS_ID']).reset_index()

key_minimalerror = 'minimal_error'	
key_time_minimalerror = 'time_minimal_error'
key_minimalerror_untracked = 'minimal_error_untracked'	
key_time_minimalerror_untracked = 'time_minimal_error_untracked'
key_insecttracking = 'insect_tracking'
def read_hunt(folderpath):
	drone_filepath = folderpath+'/log.csv'
	drone_data_string = cleanWhitespaces(drone_filepath)
	drone_data = pd.read_csv(drone_data_string, sep=';')

	insect_data = concatenate_insectlogs(folderpath)

	insect_data = concatenate_insectlogs(folderpath)
	n_samples_drone = np.size(drone_data,0)
	n_samples_insect = np.size(insect_data, 0)
	
	closest_distance = np.inf
	time_at_closest_distance = np.nan
	closest_distance_untracked = np.inf
	time_at_closest_distance_untracked = np.nan
	samples_insect_tracked = 0
	flightstates_data = init_flightstate_data()
		
	for i in range(n_samples_insect):
		if(insect_data['n_frames_lost_insect'][i]==0):
			samples_insect_tracked += 1
	
	for i in range(n_samples_drone):
		time = drone_data['elapsed'][i]
		nav_state = drone_data['nav_state'][i]
		auto_throttle = drone_data['autoThrottle'][i]
		flightstates_data = eval_current_flightstate(flightstates_data, time, nav_state, auto_throttle)

		try:
			insect_idx = insect_data[insect_data['RS_ID']==drone_data['RS_ID'][i]].index[0]
			target = np.array([insect_data['posX_insect'][insect_idx], insect_data['posY_insect'][insect_idx], insect_data['posZ_insect'][insect_idx]])
			
			if(drone_data['valid'][i]==1):
				pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
				error = np.linalg.norm(target-pos)
				if(error<closest_distance_untracked):
					closest_distance_untracked = error
					time_at_closest_distance_untracked = drone_data['elapsed'][i]
				if(insect_data['n_frames_lost_insect'][insect_idx]==0):
					if(error<closest_distance):
						closest_distance = error
						time_at_closest_distance = drone_data['elapsed'][i]
			
		except IndexError:
			pass	
				
	hunt_data = {}
	hunt_data[key_minimalerror] = closest_distance
	hunt_data[key_time_minimalerror] = time_at_closest_distance - flightstates_data[key_takeofftime]
	hunt_data[key_minimalerror_untracked] = closest_distance_untracked
	hunt_data[key_time_minimalerror_untracked] = time_at_closest_distance_untracked - flightstates_data[key_takeofftime]
	hunt_data[key_insecttracking] = samples_insect_tracked/n_samples_insect
	return flightstates_data, hunt_data

def hunt_evaldata(hunt_data):
	hunt_evaldat = {}
	for key in hunt_data:
		if(not np.isnan(hunt_data[key]) and not np.isinf(hunt_data[key])):
			hunt_evaldat[key] = hunt_data[key]

	return hunt_evaldat

def hunt_evaldata_units():
	hunt_evaldata_units = {}
	hunt_evaldata_units[key_minimalerror] = 'm'
	hunt_evaldata_units[key_time_minimalerror] = 's'
	hunt_evaldata_units[key_minimalerror_untracked] = 'm'
	hunt_evaldata_units[key_time_minimalerror_untracked] = 's'
	hunt_evaldata_units[key_insecttracking] = 'ratio'

	return hunt_evaldata_units


if __name__ == "__main__":
	folderpath = '/home/ludwig/Documents/codes/pats/pc/build-vscode/logging'
	hunt_data = read_hunt(folderpath)
	print(hunt_evaldata(hunt_data))

