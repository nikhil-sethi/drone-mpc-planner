#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pandas as pd 
import numpy as np
from io import StringIO
import os
import pwd

FPS = 60

def evalFlight(folderpath, drone_data_string=None):
	if(not drone_data_string):
		drone_data = pd.read_csv(folderpath='log.csv', sep=';')
	else:
		drone_data = pd.read_csv(drone_data_string, sep=';')
		
	insect_data = concatenateInsectLogs(folderpath)
	n_samples_drone = np.size(drone_data,0)
	n_samples_insect = np.size(insect_data, 0)
	
	closest_distance = np.inf
	time_at_closest_distance = np.nan
	samples_insect_tracked = 0
	samples_drone_pos_valid = 0
	takeoff_time = np.inf
	reached_landing = False
	reached_landed = False
	takeoff_detected = False
		
	for i in range(n_samples_insect):
		if(insect_data['n_frames_lost_insect'][i]==0):
			samples_insect_tracked += 1
	
	for i in range(n_samples_drone):
		if(drone_data['valid'][i]):
			samples_drone_pos_valid += 1		
			
		if(drone_data['nav_state'][i]==22):
			reached_landing = True
			
		if(drone_data['nav_state'][i]==24):
			reached_landed = True
		
		if(drone_data['nav_state'][i]>=11 and drone_data['nav_state'][i]<25):
			takeoff_detected = True
		
		if(drone_data['autoThrottle'][i]>224 and np.isinf(takeoff_time)):
			takeoff_time = drone_data['elapsed'][i]

		try:
			insect_idx = insect_data[insect_data['RS_ID']==drone_data['RS_ID'][i]].index[0]
			target = np.array([insect_data['posX_insect'][insect_idx], insect_data['posY_insect'][insect_idx], insect_data['posZ_insect'][insect_idx]])
			
			
			if(drone_data['joyModeSwitch'][i]==2 and drone_data['n_frames_lost_drone'][i]==0 
				and drone_data['valid'][i]==1 and insect_data['n_frames_lost_insect'][insect_idx]==0):
				pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
				error = np.linalg.norm(target-pos)
				if(error<closest_distance):
#					print('target: '+str(target)+'@time: '+str(insect_data['time'][insect_idx])+'; RS_ID: ' + str(insect_data['RS_ID'][insect_idx]))
#					print('position: '+str(pos)+'@time: '+str(drone_data['elapsed'][i])+'; RS_ID: ' + str(drone_data['RS_ID'][i]))
					closest_distance = error
					time_at_closest_distance = drone_data['elapsed'][i]
			
		except IndexError:
			pass	
				
	print('takeoff detected: '+str(takeoff_detected))
	print('reached landing: '+str(reached_landing))
	print('reached landed: '+str(reached_landed))
	print('Drone tracked: ' + str(samples_drone_pos_valid) + ' frames')
	print('Insect(s) tracked: ' + str(samples_insect_tracked) + ' frames')
	print('Closest distance: ' + str(closest_distance) + ' m at time: ' + str(time_at_closest_distance) + ' sec, ' + str(time_at_closest_distance-takeoff_time) + ' sec after takeoff.')

def cleanWhitespaces(filepath):
	file = open(filepath, "r")
	datastring = file.read()
	file.close()
	datastring = datastring.replace(' ', '')
	datastring = StringIO(datastring)
	return datastring

def findInsectLogFile(folderpath):
	""" Find largest insect log file """
	log_file = ''
	log_size = 0
	
	for file in os.listdir(folderpath):
		if file.endswith('.csv') and file.startswith('log_'):
			filesize = os.stat(folderpath+file).st_size
			if(filesize>log_size):
				log_file = file
				log_size = filesize
			
	return log_file

def concatenateInsectLogs(folderpath):
	insect_data = pd.DataFrame()
	for file in os.listdir(folderpath):
		if file.endswith('.csv') and file.startswith('log_itrk'):
			data_string = cleanWhitespaces(folderpath+file)	
			dataframe = pd.read_csv(data_string, sep=';')
			insect_data = pd.concat([insect_data, dataframe], ignore_index=True)
	return insect_data.sort_values(by=['RS_ID']).reset_index()


def getUserName():
	return pwd.getpwuid(os.getuid())[0]

if __name__ == "__main__":
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		username = getUserName()
		folderpath =  r'./'
		
	drone_filepath = folderpath + 'log.csv'
	insect_filepath = folderpath + findInsectLogFile(folderpath)
		
#	try:
	drone_data_string = cleanWhitespaces(drone_filepath)
	insect_data_string = cleanWhitespaces(insect_filepath)
	evalFlight(folderpath, drone_data_string)
		
#	except:
#		print('Unknown error occured.')
