#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

key_takeofftime = 'takeoff_time'
key_landedtime = 'landed_time'
key_enterlandingtime = 'entering_landing_time'
key_crashed = 'chrashed'

def init_flightstate_data():
	return {key_takeofftime: np.nan,
			key_landedtime: np.nan,
			key_enterlandingtime: np.nan}

def eval_current_flightstate(flight, time, nav_state, auto_throttle):
	if(nav_state==22 and np.isnan(flight[key_enterlandingtime])):
		flight[key_enterlandingtime] = time
			
	if(nav_state==23 and np.isnan(flight[key_landedtime])):
		flight[key_landedtime] = time

	if(nav_state==9 and np.isnan(flight[key_takeofftime])):
		flight[key_takeofftime] = time
	return flight

key_missiontime = 'mission_time'
key_landingtime = 'landing_time'
def flightstates_evaldata(flightstates_data, crashed_after_landing):
	flightstates_evaldata = {}

	if(len(flightstates_data)):
		flight = flightstates_data
		flightstates_evaldata[key_crashed] = False
		if(len(flight)>0):
			if(crashed_after_landing or np.isnan(flight[key_landedtime])):
				flightstates_evaldata[key_crashed] = True
			if(not np.isnan(flight[key_takeofftime]) and not np.isnan(flight[key_enterlandingtime])):
				flightstates_evaldata[key_missiontime] = flight[key_enterlandingtime] - flight[key_takeofftime]
			if(not np.isnan(flight[key_enterlandingtime]) and not np.isnan(flight[key_landedtime])):
				flightstates_evaldata[key_landingtime] = flight[key_landedtime] - flight[key_enterlandingtime]

	return flightstates_evaldata

def flightstates_evaldata_units():
	flightstates_evaldata_unit = {}
	flightstates_evaldata_unit[key_crashed] = 1
	flightstates_evaldata_unit[key_missiontime] = 's'
	flightstates_evaldata_unit[key_landingtime] = 's'

	return flightstates_evaldata_unit

if __name__ == "__main__":
	pass