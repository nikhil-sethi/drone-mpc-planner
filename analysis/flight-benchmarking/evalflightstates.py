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

	if(auto_throttle>224 and np.isnan(flight[key_takeofftime])):
		flight[key_takeofftime] = time
	return flight

key_missiontime = 'mission_time'
key_landingtime = 'landing_time'
def generate_eval_variables_flightstates(flights, crashed_after_landing):
	crashed = []
	mission_time =[]
	landing_time = []

	for i in range(len(flights)):
		flight = flights[i]
		print(flight)
		print(crashed_after_landing)
		print(i)
		if(np.isnan(flight[key_landedtime]) or crashed_after_landing[i]):
			crashed.append(True)
		else:
			crashed.append(False)
		if(not np.isnan(flight[key_takeofftime]) and not np.isnan(flight[key_enterlandingtime])):
			mission_time.append(flight[key_enterlandingtime] - flight[key_takeofftime])
		if(not np.isnan(flight[key_enterlandingtime]) and not np.isnan(flight[key_landedtime])):
			landing_time.append(flight[key_landedtime] - flight[key_enterlandingtime])

	flight_stats = {}
	flight_stats_unit = {}
	flight_stats[key_crashed] = crashed
	flight_stats_unit[key_crashed] = 1
	flight_stats[key_missiontime] = mission_time
	flight_stats_unit[key_missiontime] = 's'
	flight_stats[key_landingtime] = landing_time
	flight_stats_unit[key_landingtime] = 's'

	return flight_stats, flight_stats_unit
if __name__ == "__main__":
	pass