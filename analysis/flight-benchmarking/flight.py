#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
from xml.etree.ElementTree import parse as xmlparse

from terminallog import read_terminalfile, terminal_evaldata, key_landinglocation
from tuningflight import read_tuningflight
from control import control_evaldata
from flightstates import flightstates_evaldata, key_takeofftime
from hunt import read_hunt, hunt_evaldata, key_time_minimalerror_untracked
from longrangeflight import read_longrangeflight, longrange_evaldata
from flighttable import Flighttable

TUNING_FLIGHT = 0
LONG_RANGE_FLIGHT = 1
UNKNOWN_FLIGHT = -1
key_flighttype = 'flight_type'
key_target = 'target'
def flight_type(folderpath):
	root = xmlparse(folderpath+'/flightplan.xml').getroot() 
	flightplan_name = list(root)[0].text

	if(flightplan_name=='Tuning' or flightplan_name=='demo'):
		return {key_flighttype: TUNING_FLIGHT}
	elif(flightplan_name=='Long_range_flight'):
		waypoint = list(list(list(root)[1])[1])
		x = float(waypoint[4].text)
		y = float(waypoint[5].text)
		z = float(waypoint[6].text)
		return {key_flighttype: LONG_RANGE_FLIGHT, key_target: np.array([x,y,z])}
	else:
		return {key_flighttype: UNKNOWN_FLIGHT}

def detect_hunt_based_on_trkr_logs(folderpath):
	return os.path.isfile(folderpath+'/log_rtrkr1.csv') or os.path.isfile(folderpath+'/log_itrk1.csv')

def read_flight(folderpath, logging_folder):
	log_data = {}
	terminal_data = {}
	steps = []
	flightstate_data = {}
	hunt_data = {}
	longrange_data = {}

	replay_moth_based_on_terminal = False
	log_data, terminal_data, replay_moth_based_on_terminal = read_terminalfile(folderpath+'/terminal.log')
	if(not logging_folder):
		folderpath += '/logging'

	replay_moth_based_on_itrkrs = detect_hunt_based_on_trkr_logs(folderpath)
	replay_moth = replay_moth_based_on_terminal or replay_moth_based_on_itrkrs	

	if(replay_moth):
		flightstate_data, hunt_data = read_hunt(folderpath)
	else:
		ftype = flight_type(folderpath)
		dronelog_path = folderpath+'/log.csv'
		if(ftype[key_flighttype]==TUNING_FLIGHT):
			steps, flightstate_data = read_tuningflight(dronelog_path)
		elif(ftype[key_flighttype]==LONG_RANGE_FLIGHT):
			longrange_data, flightstate_data = read_longrangeflight(dronelog_path, ftype[key_target])

	takeoff_detected = True
	if(key_takeofftime in flightstate_data and np.isnan(flightstate_data[key_takeofftime])):
		takeoff_detected = False

	if(takeoff_detected):
		return log_data, terminal_data, steps, flightstate_data, longrange_data, hunt_data
	else:
		return {}, {}, [], {}, {}, {}

def eval_flight_data(terminal_data, steps, flightstate_data, longrange_data, hunt_data):
	terminal_evaldat, crashed_after_landing = terminal_evaldata(terminal_data)
	flightstate_evaldat = flightstates_evaldata(flightstate_data, crashed_after_landing)
	control_evaldat = control_evaldata(steps)
	longrange_evaldat = longrange_evaldata(longrange_data)
	hunt_evaldat = hunt_evaldata(hunt_data)

	return terminal_evaldat, flightstate_evaldat, control_evaldat, longrange_evaldat, hunt_evaldat

def read_and_eval_flight(folderpath, logging_folder=True, flighttable=None):
	log_data, terminal_data, steps, flightstate_data, longrange_data, hunt_data = read_flight(folderpath, logging_folder)
	terminal_evaldat, flightstate_evaldat, control_evaldat, longrange_evaldat, hunt_evaldat = eval_flight_data(terminal_data, steps, flightstate_data, longrange_data, hunt_data)

	flight_summary = folderpath
	flight_summary += ': hunt: '+str(len(hunt_data)>0)
	takeoff_detected = key_takeofftime in flightstate_data and not np.isnan(flightstate_data[key_takeofftime])
	flight_summary += '; takeoff_detected: '+ str(takeoff_detected)
	landing_location_detected = key_landinglocation in terminal_data and len(terminal_data[key_landinglocation])==2
	flight_summary += '; landing-location-detected: '+str(landing_location_detected)
	if(key_time_minimalerror_untracked in hunt_data):
		time_minimial_error_untracked = hunt_data[key_time_minimalerror_untracked] 
		flight_summary += '; tmerror-ut: '+str(time_minimial_error_untracked)
	print(flight_summary)

	if(takeoff_detected and flighttable):
		flighttable.add(log_data, control_evaldat, flightstate_evaldat, terminal_evaldat, hunt_evaldat, longrange_evaldat)

	return terminal_evaldat, flightstate_evaldat, control_evaldat, longrange_evaldat, hunt_evaldat

if __name__ == "__main__":
	import sys
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('/home/ludwig/Downloads/pats_data/pats16/dl_20200521004449')
	print(read_and_eval_flight(folderpath, True))
