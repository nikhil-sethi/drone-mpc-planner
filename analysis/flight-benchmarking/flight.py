#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import xml.etree.ElementTree as et

from terminallog import read_terminalfile, terminal_evaldata
from dronelog import read_dronelog
from control import control_evaldata
from flightstates import flightstates_evaldata
from hunt import read_hunt, hunt_evaldata


STATIC_WAYPOINTS_FLIGHT = 0
DYNAMIC_WAYPOINTS_FLIGHT = 1
def flight_type(folderpath):
	patsfile_handle = open(folderpath+'/pats.xml', 'r')
	patsfile = patsfile_handle.read()
	patsfile_handle.close()
	patslines = patsfile.split('\n')

	for i in range(len(patslines)):
		key =  patslines[i].split('>')[0].split('=')[-1]
		if(key=='"flightplan"'):
			arg = patslines[i].split('>')[1].split('<')[0]
			if(arg=='../../xml/flightplans/tuning.xml'
				or arg == '../../xml/flightplans/simple_demo.xml'):
				return STATIC_WAYPOINTS_FLIGHT

	return -1

def detect_hunt_based_on_trkr_logs(folderpath):
	return os.path.isfile(folderpath+'/log_rtrkr1.csv') or os.path.isfile(folderpath+'/log_itrk1.csv')

def read_flight(folderpath):
	terminal_data = {}
	steps = []
	flightstate_data = {}
	hunt_data = {}
	terminal_data, replay_moth_based_on_terminal = read_terminalfile(folderpath+'/terminal.log')
	replay_moth_based_on_itrkrs = detect_hunt_based_on_trkr_logs(folderpath)
	
	replay_moth = replay_moth_based_on_terminal or replay_moth_based_on_itrkrs	
	print('Hunt detected:', replay_moth)

	if(replay_moth):
		flightstate_data, hunt_data = read_hunt(folderpath)
	else:
		if(flight_type(folderpath)==STATIC_WAYPOINTS_FLIGHT):
			dronelog_path = folderpath+'/log.csv'
			steps, flightstate_data = read_dronelog(dronelog_path)

	return terminal_data, steps, flightstate_data, hunt_data

def eval_flight_data(terminal_data, steps, flightstate_data, hunt_data):
	terminal_evaldat, crashed_after_landing = terminal_evaldata(terminal_data)
	flightstate_evaldat = flightstates_evaldata(flightstate_data, crashed_after_landing)
	control_evaldat = control_evaldata(steps)
	hunt_evaldat = hunt_evaldata(hunt_data)

	return terminal_evaldat, flightstate_evaldat, control_evaldat, hunt_evaldat

def read_and_eval_flight(folderpath):
	terminal_data, steps, flightstate_data, hunt_data = read_flight(folderpath)
	terminal_evaldat, flightstate_evaldat, control_evaldat, hunt_evaldat = eval_flight_data(terminal_data, steps, flightstate_data, hunt_data)
	return terminal_evaldat, flightstate_evaldat, control_evaldat, hunt_evaldat

if __name__ == "__main__":
	import sys
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('/home/ludwig/Documents/codes/pats/pc/build-vscode/logging')
	print(read_and_eval_flight(folderpath))
