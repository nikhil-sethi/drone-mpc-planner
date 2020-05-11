#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
from evallogfile import eval_logfile
from evalterminalfile import read_terminalfile

def read_flightset_rec(folderpath_list, step_data, flight_data, terminal_data):
	if(len(folderpath_list)==0):
		return [step_data, flight_data, terminal_data]

	current_path = folderpath_list.pop(0)
	subfolders = [f.path for f in os.scandir(current_path) if f.is_dir()]
	logfile = os.path.isfile(current_path+'/log.csv')
	N_subfolders = len(subfolders)

	if(N_subfolders>0):
		folderpath_list += subfolders	

	if(logfile):
		drone_filepath = current_path+'/log.csv'
		steps, flight = eval_logfile(drone_filepath)
		print(drone_filepath+':'+' N-steps: ',len(steps))
		step_data += steps
		flight_data += [flight]

		try:
			terminal_filepath = current_path+'/terminal.log'
			terminal = read_terminalfile(terminal_filepath)
			terminal_data += [terminal]
		except:
			print(terminal_filepath, 'not found.')

	return read_flightset_rec(folderpath_list, step_data, flight_data, terminal_data)



def read_flightset(folderpath):
	folderpath_list = [folderpath]
	step_data = []
	flight_data = []
	terminal_data = []

	return read_flightset_rec(folderpath_list, step_data, flight_data, terminal_data)

if __name__ == "__main__":
	tic = time.time()
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('/home/ludwig/Downloads/perf_an-20200512T085620Z-001/perf_an')

	[step_data, flight_data, terminal_data] = read_flightset(folderpath)
	print(step_data)
	print(flight_data)
	print(terminal_data)

	