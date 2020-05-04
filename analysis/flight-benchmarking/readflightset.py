#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
from evallogfile import eval_logfile
from evalterminalfile import read_terminalfile

def read_flightset(folderpath):
	flights_folderpaths = [f.path for f in os.scandir(folderpath) if f.is_dir()]
	step_stats = []
	flight_stats = []
	terminal_stats = []

	for flight_folderpath in flights_folderpaths:
		if(os.path.isfile(flight_folderpath+'/log.csv')):	
			drone_filepath = flight_folderpath+'/log.csv'
			steps, flight = eval_logfile(drone_filepath)
			print(drone_filepath+':'+' N-steps: ',len(steps))
			step_stats += steps
			flight_stats += [flight]

			terminal_filepath = flight_folderpath+'/terminal.log'
			terminal = read_terminalfile(terminal_filepath)
			terminal_stats += [terminal]

	return step_stats, flight_stats, terminal_stats

if __name__ == "__main__":
	tic = time.time()
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/version-test/')

	step_stats, flight_stats = read_flightset(folderpath)
	print('N steps:', len(step_stats))
	print('N flights:', len(flight_stats))

	