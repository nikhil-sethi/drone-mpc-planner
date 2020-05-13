#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
key_landinglocation = 'landing_location'
def read_terminalfile(terminal_filepath):
	""" @Missing: Catch case terminal.log is not existing """
	try:
		terminalfile_handle = open(terminal_filepath, 'r')
		terminalfile = terminalfile_handle.read()
		terminalfile_handle.close()
		terminallines = terminalfile.split('\n')
	except:
		return {}, False

	landing_locations = []
	replay_moth = False
	for i in range(len(terminallines)):
		words = terminallines[i].split(' ')
		if(words[0]=='blink-location:'):
			landing_location = np.array(eval(words[1]+words[2]+words[3]))
			landing_locations.append(landing_location)
		if(len(words)>= 4 and words[0]+words[1]+words[2]=='Openinginsectlog:'):
			replay_moth = True

	terminal_data = {}
	terminal_data[key_landinglocation] = landing_locations
	return terminal_data, replay_moth

key_horizontallandingprecision = 'horizontal_landing_precision'
def terminal_evaldata(terminal_data):
	terminal_evaldata={}
	crashed_after_landing = False

	if(len(terminal_data)>0):
		if(len(terminal_data[key_landinglocation])==2):
			before = terminal_data[key_landinglocation][0]
			after = terminal_data[key_landinglocation][1]
			if(abs(before[1]-after[1])>0.05):
				crashed_after_landing = True
			else:
				before[1] = 0
				after[1] = 0
				terminal_evaldata[key_horizontallandingprecision] = np.linalg.norm(before-after)*1000.

	return terminal_evaldata, crashed_after_landing

def terminal_evaldata_units():
	terminal_evaldata_unit={}
	terminal_evaldata_unit[key_horizontallandingprecision] = 'mm'
	return terminal_evaldata_unit

if __name__ == "__main__":
	import os
	import sys
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/logging/')
	terminal_filepath = folderpath + 'terminal.log'

	terminal_data, replay_moth = read_terminalfile(terminal_filepath)
	print(terminal_data)
	print(replay_moth)
	terminal_stats, crashed_after_landing = terminal_evaldata(terminal_data)
	print(terminal_stats)
