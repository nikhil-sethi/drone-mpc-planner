#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
key_landinglocation = 'landing_location'
def read_terminalfile(terminal_filepath):
	terminalfile_handle = open(terminal_filepath, 'r')
	terminalfile = terminalfile_handle.read()
	terminalfile_handle.close()
	terminallines = terminalfile.split('\n')

	landing_locations = []
	for i in range(len(terminallines)):
		words = terminallines[i].split(' ')
		if(words[0]=='blink-location:'):
			landing_location = np.array(eval(words[1]+words[2]+words[3]))
			landing_locations.append(landing_location)

	terminal_data = {}
	terminal_data[key_landinglocation] = landing_locations
	return terminal_data

key_horizontallandingprecision = 'horizontal_landing_precision'
def generate_eval_variables_terminal(terminal_data):
	crashed_after_landing = []
	landings_precision = []

	for i in range(len(terminal_data)):
		if(len(terminal_data[i][key_landinglocation])==2):
			before = terminal_data[i][key_landinglocation][0]
			after = terminal_data[i][key_landinglocation][1]
			if(abs(before[1]-after[1])>0.05):
				crashed_after_landing.append(True)
			else:
				crashed_after_landing.append(False)
				before[1] = 0
				after[1] = 0
				landings_precision.append(np.linalg.norm(before-after)*1000.)
		else:
			crashed_after_landing.append(False)

	terminal_stats={}
	terminal_stats_unit={}
	terminal_stats[key_horizontallandingprecision] = landings_precision
	terminal_stats_unit[key_horizontallandingprecision] = 'mm'
	return terminal_stats, terminal_stats_unit, crashed_after_landing


if __name__ == "__main__":
	import os
	import sys
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/logging/')
	terminal_filepath = folderpath + 'terminal.log'

	terminal_data = read_terminalfile(terminal_filepath)
	print(terminal_data)
	terminal_stats, terminal_stats_unit, crashed_after_landing = generate_eval_variables_terminal([terminal_data])
	print(terminal_stats)
