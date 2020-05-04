#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
from readflightset import read_flightset
from evalcontrol import generate_eval_variables_control
from evalflightstates import generate_eval_variables_flightstates, key_crashed
from evalterminalfile import generate_eval_variables_terminal
from evaldata import calc_stat_varibales, plot_stats

def eval_flightset(folderpath):
	step_stats, flight_stats, terminal_stats = read_flightset(folderpath)

	control_stats, control_stats_units = generate_eval_variables_control(step_stats)
	control_eval_stats = calc_stat_varibales(control_stats)

	state_stats, state_stats_units = generate_eval_variables_flightstates(flight_stats)
	state_eval_stats = calc_stat_varibales(state_stats)

	terinal_stats, termianl_stats_units, crashed_after_landing = generate_eval_variables_terminal(terminal_stats)
	terminal_eval_stats = calc_stat_varibales(terinal_stats)

	if(crashed_after_landing):
		state_eval_stats[key_crashed] = True 

	merged_stats = {**control_eval_stats, **state_eval_stats, **terminal_eval_stats}
	merged_stats_units = {**control_stats_units, **state_stats_units, **termianl_stats_units}
	plot_stats(merged_stats, merged_stats_units, 'md')

	return 1

if __name__ == "__main__":

	tic = time.time()
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('/home/ludwig/Downloads/pats_data/pats14')

	tic = time.time()
	eval_flightset(folderpath)
	toc = time.time()
	print('Eval-time: ', toc-tic)
	
