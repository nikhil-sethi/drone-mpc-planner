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
	step_data, flightstate_data, terminal_data = read_flightset(folderpath)
	print(terminal_data)

	terinal_stats, termianl_stats_units, crashed_after_landing = generate_eval_variables_terminal(terminal_data)
	terminal_eval_stats = calc_stat_varibales(terinal_stats)

	flightstate_stats, flightstate_stats_units = generate_eval_variables_flightstates(flightstate_data, crashed_after_landing)
	flightstate_eval_stats = calc_stat_varibales(flightstate_stats)

	control_stats, control_stats_units = generate_eval_variables_control(step_data)
	control_eval_stats = calc_stat_varibales(control_stats)




	merged_stats = {**control_eval_stats, **flightstate_eval_stats, **terminal_eval_stats}
	merged_stats_units = {**control_stats_units, **flightstate_stats_units, **termianl_stats_units}
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
	
