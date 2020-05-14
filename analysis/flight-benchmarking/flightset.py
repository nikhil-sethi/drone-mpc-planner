#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
from evaldata_generic import calc_stat_varibales, plot_stats
from flight import read_and_eval_flight
from control import control_evaldata_units
from terminallog import terminal_evaldata_units
from flightstates import flightstates_evaldata_units
from longrangeflight import longrange_evaldata_units
from hunt import hunt_evaldata_units

def combine_evaldata(collection, sample):
	for key in sample:
		if key in collection:
			if(isinstance(sample[key], list)):
				collection[key] += sample[key]
			else:
				collection[key].append(sample[key])
		else:
			if(isinstance(sample[key], list)):
				collection[key] = sample[key]
			else:
				collection[key] = [sample[key]]

def read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata):
	if(len(folderpath_list)==0):
		return [step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata]

	current_path = folderpath_list.pop(0)
	subfolders = [f.path for f in os.scandir(current_path) if f.is_dir()]
	logfile = os.path.isfile(current_path+'/log.csv')
	N_subfolders = len(subfolders)

	if(N_subfolders>0):
		folderpath_list += subfolders	

	if(logfile):
		ter_edat, fs_edat, step_edat, lr_edat, hunt_edat = read_and_eval_flight(current_path)
		combine_evaldata(step_evaldata, step_edat)
		combine_evaldata(flightstates_evaldata, fs_edat)
		combine_evaldata(terminal_evaldata, ter_edat)
		combine_evaldata(hunt_evaldata, hunt_edat)
		combine_evaldata(longrange_evaldata, lr_edat)

	return read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata)



def read_flightset(folderpath):
	folderpath_list = [folderpath]
	step_evaldata = {}
	flightstates_evaldata = {}
	terminal_evaldata = {}
	longrange_evaldata = {}
	hunt_evaldata = {}

	return read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata)

def read_and_eval_flightset(folderpath):
	step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata = read_flightset(folderpath)
	merged_evaldata = {**step_evaldata, **flightstates_evaldata, **terminal_evaldata, **longrange_evaldata, **hunt_evaldata}

	step_evaldat_units = control_evaldata_units()
	flightstates_evaldat_units = flightstates_evaldata_units()
	terminal_evaldat_units = terminal_evaldata_units()
	longrange_evaldat_units = longrange_evaldata_units()
	hunt_evaldat_units= hunt_evaldata_units()
	merged_evaldata_units = {**step_evaldat_units, **flightstates_evaldat_units, **terminal_evaldat_units, **longrange_evaldat_units, **hunt_evaldat_units}

	merged_evalstats = calc_stat_varibales(merged_evaldata)

	plot_stats(merged_evalstats, merged_evaldata_units, 'md')

if __name__ == "__main__":

	tic = time.time()
	if(len(sys.argv)>=2):
		folderpath = str(sys.argv[1])
	else:
		folderpath =  os.path.expanduser('~/code/pats/pc/build-vscode/logging')

	tic = time.time()
	read_and_eval_flightset(folderpath)
	toc = time.time()
	print('Eval-time: ', toc-tic)
