#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
from evaldata_generic import calc_stat_varibales, plot_stats
from flight import read_and_eval_flight
from control import control_evaldata_units
from terminallog import terminal_evaldata_units
from flightstates import flightstates_evaldata_units
from longrangeflight import longrange_evaldata_units
from hunt import hunt_evaldata_units
from flighttable import Flighttable

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

def read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata, flighttable=None):
	if(len(folderpath_list)==0):
		if(flighttable):
			return [step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata]
		return [step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata]

	current_path = folderpath_list.pop(0)
	subfolders = [f.path for f in os.scandir(current_path) if f.is_dir()]
	N_subfolders = len(subfolders)

	terminalfile = os.path.isfile(current_path+'/terminal.log')
	logfile = os.path.isfile(current_path+'/log.csv')
	if(terminalfile):
		logfile = os.path.isfile(current_path+'/logging/log.csv')

	if(N_subfolders>0 and not terminalfile):
		folderpath_list += subfolders	

	logging_folder = True # current_path is a logging folder which includes the log.csv but not the terminal.log	
	if(terminalfile):
		logging_folder = False

	if(logfile):
		ter_edat, fs_edat, step_edat, lr_edat, hunt_edat = read_and_eval_flight(current_path, logging_folder, flighttable)
		combine_evaldata(step_evaldata, step_edat)
		combine_evaldata(flightstates_evaldata, fs_edat)
		combine_evaldata(terminal_evaldata, ter_edat)
		combine_evaldata(hunt_evaldata, hunt_edat)
		combine_evaldata(longrange_evaldata, lr_edat)

	return read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata, flighttable)



def read_flightset(folderpath, flighttable=None):
	sys.setrecursionlimit(100000) #default is 1000
	folderpath_list = [folderpath]
	step_evaldata = {}
	flightstates_evaldata = {}
	terminal_evaldata = {}
	longrange_evaldata = {}
	hunt_evaldata = {}

	return read_flightset_rec(folderpath_list, step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata, flighttable)

def read_and_eval_flightset(folderpath, flgihttable=None):
	step_evaldata, flightstates_evaldata, terminal_evaldata, longrange_evaldata, hunt_evaldata = read_flightset(folderpath, flighttable)
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
		folderpath =  os.path.expanduser('~/Downloads/pats_data')

	flighttable = Flighttable()
	tic = time.time()
	read_and_eval_flightset(folderpath, flighttable)
	toc = time.time()
	print('Eval-time: ', toc-tic)

	output_handle = open('flightstable.csv', 'w')
	output_handle.write(flighttable.tablecontent.replace('/t', ';'))
	output_handle.close()