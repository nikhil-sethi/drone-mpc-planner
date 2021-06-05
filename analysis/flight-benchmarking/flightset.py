#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import resource
import multiprocessing
import numpy as np
from evaldata_generic import calc_stat_varibales, plot_stats
from flight import read_and_eval_flight, KEY_TAKEOFF_DETECT, KEY_LOGDATA, KEY_TERMINAL_EDAT
from flight import KEY_FS_EDAT, KEY_CONTROL_EDAT, KEY_LR_EDAT, KEY_HUNT_EDAT
from control import control_evaldata_units
from terminallog import terminal_evaldata_units
from flightstates import flightstates_evaldata_units
from longrangeflight import longrange_evaldata_units
from hunt import hunt_evaldata_units
from flighttable import Flighttable


ENABLE_MULTITHREADING = False
MAX_PROCESSES = int(multiprocessing.cpu_count() * 3 / 4)
if ENABLE_MULTITHREADING:
    print("Max number of processes:", MAX_PROCESSES)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (hard, hard))


def combine_evaldata(collection, sample):
    for key in sample:
        if key in collection:
            if isinstance(sample[key], list):
                collection[key] += sample[key]
            else:
                collection[key].append(sample[key])
        else:
            if isinstance(sample[key], list):
                collection[key] = sample[key]
            else:
                collection[key] = [sample[key]]


def next_path(folderpath_list):
    current_path = folderpath_list.pop(0)
    subfolders = [f.path for f in os.scandir(current_path) if f.is_dir()]
    N_subfolders = len(subfolders)

    terminalfile = os.path.isfile(current_path + '/terminal.log')
    logfile = os.path.isfile(current_path + '/log.csv')
    if terminalfile:
        logfile = os.path.isfile(current_path + '/logging/log.csv')

    if(N_subfolders > 0 and not terminalfile):
        folderpath_list += subfolders

    logging_folder = True  # current_path is a logging folder which includes the log.csv but not the terminal.log
    if terminalfile:
        logging_folder = False

    return current_path, folderpath_list, logging_folder, logfile


def read_and_eval_flight_process_interface(current_path, logging_folder, i, proc_ret_data):
    ret_data = read_and_eval_flight(current_path, logging_folder)
    proc_ret_data[i] = ret_data


def read_flightset_rec(folderpath_list, evaldata, flighttable=None):
    if not folderpath_list:
        return evaldata

    if ENABLE_MULTITHREADING and len(folderpath_list) > 1:
        manager = multiprocessing.Manager()
        procs_ret_data = manager.dict()
        jobs = []
        n_processes = 0

        for i in range(np.min([MAX_PROCESSES, len(folderpath_list)])):
            current_path, folderpath_list, logging_folder, logfile = next_path(folderpath_list)
            if logfile:
                p = multiprocessing.Process(target=read_and_eval_flight_process_interface,
                                            args=(current_path, logging_folder, i, procs_ret_data))
                p.start()
                jobs.append(p)
                n_processes += 1

        for i in range(n_processes):
            jobs[i].join()
            jobs[i].terminate()

            if procs_ret_data and i in procs_ret_data:
                combine_evaldata(evaldata[KEY_TERMINAL_EDAT], procs_ret_data[i][KEY_TERMINAL_EDAT])
                combine_evaldata(evaldata[KEY_FS_EDAT], procs_ret_data[i][KEY_FS_EDAT])
                combine_evaldata(evaldata[KEY_CONTROL_EDAT], procs_ret_data[i][KEY_CONTROL_EDAT])
                combine_evaldata(evaldata[KEY_LR_EDAT], procs_ret_data[i][KEY_LR_EDAT])
                combine_evaldata(evaldata[KEY_HUNT_EDAT], procs_ret_data[i][KEY_HUNT_EDAT])

                if flighttable and procs_ret_data[i][KEY_TAKEOFF_DETECT]:
                    flighttable.add(procs_ret_data[i][KEY_LOGDATA],
                                    procs_ret_data[i][KEY_CONTROL_EDAT],
                                    procs_ret_data[i][KEY_FS_EDAT],
                                    procs_ret_data[i][KEY_TERMINAL_EDAT],
                                    procs_ret_data[i][KEY_HUNT_EDAT],
                                    procs_ret_data[i][KEY_LR_EDAT])

    else:
        current_path, folderpath_list, logging_folder, logfile = next_path(folderpath_list)
        if logfile:
            ret_data = read_and_eval_flight(current_path, logging_folder)
            combine_evaldata(evaldata[KEY_CONTROL_EDAT], ret_data[KEY_CONTROL_EDAT])
            combine_evaldata(evaldata[KEY_FS_EDAT], ret_data[KEY_FS_EDAT])
            combine_evaldata(evaldata[KEY_TERMINAL_EDAT], ret_data[KEY_TERMINAL_EDAT])
            combine_evaldata(evaldata[KEY_HUNT_EDAT], ret_data[KEY_HUNT_EDAT])
            combine_evaldata(evaldata[KEY_LR_EDAT], ret_data[KEY_LR_EDAT])

            if flighttable and ret_data[KEY_TAKEOFF_DETECT]:
                flighttable.add(ret_data[KEY_LOGDATA],
                                ret_data[KEY_CONTROL_EDAT],
                                ret_data[KEY_FS_EDAT],
                                ret_data[KEY_TERMINAL_EDAT],
                                ret_data[KEY_HUNT_EDAT],
                                ret_data[KEY_LR_EDAT])

    return read_flightset_rec(folderpath_list, evaldata, flighttable)


def read_flightset(folderpath, flighttable=None):
    sys.setrecursionlimit(100000)  # default is 1000
    folderpath_list = [folderpath]
    evaldata = {}
    evaldata[KEY_TERMINAL_EDAT] = {}
    evaldata[KEY_FS_EDAT] = {}
    evaldata[KEY_CONTROL_EDAT] = {}
    evaldata[KEY_LR_EDAT] = {}
    evaldata[KEY_HUNT_EDAT] = {}
    return read_flightset_rec(folderpath_list, evaldata, flighttable)


def read_and_eval_flightset(folderpath, flighttable=None):
    evaldata = read_flightset(folderpath, flighttable)
    merged_evaldata = {**evaldata[KEY_CONTROL_EDAT], **evaldata[KEY_FS_EDAT],
                       **evaldata[KEY_TERMINAL_EDAT], **evaldata[KEY_LR_EDAT],
                       **evaldata[KEY_HUNT_EDAT]}

    step_evaldat_units = control_evaldata_units()
    flightstates_evaldat_units = flightstates_evaldata_units()
    terminal_evaldat_units = terminal_evaldata_units()
    longrange_evaldat_units = longrange_evaldata_units()
    hunt_evaldat_units = hunt_evaldata_units()
    merged_evaldata_units = {**step_evaldat_units, **flightstates_evaldat_units,
                             **terminal_evaldat_units, **longrange_evaldat_units,
                             **hunt_evaldat_units}

    merged_evalstats = calc_stat_varibales(merged_evaldata)

    plot_stats(merged_evalstats, merged_evaldata_units, 'md')


if __name__ == "__main__":

    tic = time.time()
    if len(sys.argv) >= 2:
        folderpath = str(sys.argv[1])
    else:
        folderpath = os.path.expanduser('/home/ludwig/Documents/2020-06 testing-koppert/old_hunting')

    flighttable = Flighttable()
    tic = time.time()
    read_and_eval_flightset(folderpath, flighttable)
    toc = time.time()
    print('Eval-time: ', toc - tic)

    with open('flightstable.csv', 'w') as output_handle:
        output_handle.write(flighttable.tablecontent.replace('/t', ';'))
