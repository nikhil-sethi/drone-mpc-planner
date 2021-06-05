#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
from xml.etree.ElementTree import parse as xmlparse

from terminallog import read_terminalfile, terminal_evaldata, KEY_LANDINGLOC
from tuningflight import read_tuningflight
from control import control_evaldata
from flightstates import flightstates_evaldata, KEY_TAKEOFFTIME, KEY_CRASHED
from hunt import read_hunt, hunt_evaldata, KEY_TIME_MINIMALERROR_UNTRACKED
from longrangeflight import read_longrangeflight, longrange_evaldata
from flighttable import Flighttable

TUNING_FLIGHT = 0
LONG_RANGE_FLIGHT = 1
UNKNOWN_FLIGHT = -1
KEY_FLIGHTTYPE = 'flight_type'
KEY_TARGET = 'target'


def flight_type(folderpath):
    try:
        with open(folderpath + '/flightplan.xml') as xml_file:
            root = xmlparse(xml_file).getroot()
    except:
        print('Error evaluating flight-type of:', folderpath + '/flightplan.xml')
        return {KEY_FLIGHTTYPE: UNKNOWN_FLIGHT}
    flightplan_name = list(root)[0].text

    if flightplan_name == 'Tuning' or flightplan_name == 'demo':
        return {KEY_FLIGHTTYPE: TUNING_FLIGHT}
    elif flightplan_name == 'Long_range_flight':
        waypoint = list(list(list(root)[1])[1])
        x = float(waypoint[4].text)
        y = float(waypoint[5].text)
        z = float(waypoint[6].text)
        return {KEY_FLIGHTTYPE: LONG_RANGE_FLIGHT, KEY_TARGET: np.array([x, y, z])}
    else:
        return {KEY_FLIGHTTYPE: UNKNOWN_FLIGHT}


def detect_hunt_based_on_trkr_logs(folderpath):
    return os.path.isfile(folderpath + '/log_rtrkr1.csv') or os.path.isfile(folderpath + '/log_itrk1.csv')


def read_flight(folderpath, logging_folder):
    log_data = {}
    terminal_data = {}
    steps = []
    flightstate_data = {}
    hunt_data = {}
    longrange_data = {}

    replay_moth_based_on_terminal = False
    log_data, terminal_data, replay_moth_based_on_terminal = read_terminalfile(folderpath + '/terminal.log')
    if not logging_folder:
        folderpath += '/logging'

    replay_moth_based_on_itrkrs = detect_hunt_based_on_trkr_logs(folderpath)
    replay_moth = replay_moth_based_on_terminal or replay_moth_based_on_itrkrs

    if replay_moth:
        flightstate_data, hunt_data = read_hunt(folderpath)
    else:
        ftype = flight_type(folderpath)
        dronelog_path = folderpath + '/log.csv'
        if ftype[KEY_FLIGHTTYPE] == TUNING_FLIGHT:
            steps, flightstate_data = read_tuningflight(dronelog_path)
        elif ftype[KEY_FLIGHTTYPE] == LONG_RANGE_FLIGHT:
            longrange_data, flightstate_data = read_longrangeflight(dronelog_path, ftype[KEY_TARGET])

    takeoff_detected = True
    if KEY_TAKEOFFTIME in flightstate_data and np.isnan(flightstate_data[KEY_TAKEOFFTIME]):
        takeoff_detected = False

    if takeoff_detected:
        return log_data, terminal_data, steps, flightstate_data, longrange_data, hunt_data
    else:
        return {}, {}, [], {}, {}, {}


def eval_flight_data(terminal_data, steps, flightstate_data, longrange_data, hunt_data):
    terminal_evaldat, crashed_after_landing = terminal_evaldata(terminal_data)
    flightstate_evaldat = flightstates_evaldata(flightstate_data, crashed_after_landing)
    control_evaldat = control_evaldata(steps)
    longrange_evaldat = longrange_evaldata(longrange_data)
    hunt_evaldat = hunt_evaldata(hunt_data)

    return terminal_evaldat, flightstate_evaldat, control_evaldat, longrange_evaldat, hunt_evaldat


def print_flightsummary(folderpath, takeoff_detected, flightstate_data, flightstate_evaldat, hunt_data):
    flight_summary = folderpath
    flight_summary += ': hunt: ' + str(len(hunt_data) > 0)
    flight_summary += '; takeoff_detected: ' + str(takeoff_detected)
    #landing_location_detected = key_landinglocation in terminal_data and len(terminal_data[key_landinglocation])==2
    #flight_summary += '; landing-location-detected: '+str(landing_location_detected)
    crash_detected = '; crashed: ' + str(False)
    if KEY_CRASHED in flightstate_evaldat:
        crash_detected = '; crashed: ' + str(flightstate_evaldat[KEY_CRASHED])
    flight_summary += crash_detected
    # if(key_time_minimalerror_untracked in hunt_data):
    #time_minimial_error_untracked = hunt_data[key_time_minimalerror_untracked]
    #flight_summary += '; tmerror-ut: '+str(time_minimial_error_untracked)
    print(flight_summary)


KEY_TAKEOFF_DETECT = 'takeoff-detected'
KEY_LOGDATA = 'log-data'
KEY_TERMINAL_EDAT = 'terminal-eval-data'
KEY_FS_EDAT = 'flightstates-eval-data'
KEY_CONTROL_EDAT = 'control-eval-data'
KEY_LR_EDAT = 'longrange-eval-data'
KEY_HUNT_EDAT = 'hunt-eval-data'


def read_and_eval_flight(folderpath, logging_folder=True):
    log_data, terminal_data, steps, flightstate_data, longrange_data, hunt_data = read_flight(folderpath, logging_folder)
    terminal_evaldat, flightstate_evaldat, control_evaldat, longrange_evaldat, hunt_evaldat = eval_flight_data(terminal_data, steps, flightstate_data, longrange_data, hunt_data)
    takeoff_detected = KEY_TAKEOFFTIME in flightstate_data and not np.isnan(flightstate_data[KEY_TAKEOFFTIME])

    print_flightsummary(folderpath, takeoff_detected, flightstate_data, flightstate_evaldat, hunt_data)

    retobj = {}
    retobj[KEY_TAKEOFF_DETECT] = takeoff_detected
    retobj[KEY_LOGDATA] = log_data
    retobj[KEY_TERMINAL_EDAT] = terminal_evaldat
    retobj[KEY_FS_EDAT] = flightstate_evaldat
    retobj[KEY_CONTROL_EDAT] = control_evaldat
    retobj[KEY_LR_EDAT] = longrange_evaldat
    retobj[KEY_HUNT_EDAT] = hunt_evaldat
    return retobj


if __name__ == "__main__":
    import sys
    if(len(sys.argv) >= 2):
        folderpath = str(sys.argv[1])
    else:
        folderpath = os.path.expanduser('/home/ludwig/Downloads/pats_data/pats16/dl_20200521004449')
    print(read_and_eval_flight(folderpath, True))
