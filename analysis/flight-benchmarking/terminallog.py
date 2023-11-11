#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


def get_system(path):
    system = 'unknown'
    path_elements = path.split('/')
    for i in range(len(path_elements)):
        segments = path_elements[i].split('s')
        if(len(segments) == 2 and segments[0] == 'pat'):
            system = path_elements[i]

    return system


def get_foldername(terminal_filepath):
    path_elements = terminal_filepath.split('/')
    while "" in path_elements:
        path_elements.remove("")
    return path_elements[-2]


KEY_LANDINGLOC = 'landing_location'
KEY_SYSTEM = 'system'
KEY_DATE = 'date'
KEY_DATETIME = 'datetime'
KET_FOLDERNAME = 'foldername'


def read_terminalfile(terminal_filepath):
    """ @Missing: Catch case terminal.log is not existing """
    try:
        with open(terminal_filepath, 'r') as terminalfile_handle:
            terminalfile = terminalfile_handle.read()
        terminallines = terminalfile.split('\n')
    except:
        return {}, {}, False

    system = get_system(terminal_filepath)
    foldername = get_foldername(terminal_filepath)

    landing_locations = []
    date = "n.a."
    datetime = "n.a."
    replay_moth = False
    for i in range(len(terminallines)):
        words = terminallines[i].split(' ')
        if(i == 0 and len(words) > 1):
            date = words[0]
            datetime = words[1]
        if words[0] == 'blink-location:':
            try:
                landing_location = np.array(eval(words[1] + words[2] + words[3]))
                landing_locations.append(landing_location)
            except:
                print('Error in', terminal_filepath, ' - parsing landing location: ' + str(words[1] + words[2] + words[3]))
        if(len(words) >= 4 and words[0] + words[1] + words[2] == 'Openinginsectlog:'):
            replay_moth = True

    terminal_data = {}
    terminal_data[KEY_LANDINGLOC] = landing_locations
    log_data = {}
    log_data[KEY_SYSTEM] = system
    log_data[KET_FOLDERNAME] = foldername
    log_data[KEY_DATE] = date
    log_data[KEY_DATETIME] = datetime
    return log_data, terminal_data, replay_moth


KEY_HORZONTALLANDIGLOC = 'horizontal_landing_precision'


def terminal_evaldata(terminal_data):
    terminal_evaldata = {}
    crashed_after_landing = False

    if terminal_data:
        if len(terminal_data[KEY_LANDINGLOC]) == 2:
            before = terminal_data[KEY_LANDINGLOC][0]
            after = terminal_data[KEY_LANDINGLOC][1]
            if abs(before[1] - after[1]) > 0.05:
                crashed_after_landing = True
            else:
                before[1] = 0
                after[1] = 0
                terminal_evaldata[KEY_HORZONTALLANDIGLOC] = np.linalg.norm(before - after) * 1000.

    return terminal_evaldata, crashed_after_landing


def terminal_evaldata_units():
    terminal_evaldata_unit = {}
    terminal_evaldata_unit[KEY_HORZONTALLANDIGLOC] = 'mm'
    return terminal_evaldata_unit


def log_data_units():
    log_data_unit = {}
    log_data_unit[KEY_DATE] = ''
    log_data_unit[KEY_DATETIME] = ''
    log_data_unit[KEY_SYSTEM] = ''
    log_data_unit[KET_FOLDERNAME] = ''
    return log_data_unit


if __name__ == "__main__":
    import os
    import sys
    if len(sys.argv) >= 2:
        folderpath = str(sys.argv[1])
    else:
        folderpath = os.path.expanduser('/home/ludwig/Documents/2020-06 testing-koppert/29-06/pats11/00001/')
    terminal_filepath = folderpath + 'terminal.log'

    log_data, terminal_data, replay_moth = read_terminalfile(terminal_filepath)
    print(log_data)
    print(terminal_data)
    print(replay_moth)
    terminal_stats, crashed_after_landing = terminal_evaldata(terminal_data)
    print(terminal_stats)