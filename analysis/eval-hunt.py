#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
from flightstates import init_flightstate_data, eval_current_flightstate, key_takeofftime
from cleanwhitespaces import cleanWhitespaces


def concatenate_insectlogs(folderpath):
    insect_data = pd.DataFrame()
    for file in os.listdir(folderpath):
        if file.endswith('.csv') and file.startswith('log_itrk'):
            data_string = cleanWhitespaces(folderpath + '/' + file)
            dataframe = pd.read_csv(data_string, sep=';')
            insect_data = pd.concat([insect_data, dataframe], ignore_index=True)

    replayinsect_data = pd.DataFrame()
    n_replaylogs = 0
    for file in os.listdir(folderpath):
        if file.endswith('.csv') and file.startswith('log_rtrk'):
            data_string = cleanWhitespaces(folderpath + '/' + file)
            dataframe = pd.read_csv(data_string, sep=';')
            replayinsect_data = pd.concat([replayinsect_data, dataframe], ignore_index=True)
            n_replaylogs += 1

    key_nlostframes = 'n_frames_lost_insect'
    key_posxtarget = 'posX_insect'
    key_posytarget = 'posY_insect'
    key_posztarget = 'posZ_insect'
    if(n_replaylogs > 0):
        key_nlostframes = 'n_frames_lost_replay'
        key_posxtarget = 'posX_replay'
        key_posytarget = 'posY_replay'
        key_posztarget = 'posZ_replay'
        return replayinsect_data.sort_values(by=['RS_ID']).reset_index(), key_nlostframes, key_posxtarget, key_posytarget, key_posztarget
    else:
        return insect_data.sort_values(by=['RS_ID']).reset_index(), key_nlostframes, key_posxtarget, key_posytarget, key_posztarget


key_minimalerror = 'minimal_error'
key_time_minimalerror = 'time_minimal_error'
key_minimalerror_untracked = 'minimal_error_untracked'
key_time_minimalerror_untracked = 'time_minimal_error_untracked'
key_insecttracking = 'insect_tracking'


def read_hunt(folderpath):
    drone_filepath = folderpath + '/log.csv'
    drone_data_string = cleanWhitespaces(drone_filepath)
    drone_data = pd.read_csv(drone_data_string, sep=';')

    insect_data, key_nlostframes, key_posxtarget, key_posytarget, key_posztarget = concatenate_insectlogs(folderpath)

    n_samples_drone = np.size(drone_data, 0)
    n_samples_insect = np.size(insect_data, 0)

    errors = np.array([])
    errors_untracked = np.array([])
    times = np.array([])
    for i in range(n_samples_drone):
        time = drone_data['elapsed'][i]
        nav_state = drone_data['nav_state'][i]
        auto_throttle = drone_data['autoThrottle'][i]

        try:
            insect_idx = insect_data[insect_data['RS_ID'] == drone_data['RS_ID'][i]].index[0]
            target = np.array([insect_data[key_posxtarget][insect_idx], insect_data[key_posytarget][insect_idx], insect_data[key_posztarget][insect_idx]])

            if(drone_data['valid'][i] == 1):
                pos = np.array([drone_data['posX_drone'][i], drone_data['posY_drone'][i], drone_data['posZ_drone'][i]])
                error = np.linalg.norm(target - pos)
                times = np.append(times, np.append([time]))
                errors_untracked = np.append(errors_untracked, np.append([error]))

                if(error < closest_distance_untracked):
                    # print(str(insect_data['RS_ID'][insect_idx])+':'+str(error)+'@'+str(drone_data['elapsed'][i]))
                    closest_distance_untracked = error
                    time_at_closest_distance_untracked = drone_data['elapsed'][i]
                if(insect_data[key_nlostframes][insect_idx] == 0):
                    errors = np.append(errors, np.append([error]))
                    if(error < closest_distance):
                        closest_distance = error
                        time_at_closest_distance = drone_data['elapsed'][i]
                else:
                    errors = np.append(errors, np.array([np.nan]))

        except IndexError:
            pass

    return times, errors, errors_untracked


if __name__ == "__main__":
    folderpath = os.path.expanduser('/home/ludwig/Documents/codes/pats/pc/build-vscode/logging')
    times, errors, errors_untracked = read_hunt(folderpath)
