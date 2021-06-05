#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from io import StringIO
import pandas as pd


def cleanWhitespaces(filepath):
    file = open(filepath, "r")
    datastring = file.read()
    file.close()
    datastring = datastring.replace(' ', '')
    datastring = StringIO(datastring)
    return datastring


if __name__ == "__main__":
    original_samplerate = 60
    target_samplerate = 90
    insectlog_filepath = os.path.expanduser('~/code/pats/pc/insect_logs/56.csv')

    csv_string = cleanWhitespaces(insectlog_filepath)
    insect_log = pd.read_csv(csv_string, sep=';')
    n_samples = len(insect_log)

    datetime = pd.date_range('01-01-1970', periods=n_samples, freq=str(int(1. / original_samplerate * 1000)) + 'ms')
    insect_log['time'] = datetime
    insectlog_resampled = insect_log.resample(str(int(1. / target_samplerate * 1000)) + 'ms', on='time').mean().ffill()
    insectlog_resampled['time'] = insectlog_resampled.index

    insect_path_elements = insectlog_filepath.split('.')
    insect_path_elements[0] += '-90fps'
    insectlog_filepath = insect_path_elements[0] + '.csv'
    insectlog_resampled.to_csv(insectlog_filepath, index=False, sep=';')

    # VISUALIZE RESULTS:
#	import matplotlib.pyplot as plt
#	from mpl_toolkits.mplot3d import Axes3D
#	fig = plt.figure()
#	ax = fig.add_subplot(111)
#	ax.plot(insect_log.index/len(insect_log), insect_log['posX_insect'], label='orig')
#	ax.plot(insectlog_resampled.index/len(insectlog_resampled), insectlog_resampled['posX_insect'], label='resampled')
#	fig.suptitle('Figure Title')
#	ax.set_title('Axes Title')
#	ax.set_xlabel('x [m]')
#	ax.set_ylabel('y [m]')
#	ax.legend()
#	plt.show()
