#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from io import StringIO
import pandas as pd

x = 0
y = 1
z = 2

def cleanWhitespaces(filepath):
    """Remove whitespaces from the log which disturbes the read_csv function"""
    file = open(filepath, "r")
    datastring = file.read()
    file.close()
    datastring = datastring.replace(' ', '')
    datastring = StringIO(datastring)
    return datastring


shift = [0., -0.5, 0.7]

# Open csv data:
insectlog_filepath = os.path.expanduser('~/code/pats/base/replay_insects/65-90fps.csv')
csv_string = cleanWhitespaces(insectlog_filepath)
insect_log = pd.read_csv(csv_string, sep=';')

# Change the data:
insect_log["posX_replay"]  += shift[x]
insect_log["sposX_replay"] += shift[x]
insect_log["posY_replay"]  += shift[y]
insect_log["sposY_replay"] += shift[y]
insect_log["posZ_replay"]  += shift[z]
insect_log["sposZ_replay"] += shift[z]

# Save new data in file:
insect_path_elements = insectlog_filepath.split('.')
insect_path_elements[0] += '-copy'
insectlog_filepath = insect_path_elements[0] + '.csv'
insect_log.to_csv(insectlog_filepath, index=False, sep=';')
