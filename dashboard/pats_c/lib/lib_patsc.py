#!/usr/bin/env python3
import logging
import os
import re
import subprocess
import sqlite3

db_data_path = os.path.expanduser('~/patsc/db/pats.db')
db_classification_path = os.path.expanduser('~/patsc/db/pats_human_classification.db')
db_systems_path = os.path.expanduser('~/patsc/db/pats_systems.db')


def open_data_db():
    con = None
    try:
        con = sqlite3.connect(db_data_path, timeout=15.0)
        con.execute('pragma journal_mode=wal')
    except Exception as e:
        print(e)
    return con


def open_classification_db():
    con = None
    try:
        con = sqlite3.connect(db_classification_path)
    except Exception as e:
        print(e)
    return con


def open_systems_db():
    con = None
    try:
        con = sqlite3.connect(db_systems_path)
        con.execute('pragma journal_mode=wal')
    except Exception as e:
        print(e)
    return con


def natural_sort(l):
    def convert(text): return int(text) if text.isdigit() else text.lower()
    def alphanum_key(key): return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


def clean_moth_json_entry(moth, data):
    # fix a bug that lived for a few days having different versioning in the system table and the moth table (can be removed if these jsons are obsolete)
    if "version" in data and "Version" in moth:
        if data["version"] != moth["Version"] and str(moth["Version"]) == "1.1":
            moth["Version"] = "1.2"
    else:
        moth["Version"] = "1.0"

    # remove unused data from jsons version < 1.4:
    if float(moth["Version"]) < 1.4:
        if 'FP' in moth:
            moth.pop('FP')
        if 'TA_mean' in moth:
            moth.pop('TA_mean')
        if 'TA_std' in moth:
            moth.pop('TA_std')
        if 'TA_max' in moth:
            moth.pop('TA_max')
        if 'RA_mean' in moth:
            moth.pop('RA_mean')
        if 'RA_std' in moth:
            moth.pop('RA_std')
        if 'RA_max' in moth:
            moth.pop('RA_max')
    return moth


def true_positive(moth, minimal_size):
    if moth['Version'] == '1.0':
        return moth['duration'] > 1 and moth['duration'] < 10
    else:
        return moth['duration'] > 1 and moth['duration'] < 10 and moth['Dist_traveled'] > 0.15 and moth['Dist_traveled'] < 4 and moth['Size'] > minimal_size


def execute(cmd, retry=1, logger_name=''):
    if logger_name != '':
        logger = logging.getLogger(logger_name)

    p_result = None
    n = 0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result != None:
                n = n + 1
                break
            if logger_name == '':
                print(stdout_line.decode('utf-8'), end='')
            else:
                logger.info(stdout_line.decode('utf-8'))
        popen.stdout.close()
    return p_result
