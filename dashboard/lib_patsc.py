#!/usr/bin/env python3
import os, re
import sqlite3
from tqdm import tqdm
from json.decoder import JSONDecodeError

def open_db(db_file):
    conn = None
    cur = None
    try:
        conn = sqlite3.connect(os.path.expanduser(db_file),timeout=30.0)
        cur = conn.cursor()
    except Exception as e:
        print(e)
    return conn,cur

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def clean_moth_json_entry(moth,data):
    #fix a bug that lived for a few days having different versioning in the system table and the moth table (can be removed if these jsons are obsolete)
    if "version" in data and "Version" in moth:
        if data["version"] != moth["Version"] and str(moth["Version"]) == "1.1":
            moth["Version"] = "1.2"
    else:
            moth["Version"] = "1.0"

    #remove unused data from jsons version < 1.4:
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

def true_positive(moth,minimal_size):
    if moth['Version'] == '1.0':
        return moth['duration'] > 1 and moth['duration'] < 10
    else:
        return moth['duration'] > 1 and moth['duration'] < 10 and moth['Dist_traveled'] > 0.15 and moth['Dist_traveled'] < 4 and moth['Size'] > minimal_size
