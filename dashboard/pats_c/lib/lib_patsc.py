#!/usr/bin/env python3
import logging
import os
import re
import subprocess
import sqlite3
import numpy as np
import pandas as pd

db_data_path = os.path.expanduser('~/patsc/db/pats.db')
db_classification_path = os.path.expanduser('~/patsc/db/pats_human_classification.db')
db_systems_path = os.path.expanduser('~/patsc/db/pats_systems.db')

monster_window = pd.Timedelta(minutes=5)


def regexp(expr, item):
    reg = re.compile(expr)
    return reg.search(item) is not None


def open_data_db():
    con = None
    try:
        con = sqlite3.connect(db_data_path, timeout=15.0)
        con.execute('pragma journal_mode=wal')
        con.create_function("REGEXP", 2, regexp)
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


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


def window_filter_monster(insect_df, monster_df):
    monster_times = monster_df['time'].values
    insect_times = insect_df['time'].values
    monster_times_mesh, insect_times_mesh = np.meshgrid(monster_times, insect_times, sparse=True)
    insects_with_monsters = np.sum((monster_times_mesh >= insect_times_mesh - monster_window) * (monster_times_mesh <= insect_times_mesh + monster_window), axis=1)
    insect_without_monsters = insect_df.iloc[insects_with_monsters == 0]
    return insect_without_monsters


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


def true_positive(moth):
    if 'Version' in moth:
        if moth['Version'] == '1.0':
            return moth['duration'] > 1 and moth['duration'] < 10
        else:
            return moth['duration'] > 1 and moth['duration'] < 10 and moth['Dist_traveled'] > 0.15 and moth['Dist_traveled'] < 4
    else:
        return False

def get_insects_for_system(system):
    sql_str = f'''  SELECT insects.LG_name,avg_size,std_size,floodfill_avg_size,floodfill_std_size FROM insects
                    JOIN crop_insect_connection ON insects.insect_id = crop_insect_connection.insect_id
                    JOIN crops ON crop_insect_connection.crop_id = crops.crop_id
                    JOIN customers ON crops.crop_id = customers.crop_id
                    JOIN systems ON customers.customer_id = systems.customer_id
                    WHERE systems.system = '{system}'  '''
    with open_systems_db() as con:
        insect_info = con.execute(sql_str).fetchall()
        insect_info = [(name, avg_size - std_size, avg_size + 2 * std_size, floodfill_avg_size - floodfill_std_size, floodfill_avg_size + 2 * floodfill_std_size) for name, avg_size, std_size, floodfill_avg_size, floodfill_std_size in insect_info]
        return insect_info


def check_verion(current_version, minimal_version):
    current_version = current_version.split('.')
    minimal_version = minimal_version.split('.')
    for i in range(0, np.max([len(current_version), len(minimal_version)])):
        if i == len(current_version) or i == len(minimal_version):
            if int(current_version[i]) == int(minimal_version):
                continue
            else:
                return int(current_version[i]) > int(minimal_version)
        else:
            return len(current_version) > len(minimal_version)


def execute(cmd, retry=1, logger_name=''):
    if logger_name != '':
        logger = logging.getLogger(logger_name)

    p_result = None
    n = 0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result is not None:
                n = n + 1
                break
            if logger_name == '':
                print(stdout_line.decode('utf-8'), end='')
            else:
                logger.info(stdout_line.decode('utf-8'))
        popen.stdout.close()
    return p_result
