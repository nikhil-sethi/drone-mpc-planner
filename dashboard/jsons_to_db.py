#!/usr/bin/env python3
import socket, json,shutil
import time, argparse
import pickle, glob, os, re
import numpy as np
import datetime
import sqlite3
from tqdm import tqdm

conn = None
cur = None

def create_connection(db_file):
    conn = None
    try:
        conn = sqlite3.connect(os.path.expanduser(db_file))
    except Exception as e:
        print(e)
    return conn

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def store_moths(fn,data):
    moths =data["moths"]

    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='moth_records' ''')
    moth_table_exist = cur.fetchone()[0]==1

    columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]

    #this section only is needed once to upgrade the db
    if 'Version' not in columns:
        cur.execute('ALTER TABLE moth_records ADD COLUMN Version TEXT')
    if 'Mode' not in columns:
        cur.execute('ALTER TABLE moth_records ADD COLUMN Mode TEXT')
    if 'Video_Filename' not in columns:
        cur.execute('ALTER TABLE moth_records ADD COLUMN Video_Filename TEXT')
    if 'FP' not in columns:
        cur.execute('ALTER TABLE moth_records ADD COLUMN FP TEXT')

    sql_insert = ''
    for moth in moths:

        if not moth_table_exist:
            sql_create = 'CREATE TABLE moth_records(system,time,'
            for s in list(moth.keys())[1:]:
                sql_create = sql_create + s + ','
            sql = sql_create[:-1] + ')'
            cur.execute(sql)
            conn.commit()
            moth_table_exist = True

        date = moth['time']
        if sql_insert == '':
            sql_insert = 'INSERT INTO moth_records(system,time,'
            sql_values = ') VALUES(?,?,'
            for s in list(moth.keys())[1:]:
                sql_insert = sql_insert + s + ','
                sql_values = sql_values + '?,'
            sql_insert = sql_insert[:-1] + sql_values[:-1] + ')'


        cur.execute(sql_insert, (data["system"], date, *list(moth.values())[1:]))
    conn.commit()

def store_mode(fn,data):
    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='mode_records' ''')
    mode_table_exist = cur.fetchone()[0]==1

    if not mode_table_exist:
        sql_create = 'CREATE TABLE mode_records(system,start_datetime,end_datetime,op_mode)'
        cur.execute(sql_create)
        conn.commit()
        moth_table_exist = True

    mode_data =data["mode"]

    sql_insert = 'INSERT INTO mode_records(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'

    for entry in mode_data:
        sub_entries = [] #there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]
        for sub_entry in sub_entries:
            dt_from = sub_entry['from']
            dt_till = sub_entry['till']
            cur.execute(sql_insert, (data["system"],dt_from,dt_till,sub_entry['mode']))
    conn.commit()

def load_systems():
    global cur
    sql_str = '''SELECT DISTINCT system from mode_records'''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = [d[0] for d in systems]
    return systems

def todatetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")

def clean_mode():

    systems = load_systems()
    all_modes_cleaned = []
    pbar = tqdm(systems,desc='Cleaning modes db')
    for system in pbar:
        sql_str = 'SELECT op_mode,start_datetime,end_datetime,system from mode_records where system="' + system + '" ORDER BY "start_datetime"'
        cur.execute(sql_str)
        modes = cur.fetchall()

        prev_i=0
        for i in range(1,len(modes)):
            entry = modes[i]
            prev_entry = modes[prev_i]
            d = todatetime(entry[1]) - todatetime(prev_entry[2])
            if (prev_entry[0]==entry[0] and prev_entry[1]==entry[1] and prev_entry[2]==entry[2]):
                modes[i] = ('_duplicate',modes[i][1],modes[i][2],modes[i][3])
            elif (prev_entry[0]==entry[0] and d < datetime.timedelta(minutes=10)):
                modes[prev_i] = (modes[prev_i][0],modes[prev_i][1],modes[i][2],modes[i][3])
                modes[i] = ('_delete',modes[i][1],modes[i][2],modes[i][3])
            else:
                prev_i = i

        modes_cleaned = [entry for entry in modes if entry[0][0][0] != '_']
        all_modes_cleaned = all_modes_cleaned + modes_cleaned

    sql_clear_table = 'DELETE FROM mode_records'
    cur.execute(sql_clear_table)
    conn.commit()

    sql_insert = 'INSERT INTO mode_records(op_mode,start_datetime,end_datetime,system) VALUES(?,?,?,?)'
    for mode in all_modes_cleaned:
        cur.execute(sql_insert, mode)
    conn.commit()

def store_hunts(fn,data):
    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='hunt_records' ''')
    hunt_table_exist = cur.fetchone()[0]==1
    conn.commit()
    sql_insert = ''

    hunts =data["hunts"]
    for hunt in hunts:

        if not hunt_table_exist:
            sql_create_table = 'CREATE TABLE hunt_records(system,start_datetime,end_datetime,'
            for s in list(hunt.keys())[2:]:
                sql_create_table = sql_create_table + s + ','
            sql_create_table = sql_create_table[:-1] + ')'
            cur.execute(sql_create_table)
            conn.commit()
            hunt_table_exist = True

        dt_from = hunt['from']
        dt_till = hunt['till']
        if sql_insert == '':
            sql_insert = 'INSERT INTO hunt_records(system,start_datetime,end_datetime,'
            sql__insert_values = ') VALUES(?,?,?,'
            for s in list(hunt.keys())[2:]:
                sql_insert = sql_insert + s + ','
                sql__insert_values = sql__insert_values + '?,'
            sql_insert = sql_insert[:-1] + sql__insert_values[:-1] + ')'


        cur.execute(sql_insert, (data["system"], dt_from,dt_till, *list(hunt.values())[2:]))
    conn.commit()

def store_data(data,fn):
    global conn
    global cur

    store_moths(fn,data)
    store_mode(fn,data)
    store_hunts(fn,data)

parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/jsons/')
parser.add_argument('-p','--period', help="Path to the folder with json files", default=0)
parser.add_argument('-o','--output_db_path', help="Path to the folder with json files", default='~/pats.db')
args = parser.parse_args()

conn = create_connection(args.output_db_path)
cur = conn.cursor()

while True:

    files = natural_sort([fp for fp in glob.glob(os.path.expanduser(args.input_folder + "/*.json"))])
    pbar = tqdm(files)
    for filename in pbar:
        pbar.set_description(os.path.basename(filename))
        flag_fn = filename[:-4] + 'processed'
        if not os.path.exists(flag_fn):
            with open(filename) as json_file:
                with open(flag_fn,'w') as flag_f:
                    data = json.load(json_file)
                    required_version='1.2'
                    if "version" in data and data["version"] == required_version:
                        store_data(data,os.path.basename(filename))
                        flag_f.write('OK')
                    else:
                        flag_f.write('WRONG VERSION ' + data["version"] + '. Want: ' + required_version)

    clean_mode()

    if args.period:
        print(str(datetime.datetime.now()) + ". Periodic update after: " + str(args.period))
        time.sleep(int(args.period))
    else:
        break
