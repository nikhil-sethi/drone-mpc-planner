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
        conn = sqlite3.connect(db_file)
    except Exception as e:
        print(e)
    return conn

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def store_moths(fn,data):
    moths =data["moths"]
    pbar2=tqdm(moths,desc='Moths')

    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='moth_records' ''')
    moth_table_exist = cur.fetchone()[0]==1

    sql_insert = ''
    for moth in pbar2:

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
        # pbar2.set_description(f"Saved moth {date} of {data['system']}")

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
    
    pbar2=tqdm(mode_data,desc='Mode: ')
    for entry in pbar2:  
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
    for system in systems:
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
    hunts =data["hunts"]
    pbar2=tqdm(hunts,desc='Hunts')

    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='hunt_records' ''')
    hunt_table_exist = cur.fetchone()[0]==1
    conn.commit()
    sql_insert = ''

    for hunt in pbar2:

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
parser.add_argument('-i', help="Path to the folder with json files", required=True)
args = parser.parse_args()

database_path = r"pats_records.db"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
db_path = os.path.join(BASE_DIR, database_path)
conn = create_connection(db_path)
cur = conn.cursor()

files = natural_sort([fp for fp in glob.glob(args.i + "/*.json")])
for filename in files:
    print('Processing: ' + filename)
    flag_fn = filename[:-4] + 'processed'
    if not os.path.exists(flag_fn):
        with open(filename) as json_file:
            with open(flag_fn,'w') as flag_f:
                data = json.load(json_file)
                required_version='1.1'
                if "version" in data and data["version"] == required_version:
                    store_data(data,os.path.basename(filename))
                    flag_f.write('OK')
                else:
                    flag_f.write('WRONG VERSION ' + data["version"] + '. Want: ' + required_version)

clean_mode()
