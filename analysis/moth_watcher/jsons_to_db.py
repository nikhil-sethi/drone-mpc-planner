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
    """ create a database connection to the SQLite database
        specified by db_file
    :param db_file: database file
    :return: Connection object or None
    """
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

def iso_8601_format(dt):
    """YYYY-MM-DDThh:mm:ssTZD (1997-07-16T19:20:30-03:00)"""

    if dt is None:
        return ""

    fmt_datetime = dt.strftime('%Y-%m-%dT%H:%M:%S')
    tz = dt.utcoffset()
    if tz is None:
        fmt_timezone = "+00:00"
    else:
        fmt_timezone = str.format('{0:+06.2f}', float(tz.total_seconds() / 3600))

    return fmt_datetime + fmt_timezone

def store_moths(fn,data,db_path):
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

        parsed = datetime.datetime.strptime(moth['time'], "%m/%d/%Y, %H:%M:%S")
        date = iso_8601_format(parsed)
        if sql_insert == '':
            sql_insert = 'INSERT INTO moth_records(system,time,'
            sql_values = ') VALUES(?,?,'
            for s in list(moth.keys())[1:]:
                sql_insert = sql_insert + s + ','
                sql_values = sql_values + '?,'
            sql_insert = sql_insert[:-1] + sql_values[:-1] + ')'
        
        try:
            cur.execute(sql_insert, (data["system"], date, *list(moth.values())[1:]))
        except:
            continue
        conn.commit()
        # pbar2.set_description(f"Saved moth {date} of {data['system']}")

def store_mode(fn,data,db_path):
    cur.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='mode_records' ''')
    mode_table_exist = cur.fetchone()[0]==1

    if not mode_table_exist:
        sql_create = 'CREATE TABLE mode_records(system,start_datetime,end_datetime,op_mode)'
        cur.execute(sql_create)
        conn.commit()
        moth_table_exist = True

    mode_data =data["mode"]

    sql_insert = 'INSERT INTO mode_records(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'
    
    pbar2=tqdm(mode_data,desc='mode: ')
    for entry in pbar2:  
        sub_entries = [] #there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]
        for sub_entry in sub_entries:

            dt_from = datetime.datetime.strptime(sub_entry['from'], "%Y%m%d_%H%M%S")
            dt_till = datetime.datetime.strptime(sub_entry['till'], "%Y%m%d_%H%M%S")
            dt_from = iso_8601_format(dt_from)
            dt_till = iso_8601_format(dt_till)
            try:
                cur.execute(sql_insert, (data["system"],dt_from,dt_till,sub_entry['mode']))
            except:
                continue
            conn.commit()

        
def store_hunts(fn,data,db_path):
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

        dt_from = datetime.datetime.strptime(hunt['from'], "%Y%m%d_%H%M%S")
        dt_till = datetime.datetime.strptime(hunt['till'], "%Y%m%d_%H%M%S")
        dt_from = iso_8601_format(dt_from)
        dt_till = iso_8601_format(dt_till)
        if sql_insert == '':
            sql_insert = 'INSERT INTO hunt_records(system,start_datetime,end_datetime,'
            sql__insert_values = ') VALUES(?,?,?,'
            for s in list(hunt.keys())[2:]:
                sql_insert = sql_insert + s + ','
                sql__insert_values = sql__insert_values + '?,'
            sql_insert = sql_insert[:-1] + sql__insert_values[:-1] + ')'
        
        try:
            cur.execute(sql_insert, (data["system"], dt_from,dt_till, *list(hunt.values())[2:]))
        except:
            continue
        conn.commit()
        

def store_data(data,fn):
    global conn
    global cur

    database_path = r"pats_records.db"
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    db_path = os.path.join(BASE_DIR, database_path)

    if not conn:
        conn = create_connection(db_path)
        cur = conn.cursor()

    store_moths(fn,data,db_path)
    store_mode(fn,data,db_path)
    store_hunts(fn,data,db_path)

parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
parser.add_argument('-i', help="Path to the folder with json files", required=True)
args = parser.parse_args()

files = natural_sort([fp for fp in glob.glob(args.i + "/*.json")])
pbar= tqdm(files, desc='Total: ')
for filename in pbar:
    pbar.set_description(filename)
    flag_fn = filename[:-4] + 'processed'
    if not os.path.exists(flag_fn):
        with open(filename) as json_file:
            with open(flag_fn,'w') as flag_f:
                try:
                    data = json.load(json_file)
                    if "version" in data and data["version"] == '1.0':
                        store_data(data,os.path.basename(filename))
                        flag_f.write('OK')
                    else:
                        flag_f.write('OLD VERSION')
                except:
                    shutil.move(filename,filename +'.error')
                    continue
