#!/usr/bin/env python3
import socket, json,shutil
import time, argparse
import pickle, glob, os
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

def store_data(data,fn):
    global conn
    global cur

    database_path = r"pats_records.db"
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    db_path = os.path.join(BASE_DIR, database_path)
    flights =data["flights"]
    pbar2=tqdm(flights,desc=fn)
    for flight in pbar2:
        
        if not conn:
            if not os.path.exists(db_path):
                conn = create_connection(db_path)
                cur = conn.cursor()
                sql_heads = 'CREATE TABLE moth_records(system,time,'
                for s in list(flight.keys())[1:]:
                    sql_heads = sql_heads + s + ','
                sql = sql_heads[:-1] + ')'
                cur.execute(sql)
                conn.commit()
            else:
                conn = create_connection(db_path)
                cur = conn.cursor()

        parsed = datetime.datetime.strptime(flight['time'], "%m/%d/%Y, %H:%M:%S")
        date = iso_8601_format(parsed)
        sql_heads = 'INSERT INTO moth_records(system,time,'
        sql_values = ') VALUES(?,?,'
        for s in list(flight.keys())[1:]:
            sql_heads = sql_heads + s + ','
            sql_values = sql_values + '?,'
        sql = sql_heads[:-1] + sql_values[:-1] + ')'
        
        try:
            cur.execute(sql, (data["system"], date, *list(flight.values())[1:]))
        except:
            continue
        conn.commit()
        pbar2.set_description(f"Saved {date} of {data['system']}")

parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
parser.add_argument('-i', help="Path to the folder with json files", required=True)
args = parser.parse_args()

files = glob.glob(args.i + "/*.json")
pbar= tqdm(files, desc='Total: ')
for file_ in pbar:
    pbar.set_description(file_)
    with open(file_) as json_file:
        try:
            data = json.load(json_file)
            shutil.move(file_,file_+'.OK')
            store_data(data,os.path.basename(file_))
        except:
            shutil.move(file_,file_+'.error')
            continue 
        
