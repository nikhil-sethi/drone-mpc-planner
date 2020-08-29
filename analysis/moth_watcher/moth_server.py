#!/usr/bin/python3
import socket, json
import time, argparse
import pickle, glob, os
import numpy as np
import datetime
import sqlite3

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

def store_data(data):
    database_path = r"moth_records.db"
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    db_path = os.path.join(BASE_DIR, database_path)
    conn = create_connection(db_path)

    for flight in data["flights"]:
        parsed = datetime.datetime.strptime(flight['time'], "%m/%d/%Y, %H:%M:%S")
        date = iso_8601_format(parsed)
        sql = ''' INSERT INTO analytic_records(system,flight_time,flight_duration,start_RS_ID,\
        velocity_mean,velocity_std,velocity_max,turning_angle_mean,turning_angle_std,turning_angle_max,\
        radial_accelaration_mean,radial_accelaration_std,radial_accelaration_max, filename)\
        VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?) '''
        cur = conn.cursor()
        try:
            cur.execute(sql, (data["system"], date, *list(flight.values())[1:]))
        except:
            continue
        conn.commit()
        print(f"Saved {date} of {data['system']}")


parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is \
    reable for the electron app. \nExample python3 moth_server.py -i "./data"')

parser.add_argument('-i', help="Path to the folder with json files, 's' for server", required=True, default="s")
args = parser.parse_args()
if args.i != "s":
    files = glob.glob(args.i + "/*.json")
    for file_ in files:
        with open(file_) as json_file:
            data = json.load(json_file)
            store_data(data)
            print(f"Stored data of {file_} to database.")
else:
    HEADERSIZE = 10
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("145.94.54.216", 1243))
    s.listen(5)

    while True:
        # now our endpoint knows about the OTHER endpoint.
        clientsocket, address = s.accept()
        print(f"Connection from {address} has been established.")

        try:
            full_msg = b''
            new_msg = True
            while True:
                msg = clientsocket.recv(16)
                if new_msg:
                    msglen = int(msg[:HEADERSIZE])
                    new_msg = False
    
                full_msg += msg

                if len(full_msg)-HEADERSIZE == msglen:
                    print("full msg recieved")
                    data = pickle.loads(full_msg[HEADERSIZE:])
                    store_data(data)
                    new_msg = True
                    full_msg = b""
        except:
            pass
