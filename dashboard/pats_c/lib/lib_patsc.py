#!/usr/bin/env python3
import logging
import os
import re
import subprocess
import sqlite3
import numpy as np
import pandas as pd
from flask_login import current_user

db_data_path = os.path.expanduser('~/patsc/db/pats.db')
db_classification_path = os.path.expanduser('~/patsc/db/pats_human_classification.db')
db_systems_path = os.path.expanduser('~/patsc/db/pats_systems.db')
daemon_error_log = os.path.expanduser('~/patsc/logs/daemon_error.log')
patsc_error_log = os.path.expanduser('~/patsc/logs/patsc_error.log')

monster_window = pd.Timedelta(minutes=5)


def init_system_and_customer_options(customer_dict, demo):
    sys_options = []
    customer_options = []
    for customer in customer_dict.keys():
        if not (customer == 'Maintance' or customer == 'Admin' or customer == 'Unassigned_systems' or customer == 'Deactivated_systems'):
            customer_options.append({'label': customer, 'value': customer})
            for i, (system, location, crop) in enumerate(customer_dict[customer]):
                if customer == 'Pats':
                    sys_options.append({'label': system, 'value': system, 'title': system})
                elif demo:
                    sys_options.append({'label': 'Demo ' + crop, 'value': system, 'title': system})
                    break  # This break makes sure that only one system per customer is added for the demo user, like Bram wanted. Dirty demo hack part uno.
                elif location:
                    if len(customer_dict.keys()) == 1:
                        sys_options.append({'label': location, 'value': system, 'title': system})
                    else:
                        sys_options.append({'label': customer + ' ' + location, 'value': system, 'title': system})
                else:
                    sys_options.append({'label': customer + ' ' + str(i + 1), 'value': system, 'title': system})
    return customer_options, sys_options


def load_systems_customer(customer_name, cur):
    sql_str = '''SELECT system,location,crops.name FROM systems
                 JOIN customers ON customers.customer_id = systems.customer_id
                 JOIN crops ON crops.crop_id = customers.crop_id
                 WHERE customers.name = :customer_name
                 ORDER BY system_id''', {'customer_name': customer_name}
    systems = cur.execute(*sql_str).fetchall()
    return systems


def load_customers():
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            demo = 'demo' in username
            with open_systems_db() as con:
                sql_str = '''SELECT customers.name FROM customers
                            JOIN user_customer_connection ON user_customer_connection.customer_id=customers.customer_id
                            JOIN users ON users.user_id=user_customer_connection.user_id
                            WHERE users.name = :username
                            ORDER BY customers.name''', {'username': username}
                cur = con.execute(*sql_str)
                customers = cur.fetchall()
                customer_dict = {}
                for customer in customers:
                    customer_dict[customer[0]] = load_systems_customer(customer[0], cur)
            return customer_dict, demo
    return {}, False


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


def datetime_to_str(d):
    return d.strftime('%Y%m%d_%H%M%S')


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


def window_filter_monster(detection_df, monster_df):
    monster_times = monster_df['time'].values
    detection_times = detection_df['time'].values
    monster_times_mesh, detection_times_mesh = np.meshgrid(monster_times, detection_times, sparse=True)
    detections_with_monsters = np.sum((monster_times_mesh >= detection_times_mesh - monster_window) * (monster_times_mesh <= detection_times_mesh + monster_window), axis=1)
    detections_without_monsters = detection_df.iloc[detections_with_monsters == 0]
    return detections_without_monsters


def clean_detection_json_entry(detection, data):
    # fix a bug that lived for a few days having different versioning in the system table and the detections table (can be removed if these jsons are obsolete)
    if "version" in data and "version" in detection:
        if data["version"] != detection["version"] and str(detection["version"]) == "1.1":
            detection["version"] = "1.2"
    else:
        detection["version"] = "1.0"

    # remove unused data from jsons version < 1.4:
    if float(detection["version"]) < 1.4:
        if 'FP' in detection:
            detection.pop('FP')
        if 'TA_mean' in detection:
            detection.pop('TA_mean')
        if 'TA_std' in detection:
            detection.pop('TA_std')
        if 'TA_max' in detection:
            detection.pop('TA_max')
        if 'RA_mean' in detection:
            detection.pop('RA_mean')
        if 'RA_std' in detection:
            detection.pop('RA_std')
        if 'RA_max' in detection:
            detection.pop('RA_max')
    return detection


def true_positive(detection):
    if 'version' in detection:
        if detection['version'] == '1.0':
            return detection['duration'] > 1 and detection['duration'] < 10
        else:
            return detection['duration'] > 1 and detection['duration'] < 10 and detection['dist_traveled'] > 0.15 and detection['dist_traveled'] < 4
    else:
        return False


def detection_classes(system):
    sql_str = f'''  SELECT detections.lg_label,avg_size,std_size,floodfill_avg_size,floodfill_std_size FROM detections
                    JOIN crop_detection_connection ON detections.detection_id = crop_detection_connection.detection_id
                    JOIN crops ON crop_detection_connection.crop_id = crops.crop_id
                    JOIN customers ON crops.crop_id = customers.crop_id
                    JOIN systems ON customers.customer_id = systems.customer_id
                    WHERE systems.system = '{system}'  '''
    with open_systems_db() as con:
        detection_info = con.execute(sql_str).fetchall()
        detection_info = [(name, avg_size - std_size, avg_size + 2 * std_size, floodfill_avg_size - floodfill_std_size, floodfill_avg_size + 2 * floodfill_std_size) for name, avg_size, std_size, floodfill_avg_size, floodfill_std_size in detection_info]
        return detection_info


def check_verion(current_version, minimal_version):
    current_version = current_version.split('.')
    minimal_version = minimal_version.split('.')
    for i in range(0, np.max([len(current_version), len(minimal_version)])):
        if i == len(current_version) or i == len(minimal_version):
            return len(current_version) > len(minimal_version)
        else:
            if int(current_version[i]) == int(minimal_version[i]):
                continue
            else:
                return int(current_version[i]) > int(minimal_version[i])


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
