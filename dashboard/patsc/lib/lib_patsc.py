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
db_meta_path = os.path.expanduser('~/patsc/db/pats_systems.db')
patsc_log_path = os.path.expanduser('~/patsc/logs/')
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


def load_systems_customer(customer_name, con):
    sql_str = '''SELECT system,location,crops.name FROM systems
                 JOIN customers ON customers.customer_id = systems.customer_id
                 JOIN crops ON crops.crop_id = customers.crop_id
                 WHERE customers.name = :customer_name
                 ORDER BY system_id''', {'customer_name': customer_name}
    systems = con.execute(*sql_str).fetchall()
    return systems


def load_customers():
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            demo = 'demo' in username
            with open_meta_db() as con:
                sql_str = '''SELECT customers.name FROM customers
                            JOIN user_customer_connection ON user_customer_connection.customer_id=customers.customer_id
                            JOIN users ON users.user_id=user_customer_connection.user_id
                            WHERE users.name = :username
                            ORDER BY customers.name''', {'username': username}
                cur = con.execute(*sql_str)
                customers = cur.fetchall()
                customer_dict = {}
                for customer in customers:
                    customer_dict[customer[0]] = load_systems_customer(customer[0], con)
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


def open_meta_db():
    con = None
    try:
        con = sqlite3.connect(db_meta_path)
        con.execute('pragma journal_mode=wal')
    except Exception as e:
        print(e)
    return con


def check_if_table_exists(table_name, con):
    cur = con.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='" + table_name + "'")
    return cur.fetchone()[0]


def datetime_to_str(d):
    return d.strftime('%Y%m%d_%H%M%S')


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


def window_filter_monster(detection_df, monster_df):
    monster_times = monster_df['start_datetime'].values
    detection_times = detection_df['start_datetime'].values
    monster_times_mesh, detection_times_mesh = np.meshgrid(monster_times, detection_times, sparse=True)
    detections_with_monsters = np.sum((monster_times_mesh >= detection_times_mesh - monster_window) * (monster_times_mesh <= detection_times_mesh + monster_window), axis=1)
    detections_without_monsters = detection_df.iloc[detections_with_monsters == 0]
    return detections_without_monsters


def true_positive(detection):
    if 'version' in detection:
        if detection['version'] == '1.0':
            return detection['duration'] > 1 and detection['duration'] < 10
        else:
            return detection['duration'] > 1 and detection['duration'] < 10 and detection['dist_traveled'] > 0.15 and detection['dist_traveled'] < 4
    else:
        return False


def detection_classes(system):
    sql_str = f'''  SELECT detection_classes.lg_label,avg_size,std_size,floodfill_avg_size,floodfill_std_size FROM detection_classes
                    JOIN crop_detection_class_connection ON detection_classes.class_id = crop_detection_class_connection.class_id
                    JOIN crops ON crop_detection_class_connection.crop_id = crops.crop_id
                    JOIN customers ON crops.crop_id = customers.crop_id
                    JOIN systems ON customers.customer_id = systems.customer_id
                    WHERE systems.system = '{system}'  '''
    with open_meta_db() as con:
        detection_info = con.execute(sql_str).fetchall()
        detection_info = [(name, avg_size - std_size, avg_size + 2 * std_size, floodfill_avg_size - floodfill_std_size, floodfill_avg_size + 2 * floodfill_std_size) for name, avg_size, std_size, floodfill_avg_size, floodfill_std_size in detection_info]
        return detection_info


def check_verion(current_version_str: str, minimal_version_str: str):
    if current_version_str == minimal_version_str:
        return True
    current_version = current_version_str.split('.')
    minimal_version = minimal_version_str.split('.')
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
