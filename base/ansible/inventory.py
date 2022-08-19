#!/usr/bin/python3
import argparse
import os
import re
import json
import sqlite3
from datetime import datetime
from typing import Dict, List
import pandas as pd

db_systems_path = os.path.expanduser('~/patsc/db/meta.db')


def open_meta_db():
    try:
        con = sqlite3.connect(db_systems_path)
        con.execute('pragma journal_mode=wal')
        return con
    except Exception as e:  # pylint: disable=broad-except
        print(e)
    return None


with open_meta_db() as con:
    systems = con.execute('''SELECT system,operation,maintenance,baseboard,customers.name FROM systems JOIN customers ON customers.customer_id = systems.customer_id ORDER BY system_id''').fetchall()
    operation_modes = pd.read_sql_query('SELECT * FROM operational_modes', con)
operation_modes = operation_modes.set_index('name').to_dict()['status_id']

groups: Dict[str, Dict[str, List[str]]] = {'all': {'hosts': []}, 'c': {'hosts': []}, 'x': {'hosts': []}, 'kevin': {'hosts': []}, 'qc': {'hosts': []}, 'tree': {'hosts': []}, 'darkroom': {'hosts': []}}

for system, operation_mode, maintenance_date_str, baseboard, customer in systems:
    customer = re.sub('[^a-zA-Z0-9 \n\.]', '', customer)

    maintenance = False
    if maintenance_date_str:
        maintenance_date_str = maintenance_date_str.strip()
        if datetime.strptime(maintenance_date_str, "%Y%m%d") > datetime.today():
            maintenance = True

    if not maintenance:
        if baseboard or True:  # optional use for upgrading baseboards
            groups['all']['hosts'].append(system)
            if operation_mode == operation_modes['c']:
                groups['c']['hosts'].append(system)

            if operation_mode == operation_modes['x']:
                groups['x']['hosts'].append(system)

            if operation_mode == operation_modes['kevin']:
                groups['kevin']['hosts'].append(system)

            if operation_mode == operation_modes['qc']:
                groups['qc']['hosts'].append(system)

            if operation_mode == operation_modes['tree']:
                groups['tree']['hosts'].append(system)

            if operation_mode == operation_modes['darkroom']:
                groups['darkroom']['hosts'].append(system)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that creates an inventory based on the system database')
    parser.add_argument('--list', help="returns the inventory listed", dest='list', action='store_true')
    args = parser.parse_args()

    if args.list:  # this usefull if statement is necesarry
        print(json.dumps(groups))
    else:
        print(json.dumps(groups))
