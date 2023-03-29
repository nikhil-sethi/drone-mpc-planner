#!/usr/bin/python3
import argparse
import os
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
    systems = pd.read_sql_query('''
                                    SELECT c_systems.id, maintenance, baseboard, sections.name, operational_modes.name as operation
                                    FROM c_systems
                                    JOIN sections ON sections.id = c_systems.section_id
                                    JOIN operational_modes ON operational_modes.id = c_systems.operation
                                    ORDER BY c_systems.id
                                    ''', con).to_dict('records')

groups: Dict[str, Dict[str, List[str]]] = {'all': {'hosts': []}, 'c': {'hosts': []}, 'x': {'hosts': []}, 'trapeye': {'hosts': []}, 'blind': {'hosts': []}, 'kevin': {'hosts': []}, 'qc': {'hosts': []}, 'rc': {'hosts': []}, 'darkroom': {'hosts': []}}
for system in systems:
    maintenance = False
    if system['maintenance']:
        system['maintenance'] = system['maintenance'].strip()
        if '_' in system['maintenance']:
            system['maintenance'] = system['maintenance'].split('_')[0]
        if datetime.strptime(system['maintenance'], "%Y%m%d") > datetime.today():
            maintenance = True

    system['baseboard'] = bool(system['baseboard'])
    system['maintenance'] = bool(maintenance)
    system_name = 'pats' + str(system['id'])
    groups[system_name] = {'hosts': [system_name]}
    if not maintenance:
        if system['baseboard'] or True:  # optional use for upgrading baseboards
            groups['all']['hosts'].append(system_name)
            if system['operation'] == 'c':
                groups['c']['hosts'].append(system_name)

            if system['operation'] == 'x':
                groups['x']['hosts'].append(system_name)

            if system['operation'] == 'trapeye':
                groups['trapeye']['hosts'].append(system_name)

            if system['operation'] == 'blind':
                groups['blind']['hosts'].append(system_name)

            if system['operation'] == 'kevin':
                groups['kevin']['hosts'].append(system_name)

            if system['operation'] == 'qc':
                groups['qc']['hosts'].append(system_name)

            if system['operation'] == 'rc':
                groups['rc']['hosts'].append(system_name)

            if system['operation'] == 'darkroom':
                groups['darkroom']['hosts'].append(system_name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that creates an inventory based on the system database')
    parser.add_argument('--list', help="returns the inventory listed", dest='list', action='store_true')
    args = parser.parse_args()

    if args.list:  # this usefull if statement is necesarry
        print(json.dumps(groups))
    else:
        print(json.dumps(groups))
