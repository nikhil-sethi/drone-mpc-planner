#!/usr/bin/python3
import argparse
import os
import json
import sqlite3
from typing import Dict, List

db_systems_path = os.path.expanduser('~/patsc/db/pats_systems.db')


def open_systems_db():
    con = None
    try:
        con = sqlite3.connect(db_systems_path)
        con.execute('pragma journal_mode=wal')
    except Exception as e:
        print(e)
    return con


sql_str = ''' SELECT system,active,customers.name FROM systems JOIN customers ON customers.customer_id = systems.customer_id ORDER BY system_id'''
with open_systems_db() as con:
    systems = con.execute(sql_str).fetchall()

groups: Dict[str, Dict[str, List[str]]] = {'monitoring': {'children': []}, 'hunts': {'children': []}, 'office': {'children': []}}
for system, active, customer in systems:
    customer = customer.replace(' ', '_').replace('.', '_')
    if active:
        if customer in groups:
            groups[customer]['hosts'].append(system)
        else:
            groups[customer] = {'hosts': []}
            groups[customer]['hosts'].append(system)
            if customer in ['Pats', 'Agrobofood']:
                groups['hunts']['children'].append(customer)
            elif customer in ['Admin', 'Unassigned_systems', 'Deactivated_systems', 'Maintance']:
                groups['office']['children'].append(customer)
            else:
                groups['monitoring']['children'].append(customer)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that creates an inventory based on the system database')
    parser.add_argument('--list', help="returns the inventory listed", dest='list', action='store_true')
    args = parser.parse_args()

    if args.list:  # this usefull if statement is necesarry
        print(json.dumps(groups))
    else:
        print(json.dumps(groups))
