#!/usr/bin/env python3
import sys
from typing import Dict, List
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc
import requests
import json
import datetime
import glob
import os
import math
import argparse
import pandas as pd
import numpy as np
from tqdm import tqdm
from time import sleep
from json.decoder import JSONDecodeError

lg_bin_width = 5


def read_cred_lg_db():
    cred_file = os.path.expanduser('~/patsc/.lg_auth')
    if os.path.exists(cred_file):
        with open(cred_file, 'r') as creds_file:
            user = creds_file.readline().strip()
            passw = creds_file.readline().strip()
            return user, passw
    else:
        print('Error: ~/patsc/.lg_auth authorization not found')
        exit(1)


def retrieve_token():
    user, passw = read_cred_lg_db()
    task_str = 'grant_type=password&username=' + user + '&password=' + passw
    resp = requests.post('https://api.letsgrow.com/Token', data=task_str)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()['access_token']


def retrieve_module_templates(token):
    headers = {'Authorization': 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleTemplates', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()


def retrieve_modules(token):
    headers = {'Authorization': 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()


def retrieve_module_definition(token, module_id):
    headers = {'Authorization': 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()


def read_last_value(token, module_id, col_id):
    headers = {'Authorization': 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items/' + str(col_id) + '/LastValue', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()


def read_values(token, module_id, col_id, t0, t1):
    headers = {'Authorization': 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items/' + str(col_id) + '/Values?dateTimeStart=' + t0 + '&dateTimeEnd=' + t1, headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()


def write_values(token, module_id, col_id, name, json):
    if (len(json)):
        try:
            headers = {'Authorization': 'Bearer ' + token}
            resp = requests.put('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items/' + str(col_id) + '/Values', headers=headers, json=json)
            if resp.status_code != 200:
                print(name + ':')
                raise Exception('POST /tasks/ {}'.format(resp.status_code))
        except requests.exceptions.ConnectionError:
            print('LG throttling... Retrying in 1s\n\n')
            sleep(5)
            write_values(token, module_id, col_id, name, json)


def sys_info_table(token):
    # LG uses a module for each sensor (a system, in our case) (identified with a module_id),
    # which can deliver multiple data streams (e.g. temperature, humidity,... ) (identified
    # with a colId). Currently LG has added two datastreams per insect type for us, we have a unique
    # colId for 5-minute-binned insect counts (writable), and one for total insect counts (read-only).
    # The last one is useless as far as I'm concerned, but LG is fond of it anyway. For the
    # state of the system LG has added two datastreams as well, again we need to write to
    # 5-minute stream, and there is still the old insect data stream that we now use for total instects.
    # In order to write data, we need to specify both the colId and the moduleId.
    # This function creates a dictionairy that puts all the id data together,
    # later to be combined with our system_database (which contains the moduleId's per system).
    # To filter out the colIds we check the binning type (5 minutes = 4) and whether that column
    # is writable. The ItemCode should translate to a insect type or to MOTHNU for all insects or
    # PATSONOF for the system data.

    modules = retrieve_modules(token)

    LG_lookup = {}
    for module in modules:
        LG_dict = {'lg_module_id': module['Id'], 'name': module['Name'], }
        LG_dict['start_date'] = datetime.datetime.strptime(module['StartDate'], '%Y-%m-%dT%H:%M:%S')
        LG_dict['expiration_date'] = datetime.datetime.strptime(module['ExpirationDate'], '%Y-%m-%dT%H:%M:%S')

        sys = retrieve_module_definition(token, module['Id'])
        for module_item in sys['ModuleItems']:
            if module_item['ItemType'] == 4:
                if module_item['IsWriteable']:
                    LG_dict[module_item['ItemCode']] = module_item['ColId']
                else:
                    raise ValueError('Something is not writable that should be writable. Complain at LG. Mentiong the writEable spelling error while you are at it.')

        LG_lookup[module['Id']] = LG_dict
    return LG_lookup


def process_insects_in_json(data, sys_info):

    # LG wants 5 minute binned data, so we need to align our window with that
    insects = pd.DataFrame(data['moths'])
    LG_data: Dict[str, List] = {}
    if insects.empty:
        LG_data['MOTHNU'] = []
        return LG_data

    insects['time'] = pd.to_datetime(insects['time'], format='%Y%m%d_%H%M%S')
    insects = insects.loc[(insects['time'] > sys_info['start_date']) & (insects['time'] < sys_info['expiration_date'])]
    if 'Monster' in insects:
        insects = insects.loc[insects['Monster'] != 1]
        monsters = insects.loc[insects['Monster'] == 1]
        insects = insects[insects.apply(patsc.true_positive, axis=1)]
        if not insects.empty:
            insects = patsc.window_filter_monster(insects, monsters)
    else:
        insects = insects[insects.apply(patsc.true_positive, axis=1)]

    insects_info = patsc.get_insects_for_system(sys_info['system'].lower())
    for insect_name, insect_min, insect_max, insect_floodfill_min, insect_floodfill_max in insects_info:
        right_insects = pd.DataFrame()
        if 'Size' in insects:
            if patsc.check_verion(data['version'], '1.10'):
                right_insects = insects.loc[(insects['Size'] >= insect_floodfill_min) & (insects['Size'] <= insect_floodfill_max), ['time', 'duration']]  # after this step we only need the time, we keep to columns otherwise the returned type is different
            else:
                right_insects = insects.loc[(insects['Size'] >= insect_min) & (insects['Size'] <= insect_max), ['time', 'duration']]  # after this step we only need the time, we keep to columns otherwise the returned type is different

        if not right_insects.empty:
            binned_insects = right_insects.resample(str(lg_bin_width) + 'T', on='time').count()  # T means minute for some reason
            LG_data[insect_name] = pd.DataFrame({'Offset': 0.0, 'TimeStamp': binned_insects.index.strftime('%Y-%m-%dT%H:%M:%S'), 'Value': binned_insects.values[:, 0]}).to_dict('records')
        else:
            LG_data[insect_name] = []
    if not insects.empty:
        binned_insects = insects.resample(str(lg_bin_width) + 'T', on='time').count()
        LG_data['MOTHNU'] = pd.DataFrame({'Offset': 0.0, 'TimeStamp': binned_insects.index.strftime('%Y-%m-%dT%H:%M:%S'), 'Value': binned_insects.values[:, 0]}).to_dict('records')
    else:
        LG_data['MOTHNU'] = []

    return LG_data


def process_mode_in_json(data, sys_info):

    # LG wants 5 minute binned data, so we need to align our window with that
    t0 = sys_info['expiration_date']
    t1 = sys_info['start_date']
    for entry in data['mode']:
        sub_entries = []  # there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]

        for status in sub_entries:
            t_from = datetime.datetime.strptime(status['from'], '%Y%m%d_%H%M%S')
            if t_from > sys_info['start_date']:  # LG date subscription boundries
                if t_from < t0:
                    t0 = t_from
            t_till = datetime.datetime.strptime(status['till'], '%Y%m%d_%H%M%S')
            if t_till > sys_info['expiration_date']:
                t1 = sys_info['expiration_date']
            elif t_till > t1:
                t1 = t_till

    t0 = t0 - datetime.timedelta(seconds=t0.second, minutes=t0.minute - math.floor(t0.minute / lg_bin_width) * lg_bin_width)
    t1 = t1 - datetime.timedelta(seconds=t1.second, minutes=t1.minute - math.ceil(datetime.timedelta(seconds=t1.second, minutes=t1.minute).total_seconds() / (60 * lg_bin_width)) * lg_bin_width)
    bin_cnt = math.ceil((t1 - t0).total_seconds() / (60 * lg_bin_width))
    if bin_cnt <= 0:
        return []

    bins = [0] * bin_cnt
    for entry in data['mode']:
        sub_entries = []  # there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]

        for status in sub_entries:
            dt_start = datetime.datetime.strptime(status['from'], '%Y%m%d_%H%M%S') - t0
            id_start = math.floor(dt_start.total_seconds() / 300)
            id_start = int(np.clip(id_start, 0, bin_cnt))
            dt_final = datetime.datetime.strptime(status['till'], '%Y%m%d_%H%M%S') - t0
            id_final = math.floor(dt_final.total_seconds() / 300)
            id_final = int(np.clip(id_final, 0, bin_cnt))
            for i in range(id_start, id_final):
                if status['mode'] == 'op_mode_monitoring' or status['mode'] == 'monitoring':
                    bins[i] = 1
                elif status['mode'] == 'wait_for_dark':
                    bins[i] = 2
                elif status['mode'] == 'error':
                    bins[i] = 4
                else:
                    bins[i] = 4  # this should not be possible

    LG_data = []
    times = [(t0 + datetime.timedelta(minutes=lg_bin_width) * x).strftime('%Y-%m-%dT%H:%M:%S') for x in range(bin_cnt)]
    for i in range(0, len(bins)):
        LG_data.append({'Offset': 0.0, 'TimeStamp': times[i], 'Value': bins[i]})

    return LG_data


def upload_json_to_LG(token, json_data, sys_info, dry_run):
    binned_insect_data = process_insects_in_json(json_data, sys_info)
    binned_mode_data = process_mode_in_json(json_data, sys_info)
    if (len(binned_insect_data) or len(binned_mode_data)) and not dry_run:
        for insect_type in binned_insect_data.keys():
            if insect_type in sys_info:
                write_values(token, sys_info['lg_module_id'], sys_info[insect_type], sys_info['name'], binned_insect_data[insect_type])
            else:
                print(sys_info['name'] + ' does not know ' + insect_type)
        write_values(token, sys_info['lg_module_id'], sys_info['PATSONOF'], sys_info['name'], binned_mode_data)
        return 'OK'
    elif dry_run:
        return 'DRY RUN'
    else:
        return 'WARNING: EMPTY'


def load_system_info():
    sql_str = '''SELECT system,active,LG FROM systems JOIN customers ON customers.customer_id = systems.customer_id WHERE active = 1 AND LG'''
    with patsc.open_systems_db() as con:
        systems = pd.read_sql_query(sql_str, con)
        systems['system'] = systems['system'].str.upper()

    return systems


def jsons_to_LG(input_folder, dry_run=False):
    token = retrieve_token()
    LG_lookup = sys_info_table(token)
    systems_df = load_system_info()
    LG_404_err = False
    files = patsc.natural_sort([fp for fp in glob.glob(os.path.expanduser(input_folder + '/*.json'))])
    pbar = tqdm(files)
    for filename in pbar:
        pbar.set_description('LG upload: ' + os.path.basename(filename))
        flag_fn = filename[:-4] + 'LG_processed'
        if not os.path.exists(flag_fn):
            with open(filename) as json_file:
                with open(flag_fn, 'w') as flag_f:
                    if os.stat(filename).st_size < 40000000:
                        try:
                            json_data = json.load(json_file)
                            sys_name = json_data['system'].replace('-proto', '').upper()
                            if sys_name in systems_df['system'].values:
                                sys_info = systems_df.loc[systems_df['system'] == sys_name]
                                if sys_info.iloc[0]['active'] and sys_info.iloc[0]['LG']:
                                    min_required_version = 1.0
                                    if 'version' in json_data and float(json_data['version']) >= min_required_version:
                                        if int(sys_info['LG']) in LG_lookup:
                                            sys_LG = LG_lookup[int(sys_info['LG'])]
                                            sys_info = {**sys_info.to_dict('records')[0], **sys_LG}
                                            try:
                                                res = upload_json_to_LG(token, json_data, sys_info, dry_run)
                                            except Exception as e:
                                                if '404' in e.args[0]:
                                                    LG_404_err = True
                                                else:
                                                    print(e)
                                                    exit(1)  # until we have developped proper logging #871

                                            else:
                                                flag_f.write(res + '\n')
                                        else:
                                            flag_f.write('GETTING INFORMATION FROM LG WENT WRONG: ' + sys_name + '\n')
                                    else:
                                        flag_f.write('WRONG VERSION ' + json_data['version'] + '. Required: ' + min_required_version + '\n')
                                else:
                                    flag_f.write('SYSTEM NO LG OR INACTIVE: ' + sys_name + '\n')
                            else:
                                flag_f.write('SYSTEM UNKOWN: ' + sys_name + '\n')
                        except JSONDecodeError:
                            flag_f.write('JSONDecodeError\n')
                    else:
                        flag_f.write('File size too big\n')
        if LG_404_err:
            if os.path.exists(flag_fn):
                os.remove(flag_fn)
            print('LG server seems to be down... aborting')
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
    parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/jsons/')
    parser.add_argument('--dry-run', help="Run script now without sending mail", dest='dry_run', action='store_true')
    args = parser.parse_args()

    jsons_to_LG(args.input_folder, args.dry_run)
