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
    # with a colId). Currently LG has added two datastreams per detection type for us, we have a unique
    # colId for 5-minute-binned detection counts (writable), and one for total detection counts (read-only).
    # The last one is useless as far as I'm concerned, but LG is fond of it anyway. For the
    # state of the system LG has added two datastreams as well, again we need to write to
    # 5-minute stream, and there is still the old detection data stream that we now use for total instects.
    # In order to write data, we need to specify both the colId and the moduleId.
    # This function creates a dictionairy that puts all the id data together,
    # later to be combined with our system_database (which contains the moduleId's per system).
    # To filter out the colIds we check the binning type (5 minutes = 4) and whether that column
    # is writable. The ItemCode should translate to a detection type or to MOTHNU for all detections or
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


def process_detections_in_json(data, sys_info):

    # LG wants 5 minute binned data, so we need to align our window with that
    if not patsc.check_verion(data['version'], '2'):  # legacy v1
        detections = pd.DataFrame(data["moths"])
        detections.columns = map(str.lower, detections.columns)
        detections = detections.rename(columns={'time': 'start_datetime'})
    else:
        detections = pd.DataFrame(data["detections"])
    LG_data: Dict[str, List] = {}
    if detections.empty:
        LG_data['MOTHNU'] = []
        return LG_data

    detections['start_datetime'] = pd.to_datetime(detections['start_datetime'], format='%Y%m%d_%H%M%S')
    detections = detections.loc[(detections['start_datetime'] > sys_info['start_date']) & (detections['start_datetime'] < sys_info['expiration_date'])]
    if 'monster' in detections:
        detections = detections.loc[detections['monster'] != 1]
        monsters = detections.loc[detections['monster'] == 1]
        detections = detections[detections.apply(patsc.true_positive, axis=1)]
        if not detections.empty and not monsters.empty:
            detections = patsc.window_filter_monster(detections, monsters)
    else:
        detections = detections[detections.apply(patsc.true_positive, axis=1)]

    detection_classes = patsc.detection_classes(sys_info['system'].lower())
    for detection_name, detection_min, detection_max, detection_floodfill_min, detection_floodfill_max in detection_classes:
        right_detections = pd.DataFrame()
        if 'size' in detections:
            if patsc.check_verion(data['version'], '1.10'):
                right_detections = detections.loc[(detections['size'] >= detection_floodfill_min) & (detections['size'] <= detection_floodfill_max), ['start_datetime', 'duration']]  # after this step we only need the time, we keep to columns otherwise the returned type is different
            else:
                right_detections = detections.loc[(detections['size'] >= detection_min) & (detections['size'] <= detection_max), ['start_datetime', 'duration']]  # after this step we only need the time, we keep to columns otherwise the returned type is different

        if not right_detections.empty:
            binned_detections = right_detections.resample(str(lg_bin_width) + 'T', on='start_datetime').count()  # T means minute for some reason
            LG_data[detection_name] = pd.DataFrame({'Offset': 0.0, 'TimeStamp': binned_detections.index.strftime('%Y-%m-%dT%H:%M:%S'), 'Value': binned_detections.values[:, 0]}).to_dict('records')
        else:
            LG_data[detection_name] = []
    if not detections.empty:
        binned_detections = detections.resample(str(lg_bin_width) + 'T', on='start_datetime').count()
        LG_data['MOTHNU'] = pd.DataFrame({'Offset': 0.0, 'TimeStamp': binned_detections.index.strftime('%Y-%m-%dT%H:%M:%S'), 'Value': binned_detections.values[:, 0]}).to_dict('records')
    else:
        LG_data['MOTHNU'] = []

    return LG_data


def process_status_in_json(data, sys_info):

    # LG wants 5 minute binned data, so we need to align our window with that
    t0 = sys_info['expiration_date']
    t1 = sys_info['start_date']

    if not patsc.check_verion(data['version'], '2'):  # legacy v1
        start_datetime_column_name = 'from'
        end_datetime_id_str = 'till'
    else:
        start_datetime_column_name = 'start_datetime'
        end_datetime_id_str = 'end_datetime'

    for entry in data['mode']:
        sub_entries = []  # there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]

        for status in sub_entries:
            start_datetime = datetime.datetime.strptime(status[start_datetime_column_name], '%Y%m%d_%H%M%S')
            if start_datetime > sys_info['start_date']:  # LG date subscription boundries
                if start_datetime < t0:
                    t0 = start_datetime
            end_datetime = datetime.datetime.strptime(status[end_datetime_id_str], '%Y%m%d_%H%M%S')
            if end_datetime > sys_info['expiration_date']:
                t1 = sys_info['expiration_date']
            elif end_datetime > t1:
                t1 = end_datetime

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
            dt_start = datetime.datetime.strptime(status[start_datetime_column_name], '%Y%m%d_%H%M%S') - t0
            id_start = math.floor(dt_start.total_seconds() / 300)
            id_start = int(np.clip(id_start, 0, bin_cnt))
            dt_final = datetime.datetime.strptime(status[end_datetime_id_str], '%Y%m%d_%H%M%S') - t0
            id_final = math.floor(dt_final.total_seconds() / 300)
            id_final = int(np.clip(id_final, 0, bin_cnt))
            for i in range(id_start, id_final):
                if status['mode'] == 'op_mode_monitoring' or status['mode'] == 'monitoring' or status['mode'] == 'op_mode_c':
                    bins[i] = 1
                elif status['mode'] == 'wait_for_dark':
                    bins[i] = 2
                elif status['mode'] == 'error':
                    bins[i] = 4
                elif status['mode'] == 'op_mode_x':
                    bins[i] = 5
                else:
                    bins[i] = 66  # this should not be possible.
                    print("Error, some weird mode detected...?")  # until #1202

    LG_data = []
    times = [(t0 + datetime.timedelta(minutes=lg_bin_width) * x).strftime('%Y-%m-%dT%H:%M:%S') for x in range(bin_cnt)]
    for i in range(0, len(bins)):
        LG_data.append({'Offset': 0.0, 'TimeStamp': times[i], 'Value': bins[i]})

    return LG_data


def upload_json_to_LG(token, json_data, sys_info, dry_run):
    binned_detection_data = process_detections_in_json(json_data, sys_info)
    binned_status_data = process_status_in_json(json_data, sys_info)
    if (len(binned_detection_data) or len(binned_status_data)) and not dry_run:
        for detection_type in binned_detection_data.keys():
            if detection_type in sys_info:
                write_values(token, sys_info['lg_module_id'], sys_info[detection_type], sys_info['name'], binned_detection_data[detection_type])
            else:
                print(sys_info['name'] + ' does not know ' + detection_type)
        write_values(token, sys_info['lg_module_id'], sys_info['PATSONOF'], sys_info['name'], binned_status_data)
        return 'OK'
    elif dry_run:
        return 'DRY RUN'
    else:
        return 'WARNING: EMPTY'


def load_system_info():
    sql_str = '''SELECT system,active,lg FROM systems JOIN customers ON customers.customer_id = systems.customer_id WHERE active = 1 AND lg'''
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
        if not os.path.exists(flag_fn) or dry_run:
            with open(filename) as json_file:
                with open(flag_fn, 'w') as flag_f:
                    if os.stat(filename).st_size < 40000000:
                        try:
                            json_data = json.load(json_file)
                            sys_name = json_data['system'].replace('-proto', '').upper()
                            if sys_name in systems_df['system'].values:
                                sys_info = systems_df.loc[systems_df['system'] == sys_name]
                                if sys_info.iloc[0]['active'] and sys_info.iloc[0]['lg']:
                                    min_required_version = '1.0'
                                    if 'version' in json_data and patsc.check_verion(json_data['version'] , min_required_version):
                                        if int(sys_info['lg']) in LG_lookup:
                                            sys_LG = LG_lookup[int(sys_info['lg'])]
                                            sys_info = {**sys_info.to_dict('records')[0], **sys_LG}
                                            try:
                                                res = upload_json_to_LG(token, json_data, sys_info, dry_run)
                                            except Exception as e:
                                                if '404' in e.args[0]:
                                                    LG_404_err = True
                                                else:
                                                    print(e)
                                                    exit(1)  # until we have developped proper logging #871 --> we have, so #1202

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
    parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/patsc/jsons/')
    parser.add_argument('--dry-run', help="Run script now without sending mail", dest='dry_run', action='store_true')
    args = parser.parse_args()

    jsons_to_LG(args.input_folder, args.dry_run)
