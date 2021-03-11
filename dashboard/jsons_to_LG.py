#!/usr/bin/env python3
import requests,json,datetime,glob,os,math
import pandas as pd
import numpy as np
from tqdm import tqdm
from time import sleep
from json.decoder import JSONDecodeError
import lib_patsc as patsc

def read_cred_lg_db():
    cred_file = os.path.expanduser('~/patsc/.lg_auth')
    if os.path.exists(cred_file):
        with open (cred_file, 'r') as creds_file:
                user = creds_file.readline().strip()
                passw = creds_file.readline().strip()
                return user,passw
    else:
        print('Error: ~/patsc/.lg_auth authorization not found')
        exit(1)

def retrieve_token():
    user,passw = read_cred_lg_db()
    task_str = 'grant_type=password&username=' + user + '&password=' + passw
    resp = requests.post('https://api.letsgrow.com/Token', data=task_str)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()['access_token']

def retrieve_module_templates(token):
    headers = { 'Authorization' : 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleTemplates', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()

def retrieve_modules(token):
    headers = { 'Authorization' : 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()

def retrieve_module_definition(token,module_id):
    headers = { 'Authorization' : 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()

def read_last_value(token,module_id,col_id):
    headers = { 'Authorization' : 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items/' + str(col_id) +'/LastValue', headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()

def read_values(token,module_id,col_id,t0,t1):
    headers = { 'Authorization' : 'Bearer ' + token}
    resp = requests.get('https://api.letsgrow.com/api/ModuleDefinitions/' + str(module_id) + '/Items/' + str(col_id) +'/Values?dateTimeStart=' + t0 + '&dateTimeEnd=' + t1, headers=headers)
    if resp.status_code != 200:
        raise Exception('POST /tasks/ {}'.format(resp.status_code))
    return resp.json()

def write_values(token,module_id,col_id,name,json):
    if (len(json)):
        try:
            headers = { 'Authorization' : 'Bearer ' + token}
            resp = requests.put('https://api.letsgrow.com/api/ModuleDefinitions/'+ str(module_id) + '/Items/' + str(col_id) + '/Values',headers=headers,json=json)
            if resp.status_code != 200:
                print(name + ':')
                raise Exception('POST /tasks/ {}'.format(resp.status_code))
        except requests.exceptions.ConnectionError as e:
            print('LG throttling... Retrying in 1s\n\n')
            sleep(5)
            write_values(token,module_id,col_id,name,json)

def sys_info_table(token):
    #LG uses a module for each sensor (a system, in our case) (identified with a module_id),
    #which can deliver multiple data streams (e.g. temperature, humidity,... ) (identified
    #with a colId). Currently LG has added two moth datastreams for us, we have a unique
    #colId for 5-minute-binned moth counts (writable), and one for total moth counts (read-only).
    #The last one is useless as far as I'm concerned, but LG is fond of it anyway. For the
    #state of the system LG has added a few datastreams as well, again we need to write to
    #5-minute stream. In order to write data, we need to specify both the colId and the moduleId.
    #This function creates a table that puts all the id data together,
    #later to be combined with our system_database (which contains the moduleId's per system).
    #To filter out the colIds we use the ItemCode MOTHNU and PATSONOF, which should be unique
    #for the moth data and system status streams, together with the ItemType, which translates
    #to the binning type (5 minutes = 4).
    modules = retrieve_modules(token)
    columns=['name','lg_module_id','moth_col_id','onoff_col_id','start_date','expiration_date'] #we convert to our case style here

    LG_df = pd.DataFrame(columns=columns)
    LG_df['start_date']= pd.to_datetime(LG_df['start_date'])
    LG_df['expiration_date']= pd.to_datetime(LG_df['expiration_date'])
    for module in modules:
            sys = retrieve_module_definition(token,module['Id'])
            for module_item in sys['ModuleItems']:
                lg_module_ids = LG_df['lg_module_id'].values.tolist()
                if module['Id'] in lg_module_ids:
                    existing_module_row_id = lg_module_ids.index(module['Id'])
                else:
                    existing_module_row_id = -1

                if module_item['ItemCode'] == 'MOTHNU' and module_item['ItemType'] == 4:
                    row = pd.DataFrame([[module['Name'],module['Id'],module_item['ColId'],'?',datetime.datetime.strptime(module['StartDate'], '%Y-%m-%dT%H:%M:%S'),datetime.datetime.strptime(module['ExpirationDate'], '%Y-%m-%dT%H:%M:%S')]],columns=columns)
                    if existing_module_row_id >= 0:
                        existing_module_row = LG_df.loc[LG_df['lg_module_id'] == module['Id']]
                        if existing_module_row['start_date'][0] != row['start_date'][0] or existing_module_row['expiration_date'][0] != row['expiration_date'][0]:
                            raise ValueError('Dates between the MOTHNU and PATSONOF dont match! Complain at LG to make them them same.')
                        LG_df.iat[existing_module_row_id,LG_df.columns.get_loc('moth_col_id')] = module_item['ColId']
                    else:
                        LG_df = LG_df.append(row)
                elif module_item['ItemCode'] == 'PATSONOF' and module_item['ItemType'] == 4:

                    row = pd.DataFrame([[module['Name'],module['Id'],'?',module_item['ColId'],datetime.datetime.strptime(module['StartDate'], '%Y-%m-%dT%H:%M:%S'),datetime.datetime.strptime(module['ExpirationDate'], '%Y-%m-%dT%H:%M:%S')]],columns=columns)
                    if existing_module_row_id >= 0:
                        existing_module_row = LG_df.loc[LG_df['lg_module_id'] == module['Id']]
                        #print(module_item['CustomerDescription'] + ' ' + module_item['Description'] + ': ' + str(module['Id']) + ' onoff: ' + str(module_item['ColId']) + ' moth: ' + str(existing_module_row['moth_col_id'][0]))
                        if existing_module_row['start_date'][0] != row['start_date'][0] or existing_module_row['expiration_date'][0] != row['expiration_date'][0]:
                            raise ValueError('Dates between the MOTHNU and PATSONOF dont match! Complain at LG to make them them same.')
                        if not module_item['IsWriteable']:
                            raise ValueError('Something is not writable that should be writable. Complain at LG. Mentiong the writEable spelling error while you are at it.')
                        LG_df.iat[existing_module_row_id,LG_df.columns.get_loc('onoff_col_id')] = module_item['ColId']
                    else:
                        LG_df = LG_df.append(row)
    return LG_df

def process_moth_in_json(data,sys_info):

    #LG wants 5 minute binned data, so we need to align our window with that
    t0 = sys_info['expiration_date']
    t1 = sys_info['start_date']
    for moth in data['moths']:
        moth = patsc.clean_moth_json_entry(moth,data)
        if patsc.true_positive(moth,sys_info['minimal_size']):
            t = datetime.datetime.strptime(moth['time'], '%Y%m%d_%H%M%S')
            if t > sys_info['start_date'] and t < sys_info['expiration_date']: #LG date subscription boundries
                if t <t0:
                    t0 = t
                if t > t1:
                    t1 = t
    t00 = t0 - datetime.timedelta(seconds=t0.second,minutes=t0.minute - math.floor(t0.minute/5)*5)
    t11 = t1 - datetime.timedelta(seconds=t1.second,minutes=t1.minute - math.ceil(datetime.timedelta(seconds=t1.second,minutes=t1.minute ).total_seconds()/300)*5)
    bin_cnt = math.ceil((t11 - t00).total_seconds() / 300)

    bins = [0]*bin_cnt
    for moth in data['moths']:
        moth = patsc.clean_moth_json_entry(moth,data)
        if patsc.true_positive(moth,sys_info['minimal_size']):
            t = datetime.datetime.strptime(moth['time'], '%Y%m%d_%H%M%S')
            if t > sys_info['start_date'] and t < sys_info['expiration_date']:
                dt = t - t00
                id = math.floor(dt.total_seconds() / 300)
                if id  >= bin_cnt:
                    #this is an edge case that probably should go in the next bin, or increase the
                    # bin_cnt with one, but this means it will overlap with the next bin, which
                    # could be overwritten. Better to have a small 5 minute shift for this edge case...
                    id = bin_cnt-1
                bins[id] +=1

    LG_data = []
    times = [(t00+datetime.timedelta(minutes=5)*x ).strftime('%Y-%m-%dT%H:%M:%S') for x in range(bin_cnt)]
    for i in range(0,len(bins)):
        LG_data.append({'Offset': 0.0,'TimeStamp': times[i], 'Value' : bins[i]})

    return LG_data

def process_mode_in_json(data,sys_info):

    #LG wants 5 minute binned data, so we need to align our window with that
    t0 = sys_info['expiration_date']
    t1 = sys_info['start_date']
    for entry in data['mode']:
        sub_entries = [] #there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]

        for status in sub_entries:
            t_from = datetime.datetime.strptime(status['from'], '%Y%m%d_%H%M%S')
            if t_from > sys_info['start_date']: #LG date subscription boundries
                if t_from <t0:
                    t0 = t_from
            t_till = datetime.datetime.strptime(status['till'], '%Y%m%d_%H%M%S')
            if t_till > sys_info['expiration_date']:
                t1 = sys_info['expiration_date']
            elif t_till > t1:
                t1 = t_till

    t0 = t0 - datetime.timedelta(seconds=t0.second,minutes=t0.minute - math.floor(t0.minute/5)*5)
    t1 = t1 - datetime.timedelta(seconds=t1.second,minutes=t1.minute - math.ceil(datetime.timedelta(seconds=t1.second,minutes=t1.minute ).total_seconds()/300)*5)
    bin_cnt = math.ceil((t1 - t0).total_seconds() / 300)
    if bin_cnt <= 0:
        return []

    bins = [0]*bin_cnt
    for entry in data['mode']:
        sub_entries = [] #there may be another nested level here, in case of waiting for darkness
        if type(entry) == list:
            sub_entries = entry
        else:
            sub_entries = [entry]

        for status in sub_entries:
            dt_start = datetime.datetime.strptime(status['from'], '%Y%m%d_%H%M%S') - t0
            id_start = math.floor(dt_start.total_seconds() / 300)
            id_start = int(np.clip(id_start,0,bin_cnt))
            dt_final = datetime.datetime.strptime(status['till'], '%Y%m%d_%H%M%S') - t0
            id_final = math.floor(dt_final.total_seconds() / 300)
            id_final = int(np.clip(id_final,0,bin_cnt))
            for i in range(id_start,id_final):
                if status['mode'] == 'op_mode_monitoring' or status['mode'] == 'monitoring':
                    bins[i] = 1
                elif status['mode'] == 'wait_for_dark':
                    bins[i] = 2
                elif status['mode'] == 'error':
                    bins[i] = 4
                else:
                    bins[i] = 4 #this should not be possible

    LG_data = []
    times = [(t0+datetime.timedelta(minutes=5)*x ).strftime('%Y-%m-%dT%H:%M:%S') for x in range(bin_cnt)]
    for i in range(0,len(bins)):
        LG_data.append({'Offset': 0.0,'TimeStamp': times[i], 'Value' : bins[i]})

    return LG_data

def upload_json_to_LG(token,json_data,sys_info):
    binned_moth_data = process_moth_in_json(json_data,sys_info)
    binned_mode_data = process_mode_in_json(json_data,sys_info)
    if len(binned_moth_data) or len(binned_mode_data):
        write_values(token,sys_info['lg_module_id'],sys_info['moth_col_id'],sys_info['name'],binned_moth_data)
        write_values(token,sys_info['lg_module_id'],sys_info['onoff_col_id'],sys_info['name'],binned_mode_data)
        return 'OK'
    else:
        return 'WARNING: EMPTY'

def load_system_info():
    sys_db_con,sys_db_cur = patsc.open_db(os.path.expanduser('~/patsc/db/pats_systems.db'))
    sql_str = '''SELECT system,active,LG FROM systems'''
    systems = pd.read_sql_query(sql_str,sys_db_con)
    systems['system'] = systems['system'].str.replace('-proto' , '')
    systems['system'] = systems['system'].str.upper()

    sys_db_cur.execute('''SELECT systems.system, groups.minimal_size FROM systems JOIN groups ON groups.group_id = systems.group_id''')
    size_data = sys_db_cur.fetchall()
    systems['minimal_size'] = [item[1] for item in size_data]

    return systems

def jsons_to_LG(input_folder):
    token = retrieve_token()
    LG_lookup = sys_info_table(token)
    systems_df = load_system_info()

    files = patsc.natural_sort([fp for fp in glob.glob(os.path.expanduser(input_folder + '/*.json'))])
    pbar = tqdm(files)
    for filename in pbar:
        pbar.set_description('LG upload: '+ os.path.basename(filename))
        flag_fn = filename[:-4] + 'LG_processed'
        if not os.path.exists(flag_fn):
            with open(filename) as json_file:
                with open(flag_fn,'w') as flag_f:
                    if os.stat(filename).st_size < 40000000:
                        try:
                            json_data = json.load(json_file)
                            sys_name = json_data['system'].replace('-proto','').upper()
                            if sys_name in systems_df['system'].values:
                                sys_info = systems_df.loc[systems_df['system'] == sys_name]
                                if sys_info.iloc[0]['active'] and sys_info.iloc[0]['LG']:
                                    min_required_version=1.0
                                    if 'version' in json_data and float(json_data['version']) >= min_required_version:
                                        sys_LG = LG_lookup.loc[LG_lookup['lg_module_id'] == int(sys_info['LG'])]
                                        sys_info = pd.concat([sys_info.reset_index(drop=True), sys_LG.reset_index(drop=True)], axis=1)
                                        sys_info = sys_info.to_dict('records')[0]
                                        res = upload_json_to_LG(token,json_data,sys_info)
                                        flag_f.write(res + '\n')
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

if __name__ == "__main__":
    jsons_to_LG('~/jsons/')
