#!/usr/bin/env python3
import datetime
import pandas as pd
import numpy as np
import sys
import os
import pathlib
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc


def create_insect_filter(insect):
    if insect != 'all':
        with patsc.open_systems_db() as con:
            sql_str = f'''SELECT floodfill_avg_size,floodfill_std_size FROM insects WHERE name LIKE '%{insect}%' '''
            avg_size, std_size = con.execute(sql_str).fetchone()
            min_size = avg_size - std_size
            max_size = avg_size + 2 * std_size
            return f''' (Dist_traveled > 0.15 AND Dist_traveled < 4 AND Size > {min_size} AND Size < {max_size} OR Monster = 1) '''
    return ''' (Dist_traveled > 0.15 AND Dist_traveled < 4 OR Monster = 1) '''


def create_time_filter(start_date: datetime.datetime, end_date: datetime.datetime, hour_str):
    time_filter = f''' time > '{(start_date).strftime('%Y%m%d_%H%M%S')}' AND time < '{(end_date).strftime('%Y%m%d_%H%M%S')}' '''
    if hour_str:
        time_filter += f''' AND ( {' OR '.join([f"time LIKE '_________{str(hour)}____' "for hour in hour_str])} )'''

    return time_filter


def window_filter_monster(insect_df, monster_df):
    monster_times = monster_df['time'].values
    insect_times = insect_df['time'].values
    monster_times_mesh, insect_times_mesh = np.meshgrid(monster_times, insect_times, sparse=True)
    insects_with_monsters = np.sum((monster_times_mesh >= insect_times_mesh - pd.Timedelta(minutes=5)) * (monster_times_mesh <= insect_times_mesh + pd.Timedelta(minutes=5)), axis=1)
    insect_without_monsters = insect_df.iloc[insects_with_monsters == 0]
    return insect_without_monsters


def get_insects(system, insect, start_date: datetime.datetime, end_date: datetime.datetime, hour_str=[]):
    print('Getting insects from ' + system + ' ...')
    insect_filter = create_insect_filter(insect)
    time_filer = create_time_filter(start_date - datetime.timedelta(minutes=5), end_date + datetime.timedelta(minutes=5), hour_str)
    sql_str = f'''SELECT time, Monster, Folder, Filename FROM moth_records WHERE system = '{str(system)}' AND {insect_filter} AND {time_filer} '''
    with patsc.open_data_db() as con:
        insect_df = pd.read_sql_query(sql_str, con)

    insect_df['time'] = pd.to_datetime(insect_df['time'], format='%Y%m%d_%H%M%S')
    monster_df = insect_df.loc[insect_df['Monster'] == 1]
    insect_df = insect_df.loc[insect_df['Monster'] != 1]
    insect_df = window_filter_monster(insect_df, monster_df)
    insect_df = insect_df[(insect_df['time'] > start_date) & (insect_df['time'] < end_date)]

    return insect_df


def get_insect_log_paths(system, insect_df):
    print('Getting download list for ' + system + ' ...')
    insect_folders = insect_df['Folder']
    insect_logs = insect_df['Filename']
    download_list = insect_folders + '/logging/' + insect_logs
    return download_list


def tar_insect_logs(system, download_list):
    download_list.to_csv('files_to_be_tarred.tmp', index=False, header=False)
    rsync_upload_cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + 'files_to_be_tarred.tmp' + ' ' + system + ':files_to_be_tarred.tmp']
    patsc.execute(rsync_upload_cmd, 5)
    print('Tarring ' + str(len(download_list)) + ' logs on ' + system + '...')
    tar_cmd = ['ssh -o StrictHostKeyChecking=no -T ' + system + ' "rm -rf  /home/pats/dataset_logs.tar.gz && cd /home/pats/pats/data/processed/ && tar --ignore-failed-read -czf /home/pats/dataset_logs.tar.gz --files-from=/home/pats/files_to_be_tarred.tmp"']
    if patsc.execute(tar_cmd, 3) != 0:
        print('Error :(')
        exit(1)


def download_insect_tar(system, target_path):
    print('Downloading data from ' + system)
    pathlib.Path(target_path).mkdir(parents=True, exist_ok=True)
    rsync_cmd = ['rsync --timeout=5 -az -P -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + system + ':dataset_logs.tar.gz' + ' ' + target_path + system + '_dataset_logs.tar.gz']
    patsc.execute(rsync_cmd, 5)


def untar_insects_logs(system, target_path):
    print('Untarring ' + target_path + system + '_dataset_logs.tar.gz')
    pathlib.Path(target_path + system).mkdir(parents=True, exist_ok=True)
    untar_cmd = 'tar -xf ' + target_path + system + '_dataset_logs.tar.gz -C ' + target_path + system + '/'
    patsc.execute(untar_cmd)


def clean_up(system, target_path):
    print('Clean up...')
    os.remove(target_path + system + '_dataset_logs.tar.gz')
    os.remove('files_to_be_tarred.tmp')
    cleanup_cmd = ['ssh -o StrictHostKeyChecking=no -T ' + system + ' "rm -rf  /home/pats/dataset_logs.tar.gz && rm /home/pats/files_to_be_tarred.tmp"']
    patsc.execute(cleanup_cmd)


today = datetime.datetime.now()
sept24 = datetime.datetime(year=2021, month=9, day=24)  # This is the date that the monster branch got active
startwintertime = datetime.datetime(year=2021, month=10, day=31)
data_to_get = [  # {'system': 'pats32', 'insect': 'all', 'start_date': datetime.datetime(year=2021, month=10, day=21), 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats9', 'insect': 'Turkse', 'start_date': datetime.datetime(year=2021, month=9, day=1), 'end_date': sept24, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats73', 'insect': 'Turkse', 'start_date': datetime.datetime(year=2021, month=9, day=1), 'end_date': sept24, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats51', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats29', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats110', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats111', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats112', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats113', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats113', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats21', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats2', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats22', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats3', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats47', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats48', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats49', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats75', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats64', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats11', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats12', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats70', 'insect': 'Turkse', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20']},
    {'system': 'pats50', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats93', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats94', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats54', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats28', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats74', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats55', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats56', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats57', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats58', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats1', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats124', 'insect': 'Dupo', 'start_date': sept24, 'end_date': today, 'hour_str': ['18', '19', '20', '21']},
    {'system': 'pats32', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today, 'hour_str': ['17', '18', '19', '20', '21', '22', '23', '00', '01', '02', '03', '04', '05', '06', '07']},  # doet traag + warnings?
    {'system': 'pats104', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats98', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats99', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats100', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats101', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats102', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats63', 'insect': 'Tuta', 'start_date': sept24, 'end_date': today},
    {'system': 'pats127', 'insect': 'Kool', 'start_date': sept24, 'end_date': today},
    {'system': 'pats16', 'insect': 'Kool', 'start_date': sept24, 'end_date': today},
    {'system': 'pats17', 'insect': 'Kool', 'start_date': sept24, 'end_date': today},
    {'system': 'pats7', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
    {'system': 'pats71', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
    {'system': 'pats59', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
    {'system': 'pats45', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
    {'system': 'pats46', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
    {'system': 'pats60', 'insect': 'Opogona', 'start_date': sept24, 'end_date': today, 'hour_str': ['01', '02', '03', '04']},
]

for data in data_to_get:
    insect_df = get_insects(**data)
    download_list = get_insect_log_paths(data['system'], insect_df)
    tar_insect_logs(data['system'], download_list)
    target_path = os.path.expanduser('~/Downloads/pats_dataset/' + data['insect'] + '/')
    download_insect_tar(data['system'], target_path)
    untar_insects_logs(data['system'], target_path)
    clean_up(data['system'], target_path)
