#!/usr/bin/env python3
import os,datetime,pathlib
import pandas as pd
import pats_c.lib.lib_patsc as patsc

insect = 'tomato_looper'
download_renders = False # leave to false because it uses a lot of data
download_raw_cuts = False  # leave to false because it REALLY uses a lot of data, only for wifi systems!
start_date = datetime.date.today() - datetime.timedelta(days=14)

if insect == 'tomato_looper': #(turkse mot)
    selected_systems = ['pats4','pats6']
    filter_str = f'''" AND time > "{start_date.strftime('%Y%m%d_%H%M%S')}"
    AND Dist_traveled > 0.15
    AND Dist_traveled < 4
    AND Size > 0.015'''
elif insect == 'tuta':
    selected_systems = ['pats52','pats66','pats68','pats69']
    filter_str = f'''" AND time > "{start_date.strftime('%Y%m%d_%H%M%S')}"
    AND Dist_traveled > 0.15
    AND Dist_traveled < 4
    AND Size > 0.01'''
elif insect == 'duponchelia':
    selected_systems = ['pats7']
    filter_str = f'''" AND time > "{start_date.strftime('%Y%m%d_%H%M%S')}"
    AND Dist_traveled > 0.15
    AND Dist_traveled < 4
    AND Size > 0.01'''
elif insect == 'stinky':
    selected_systems = ['pats53']
    filter_str = f'''" AND time >= "20210412_224000" AND time < "20210412_231500"
    AND Dist_traveled > 0.15
    AND Dist_traveled < 4
    AND Size > 0.01
    AND duration > 0.3 AND duration < 10'''
filter_str = filter_str.replace('\n','')

with patsc.open_data_db() as con:
    cur = con.cursor()
    for system in selected_systems:
        columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]
        sql_str = 'SELECT Folder,Filename FROM moth_records WHERE system="' + system + filter_str
        df = pd.read_sql_query(sql_str,con)

        download_list = df['Folder'] + '/logging/' + df['Filename']
        download_list.to_csv('files_to_be_tarred.tmp',index=False,header=False)
        rsync_upload_cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + 'files_to_be_tarred.tmp' + ' ' + system+':files_to_be_tarred.tmp']
        patsc.execute(rsync_upload_cmd,5)

        print('Tarring ' + str(len(download_list)) + ' logs on ' + system + '...')
        tar_cmd = ['ssh -o StrictHostKeyChecking=no -T ' + system +  ' "rm -rf  /home/pats/dataset_logs.tar.gz && cd /home/pats/pats/data/processed/ && tar --ignore-failed-read -czf /home/pats/dataset_logs.tar.gz --files-from=/home/pats/files_to_be_tarred.tmp"']
        if patsc.execute(tar_cmd,3) != 0:
            print('Error :(')
            exit(1)
        print('Downloading data from ' + system)
        target_path=os.path.expanduser('~/Downloads/pats_dataset/' + insect + '/')
        pathlib.Path(target_path).mkdir(parents=True, exist_ok=True)
        rsync_cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + system+':dataset_logs.tar.gz' + ' ' + target_path + system + '_dataset_logs.tar.gz']
        patsc.execute(rsync_cmd,5)

        print('Untarring ' + target_path + system + '_dataset_logs.tar.gz')
        pathlib.Path(target_path + system).mkdir(parents=True, exist_ok=True)
        untar_cmd = 'tar -xf ' + target_path + system + '_dataset_logs.tar.gz -C ' + target_path + system + '/'
        patsc.execute(untar_cmd)

        print('Clean up...')
        os.remove(target_path + system + '_dataset_logs.tar.gz')
        os.remove('files_to_be_tarred.tmp')
        cleanup_cmd = ['ssh -o StrictHostKeyChecking=no -T ' + system +  ' "rm -rf  /home/pats/dataset_logs.tar.gz && rm /home/pats/files_to_be_tarred.tmp"']
        patsc.execute(cleanup_cmd)