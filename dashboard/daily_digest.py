#!/usr/bin/env python3
import socket, json,shutil,sqlite3,subprocess
import time, argparse
import pickle, glob, os, re
from datetime import datetime, timedelta, date
import pandas as pd

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def load_systems():
    conn = sqlite3.connect(os.path.expanduser(args.db_path))
    cur = conn.cursor()
    sql_str = '''SELECT system_name,maintenance FROM systems WHERE is_active = 1 ORDER BY system_id'''
    cur.execute(sql_str)
    systems = pd.read_sql_query(sql_str,conn)
    return systems

def write_systems(changed_systems):
    conn = sqlite3.connect(os.path.expanduser(args.db_path))
    cur = conn.cursor()
    sql_str = '''UPDATE systems SET first_warning=?, maintenance=? WHERE system_name=? '''
    for sys,date,maintenance in changed_systems:
        cur.execute(sql_str,(date,maintenance,sys))
    conn.commit()
    conn.close()

def execute(cmd):
    p_result = None
    n=0

    popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        p_result = popen.poll()
        if p_result != None:
            n = n+1
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()


def send_mail(now):
    files = natural_sort([fp for fp in glob.glob(os.path.expanduser(args.input_folder + "/*.processed"))])
    systems = load_systems()
    mail_txt = ''
    recent_files = []
    for file in files:
        f = os. path. splitext(os.path.basename((file)))[0]
        if f.startswith('pats'):
            f_date = f.split('_')[-2] + '_' + f.split('_')[-1]
            try:
                d = datetime.strptime(f_date, "%Y%m%d_%H%M%S")
                f_sys = f.split('_')[0]

                if d > now - timedelta(hours=5):

                    with open (file, "r") as fr_processed:
                        if f_sys in systems['system_name'].values:
                            msg = fr_processed.readline()
                            recent_files.append([f_sys,msg])
                            systems = systems.drop(systems[systems['system_name']==f_sys].index)
                        else:
                            print('Warning, system does not exist: ' + f_sys)
            except:
                pass
    mail_txt = 'Pats status report ' + str(now) + '\n\n'
    if (len(systems)>0):
        mail_txt += 'WARNINGS: \nNothing received from: '
        for sys,maintenance in systems.to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance,'%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        recent_files.append([sys,'OK. under maintenance'])
                    else:
                        mail_txt += sys + ', '
                except:
                    mail_txt += sys + ': date_error, '
            else:
                mail_txt += sys + ', '
        mail_txt = mail_txt[:-2] + '\n'
        for sys,message in recent_files:
            if not message.startswith('OK.'):
                mail_txt += sys + ': ' + message + '\n'
        mail_txt += '\n'

    recent_files = natural_sort_systems(recent_files)
    mail_txt += 'System status:\n'
    for sys,message in recent_files:
        if message.startswith('OK. '):
            mail_txt += sys + ':\t' + message.replace('OK.','') + '\n'

    print(mail_txt)
    with open('mail.tmp','w') as mail_f:
        mail_f.write(mail_txt)

    cmd = 'mail -s "Pats daily status digest" sys-maintenance@pats-drones.com < mail.tmp'
    execute(cmd)

parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/jsons/')
parser.add_argument('-t','--hour', help="Send email at the start of this hour.", default=11)
parser.add_argument('-m', help="Mail address to send to.", default='monitoring@pats-drones.com')
parser.add_argument('-d','--db_path', help="Path to sql database", default='~/pats_systems.db')
args = parser.parse_args()

updated_today = False
while  True:
    now = datetime.now()
    if now.hour == int(args.hour) and not updated_today:
        updated_today = True
        send_mail(now)

    if now.hour == int(args.hour)+1 and updated_today:
        updated_today = False
    time.sleep(10)