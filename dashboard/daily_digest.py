#!/usr/bin/env python3
import socket, json,shutil,sqlite3,subprocess
import time, argparse
import pickle, glob, os, re
from datetime import datetime, timedelta, date
import numpy as np

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
    sql_str = '''SELECT system_name,first_warning,maintenance FROM systems WHERE is_active = 1 ORDER BY system_id'''
    cur.execute(sql_str)
    systems = np.array(cur.fetchall(),dtype=object)
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
    changed_systems = []
    in_progress = []
    under_maintenance = []
    for file in files:
        f = os. path. splitext(os.path.basename((file)))[0]
        if f.startswith('pats'):
            f_date = f.split('_')[-2] + '_' + f.split('_')[-1]
            try:
                d = datetime.strptime(f_date, "%Y%m%d_%H%M%S")
                f_sys = f.split('_')[0]

                if d > now - timedelta(hours=5):

                    with open (file, "r") as fr_processed:
                        if f_sys in systems[:,0]:
                            msg = fr_processed.readline()
                            recent_files.append([f_sys,msg])
                            full_system = systems[systems[:,0]==f_sys][0]
                            if full_system[1] is not None or full_system[2]:
                                changed_systems.append((full_system[0],None,0))
                            systems = np.delete(systems, np.argwhere(systems[:,0]==f_sys), axis= 0)
                        else:
                            print('Warning, system does not exist: ' + f_sys)
            except:
                pass
    mail_txt = 'Pats status report ' + str(now) + '\n\n'
    if (len(systems)>0):
        mail_txt += 'WARNINGS: \nNothing received from: '
        for (sys,date,maintenance) in systems:
            if not date:
                date = datetime.today().strftime('%Y%m%d')
                mail_txt += sys + ', '
                changed_systems.append((sys,date,maintenance))
            elif maintenance:
                date = datetime.strptime(date,'%Y%m%d')
                if datetime.today() - date <= timedelta(days=3):
                    in_progress.append(sys)
                else:
                    under_maintenance.append(sys)
            else:
                mail_txt += sys + ', '
        mail_txt = mail_txt[:-2] + '\n'
        for sys in recent_files:
            if not sys[1].startswith('OK.'):
                mail_txt += sys[0] + ': ' + sys[1] + '\n'
        mail_txt += '\n'

    mail_txt += 'System status:\n'
    for sys in recent_files:
        if sys[1].startswith('OK. '):
            mail_txt += sys[0] + ':\t' + sys[1].replace('OK.','') + '\n'
    mail_txt += '\nSystems in progress:\n'
    for sys in in_progress:
        mail_txt += sys + ', '
    mail_txt = mail_txt[:-2] + '\n\nSystems under maintenance:\n'
    for sys in under_maintenance:
        mail_txt += sys + ', '
    mail_txt = mail_txt[:-2]

    print(mail_txt)
    with open('mail.tmp','w') as mail_f:
        mail_f.write(mail_txt)

    cmd = 'mail -s "Pats daily status digest" sys-maintenance@pats-drones.com < mail.tmp'
    execute(cmd)

    write_systems(changed_systems)

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