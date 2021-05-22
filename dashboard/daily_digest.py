#!/usr/bin/env python3
import subprocess
import time, argparse
import glob, os, re, sys
from datetime import datetime, timedelta
from pytz import timezone
import pandas as pd
sys.path.append('pats_c/lib')
import lib_patsc as patsc

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def load_systems():
    with patsc.open_systems_db() as con:
        sql_str = '''SELECT system,maintenance FROM systems WHERE active = 1 ORDER BY system_id'''
        con.execute(sql_str)
        systems = pd.read_sql_query(sql_str,con)
    return systems

def execute(cmd):
    p_result = None
    popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        p_result = popen.poll()
        if p_result != None:
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()

def get_moth_counts(now):
    today_str = now.strftime('%Y%m%d')
    files = patsc.natural_sort([fp for fp in glob.glob(os.path.expanduser(args.input_folder + '*' + today_str + '*.processed'))])
    systems = load_systems()
    systems['msg'] = ''
    for file in files:
        f = os.path.splitext(os.path.basename((file)))[0]
        if f.startswith('pats'):
            f_date = f.split('_')[-2] + '_' + f.split('_')[-1]
            try:
                d = datetime.strptime(f_date, "%Y%m%d_%H%M%S")
                f_sys = f.split('_')[0].replace('-proto','')
                with open (file, "r") as fr_processed:
                    if f_sys in systems['system'].values:
                        msg = fr_processed.readline()
                        systems.loc[systems['system']==f_sys,'msg'] = msg
                    else:
                        print('Warning, system does not exist: ' + f_sys)
            except:
                pass
    return systems

def get_daily_errors(now):
    today_str = now.strftime('%Y%m%d')
    err_files = patsc.natural_sort([fp for fp in glob.glob(os.path.expanduser('~/daily_basestation_errors/*' + today_str + '*'))])
    systems = load_systems()
    systems['err'] = 0
    systems['overheat_temp'] = 0
    for err_file in err_files:
        e_f = os.path.splitext(os.path.basename((err_file)))[0]
        if e_f.startswith('pats'):
            e_f_date = e_f.split('_')[-2] + '_' + e_f.split('_')[-1]
            try:
                d = datetime.strptime(e_f_date, "%Y%m%d_%H%M%S")
                f_sys = e_f.split('_')[0].replace('-proto','')
                with open (err_file, "r") as fr_processed:
                    if f_sys in systems['system'].values:
                        msgs = fr_processed.readlines()
                        n_errs = len(msgs)
                        sum_overheat_temp = 0
                        cnt_overheat_temp = 0
                        try:
                            for msg in msgs:
                                if 'CPU Temperature too high' in msg:
                                    sum_overheat_temp+= float(msg.split('+')[1].split('°')[0])
                                    cnt_overheat_temp+=1
                        except Exception as e:
                            print('Log processing error, could not process overheat temperature for system:' + f_sys)
                            print(e)
                        if cnt_overheat_temp:
                            systems.loc[systems['system']==f_sys,'overheat_temp'] = sum_overheat_temp/cnt_overheat_temp
                            n_errs -= cnt_overheat_temp
                        else:
                            systems.loc[systems['system']==f_sys,'overheat_temp'] = 0
                        systems.loc[systems['system']==f_sys,'err'] = n_errs
                    else:
                        print('Warning, system does not exist: ' + f_sys)
            except Exception as e:
                pass
    return systems

def send_mail(now,dry_run):
    systems = get_moth_counts(now)
    systems = systems.merge(get_daily_errors(now),how='inner',on=['system','maintenance'])

    mail_warnings = ''
    if (sum(systems['msg']=='')>0):
        for sys,maintenance,_,_,_ in systems[systems['msg']==''].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance,'%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system']==sys,'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_warnings += sys + ', '
                except:
                    mail_warnings += sys + ': date_error, '
            else:
                mail_warnings += sys + ', '
        mail_warnings = mail_warnings[:-2] + '\n'

        for sys,_,message,err,_ in systems.to_numpy():
            if not message.startswith('OK.') and message:
                err_message = ''
                if err > 1:
                    err_message = ' System errors: ' + str(err-1) + '. '
                mail_warnings += sys + ': ' + message.strip() + err_message + '\n'

    mail_err = ''
    if (sum(systems['err']>1)>0): # err > 1 because we force rotation with an error
        for sys,maintenance,_,_,_ in systems[systems['err']>1].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance,'%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system']==sys,'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_err += sys + ', '
                except:
                    mail_err += sys + ': date_error, '
            else:
                mail_err += sys + ', '
        mail_err = mail_err[:-2] + '\n'

    mail_overheat = ''
    if (sum(systems['overheat_temp']>0)>0):
        for sys,maintenance,_,_,temp in systems[systems['overheat_temp']>1].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance,'%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system']==sys,'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_overheat += sys +' (' + str(round(temp)) + '°C)' + ', '
                except:
                    mail_overheat += sys + ': date_error, '
            else:
                mail_overheat += sys +' (' + str(round(temp)) + '°C)' + ', '
        mail_overheat = mail_overheat[:-2] + '\n'

    mail_txt = 'Pats status report ' + str(now) + '\n\n'
    if mail_err:
        mail_txt += 'ERRORS\n' + mail_err + '\n'
    if mail_warnings or mail_overheat:
        mail_txt += 'WARNINGS\n'
        if mail_warnings:
            mail_txt += 'Nothing received from: ' + mail_warnings
        if mail_overheat:
            mail_txt += 'Overheating: ' + mail_overheat
        mail_txt += '\n\n'

    mail_txt += 'System status:\n'
    for sys,_,message,err,_ in systems.to_numpy():
        if message.startswith('OK. '):
            err_message = ''
            if err > 1:
                err_message = ' System errors: ' + str(err-1) + '. '
            elif not err:
                err_message = ' System errors error!!!'
            mail_txt += sys + ':\t' + message.replace('OK.','').strip() + err_message + '\n'

    print(mail_txt)
    with open('mail.tmp','w') as mail_f:
        mail_f.write(mail_txt)

    if not dry_run:
        cmd = 'mail -s "Pats daily status digest" sys-maintenance@pats-drones.com < mail.tmp'
        execute(cmd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
    parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/jsons/')
    parser.add_argument('-t','--hour', help="Send email at the start of this hour.", default=10)
    parser.add_argument('--dry-run', help="Run script now without sending mail", dest='dry_run', action='store_true')
    args = parser.parse_args()

    updated_today = False
    cet = timezone('Europe/Amsterdam')
    while  True:
        now = datetime.now(cet)
        if (now.hour == int(args.hour) and not updated_today) or args.dry_run:
            updated_today = True
            send_mail(now,args.dry_run)
            if args.dry_run:
                break

        if now.hour == int(args.hour)+1 and updated_today:
            updated_today = False
        time.sleep(10)
