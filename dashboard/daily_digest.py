#!/usr/bin/env python3
import sys
sys.path.append('patsc/lib')  # noqa
import lib_patsc as pc
import subprocess
import time
import argparse
import glob
import os
import re
from enum import Enum
from datetime import datetime, timedelta
from pytz import timezone
import pandas as pd


class operation_statuses(Enum):
    inactive = 0
    active = 1
    testing = 2


def natural_sort_systems(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(line, key=alphanum_key)


def load_systems():
    with pc.open_meta_db() as con:
        sql_str = '''SELECT system,operation_status,maintenance,installation_date FROM systems WHERE operation_status IN (1, 2) ORDER BY system_id'''
        con.execute(sql_str)
        systems = pd.read_sql_query(sql_str, con)
    return systems


def execute(cmd):
    p_result = None
    popen = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        p_result = popen.poll()
        if p_result is not None:
            break
        print(stdout_line.decode('utf-8'), end='')
    popen.stdout.close()


def activate_system(system_name):
    system_id = int(system_name.replace('pats', ''))

    activate_sql = f'''UPDATE systems SET operation_status = 1 WHERE system_id = {system_id}; '''
    change_log_sql = f'''INSERT INTO change_log(time,table_name,column_name,id,value,user,saved) VALUES
                            ('{datetime.now().strftime('%Y%m%d_%H%M%S')}','systems','operation_status',{system_id},1,'{system_name}',1); '''
    with pc.open_meta_db() as con:
        con.executescript(activate_sql + change_log_sql)
        con.commit()


def check_operational_status(systems):
    for system, _, _, installation_date, _, _ in systems[(systems['msg'] == '') & (systems['operation_status'] == operation_statuses.testing.value)].to_numpy():
        if installation_date and installation_date != '20000101_120000':
            try:
                installation_date = datetime.strptime(systems.loc[systems['system'] == system, 'installation_date'].values[0], '%Y%m%d_%H%M%S')
                if datetime.today() > installation_date:
                    systems.loc[systems['system'] == system, 'activation_msg'] = 'Installation date has passed, system still inactive.'
            except Exception:
                systems.loc[systems['system'] == system, 'activation_msg'] = 'Installation date invalled.'
    return systems


def select_files_of_yesterday(now, folder):
    yesterday = now - timedelta(days=1)
    all_files = glob.glob(folder)
    yesterday_files = []
    for file in all_files:
        splitted_file = os.path.basename(file).split('_')
        if len(splitted_file) == 3:
            date_str = splitted_file[1] + '_' + splitted_file[2].split('.')[0]
            try:
                date = datetime.strptime(date_str, '%Y%m%d_%H%M%S')
                date = date.replace(tzinfo=cet)
            except ValueError:
                print(f'Error: could not find date in {file}')
                continue
        elif len(splitted_file) == 4:
            date_str = splitted_file[1] + '_' + splitted_file[2] + '_' + splitted_file[3].split('.')[0]
            try:
                date = datetime.strptime(date_str, '%Y%m%d_%H%M%S_%z')
            except ValueError:
                print(f'Error: could not find date in {file}')
                continue
        if date > yesterday:
            yesterday_files.append(file)
    return yesterday_files


def read_processed_jsons(now):
    files = pc.natural_sort(select_files_of_yesterday(now, os.path.expanduser(args.input_folder + '*.processed')))
    systems = load_systems()
    systems['msg'] = ''
    systems['activation_msg'] = ''
    for file in files:
        f = os.path.splitext(os.path.basename((file)))[0]
        if f.startswith('pats'):
            try:
                f_sys = f.split('_')[0].replace('-proto', '')
                with open(file, "r") as fr_processed:
                    if f_sys in systems['system'].values:
                        msg = fr_processed.readline()
                        systems.loc[systems['system'] == f_sys, 'msg'] = msg
                        if systems.loc[systems['system'] == f_sys, 'operation_status'].values[0] == operation_statuses.testing.value:
                            if 'At office' in msg:
                                at_office = int(msg.split('At office: ')[1].strip()[0])
                                if not at_office:
                                    activate_system(f_sys)
                                    systems.loc[systems['system'] == f_sys, 'activation_msg'] = 'System was activated.'
                    else:
                        print('Warning, system does not exist: ' + f_sys)
            except Exception:
                pass
    systems = check_operational_status(systems)
    systems = systems.drop(columns=['operation_status', 'installation_date'])
    return systems


def daily_errors(now):
    err_files = pc.natural_sort(select_files_of_yesterday(now, os.path.expanduser('~/daily_basestation_errors/*')))
    systems = load_systems()
    systems['err'] = 0
    systems['overheat_temp'] = 0
    for err_file in err_files:
        e_f = os.path.splitext(os.path.basename((err_file)))[0]
        if e_f.startswith('pats'):
            try:
                f_sys = e_f.split('_')[0].replace('-proto', '')
                with open(err_file, "r") as fr_processed:
                    if f_sys in systems['system'].values:
                        if systems.loc[systems['system'] == f_sys, 'operation_status'].values[0]:
                            msgs = fr_processed.readlines()
                            n_errs = len(msgs)
                            sum_overheat_temp = 0
                            cnt_overheat_temp = 0
                            try:
                                for msg in msgs:
                                    if 'CPU Temperature too high' in msg:
                                        sum_overheat_temp += float(msg.split('+')[1].split('°')[0])
                                        cnt_overheat_temp += 1
                            except Exception as e:
                                print('Log processing error, could not process overheat temperature for system:' + f_sys)
                                print(e)
                            if cnt_overheat_temp:
                                systems.loc[systems['system'] == f_sys, 'overheat_temp'] = sum_overheat_temp / cnt_overheat_temp
                                n_errs -= cnt_overheat_temp
                            else:
                                systems.loc[systems['system'] == f_sys, 'overheat_temp'] = 0
                            systems.loc[systems['system'] == f_sys, 'err'] = n_errs
                    else:
                        print('Warning, system does not exist: ' + f_sys)
            except Exception:
                pass
    systems = systems.drop(columns=['installation_date'])
    return systems


def next_date(file_reader):
    cet = timezone('Europe/Amsterdam')
    line = file_reader.readline()
    date = None
    if line:
        try:
            date = datetime.strptime(line.split(' - ')[0], '%Y-%m-%d %H:%M:%S,%f')  # The patsc log has an extended error so not every line is a date
            date = date.astimezone(cet)
        except ValueError:
            date = next_date(file_reader)
    return date


def n_errors_from_file(file, now: datetime):
    yesterday = now - timedelta(days=1)
    n_error = -1  # if the file can't be read for some reason the function will return -1
    if os.path.exists(file):
        with open(file, "r") as err_file:
            n_error = 0
            log_date = next_date(err_file)
            if log_date:
                while log_date:
                    if log_date > yesterday:
                        n_error += 1
                    log_date = next_date(err_file)
    return n_error


def system_info_to_mail(systems: pd.DataFrame, daemon_errors, patsc_errors):
    mail_warnings = ''
    if (sum(systems['msg'] == '') > 0):
        for system, maintenance, _, _, _, _, _ in systems[systems['msg'] == ''].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance, '%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system'] == system, 'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_warnings += system + ', '
                except Exception:
                    mail_warnings += system + ': date_error, '
            else:
                mail_warnings += system + ', '
        mail_warnings = mail_warnings[:-2] + '\n'

        for system, _, message, _, _, err, _ in systems.to_numpy():
            if not message.startswith('OK.') and message:
                err_message = ''
                if err > 1:
                    err_message = ' System errors: ' + str(err - 1) + '. '
                mail_warnings += system + ': ' + message.strip() + err_message + '\n'

    mail_err = ''
    if daemon_errors or patsc_errors:
        mail_err += 'Deamon erros: ' + str(daemon_errors) + ' Pats-c errors: ' + str(patsc_errors) + '\n'

    if (sum(systems['err'] > 1) > 0):  # err > 1 because we force rotation with an error
        for system, maintenance, _, _, _, _, _ in systems[systems['err'] > 1].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance, '%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system'] == system, 'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_err += system + ', '
                except Exception:
                    mail_err += system + ': date_error, '
            else:
                mail_err += system + ', '
        mail_err = mail_err[:-2] + '\n'

    mail_overheat = ''
    if (sum(systems['overheat_temp'] > 0) > 0):
        for system, maintenance, _, _, _, _, temp in systems[systems['overheat_temp'] > 1].to_numpy():
            if maintenance:
                try:
                    date = datetime.strptime(maintenance, '%Y%m%d')
                    if datetime.today() - date <= timedelta(days=1):
                        systems.loc[systems['system'] == system, 'msg'] = 'OK. under maintenance \n'
                    else:
                        mail_overheat += system + ' (' + str(round(temp)) + '°C)' + ', '
                except Exception:
                    mail_overheat += system + ': date_error, '
            else:
                mail_overheat += system + ' (' + str(round(temp)) + '°C)' + ', '
        mail_overheat = mail_overheat[:-2] + '\n'

    mail_activation = ''
    if (sum(systems['activation_msg'] != '') > 0):
        for system, _, _, activation_msg, _, _, _ in systems[systems['activation_msg'] != ''].to_numpy():
            mail_activation += system + ': ' + activation_msg.strip() + '\n'

    if mail_warnings:
        mail_warnings = 'Nothing received from: ' + mail_warnings
    if mail_overheat:
        mail_warnings += 'Overheating: ' + mail_overheat

    system_txt = ''
    for system, _, message, _, _, err, _ in systems.to_numpy():
        if message.startswith('OK. '):
            err_message = ''
            if err > 1:
                err_message = ' System errors: ' + str(err - 1) + '. '
            elif not err:
                err_message = ' System errors error!!!'
            system_txt += system + ':\t' + message.replace('OK.', '').strip() + err_message + '\n'

    return system_txt, mail_err, mail_warnings, mail_activation


def send_mail(now, dry_run):
    systems = read_processed_jsons(now)
    systems = systems.merge(daily_errors(now), how='inner', on=['system', 'maintenance'])
    daemon_errors = n_errors_from_file(pc.daemon_error_log, now)
    patsc_errors = n_errors_from_file(pc.patsc_error_log, now)

    system_txt, mail_err, mail_warnings, mail_activation = system_info_to_mail(systems[systems['operation_status'] == 1], daemon_errors, patsc_errors)
    system_txt_test, mail_err_test, mail_warnings_test, mail_activation_test = system_info_to_mail(systems[systems['operation_status'] == 2], None, None)
    if system_txt_test:
        system_txt += '\nTest systems:\n' + system_txt_test
    if mail_err_test:
        mail_err += 'Test systems:\n' + mail_err_test
    if mail_warnings_test:
        mail_warnings += 'Test systems:\n' + 'Nothing received from: ' + mail_warnings_test
    if mail_activation_test:
        mail_activation += 'Test systems:\n' + mail_activation_test + '\n'

    mail_txt = 'Pats status report ' + str(now) + '\n\n'
    if mail_err:
        mail_txt += 'ERRORS\n' + mail_err + '\n'
    if mail_warnings:
        mail_txt += 'WARNINGS\n' + mail_warnings + '\n'
    if mail_activation:
        mail_txt += 'Activated systems: \n' + mail_activation + '\n'
    mail_txt += '\n'
    if system_txt:
        mail_txt += 'System status: \n' + system_txt

    print(mail_txt)
    with open('mail.tmp', 'w') as mail_f:
        mail_f.write(mail_txt)

    if not dry_run:
        cmd = 'mail -s "Pats daily status digest" sys-maintenance@pats-drones.com < mail.tmp'
        execute(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that adds the json files or incoming json files to the database that is reable for the electron app.')
    parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/patsc/jsons/')
    parser.add_argument('-t', '--hour', help="Send email at the start of this hour.", default=10)
    parser.add_argument('--dry-run', help="Run script now without sending mail", dest='dry_run', action='store_true')
    args = parser.parse_args()

    updated_today = False
    cet = timezone('Europe/Amsterdam')
    while True:
        now = datetime.now(cet)
        if (now.hour == int(args.hour) and not updated_today) or args.dry_run:
            updated_today = True
            send_mail(now, args.dry_run)
            if args.dry_run:
                break

        if now.hour == int(args.hour) + 1 and updated_today:
            updated_today = False
        time.sleep(10)
