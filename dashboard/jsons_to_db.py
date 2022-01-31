#!/usr/bin/env python3
import sys
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc
import json
import glob
import os
import datetime
from tqdm import tqdm
from json.decoder import JSONDecodeError


def get_column_data_type(column):
    if column == 'duration' or column == 'Version' or column == 'Vel_mean' or column == 'Vel_max' or column == 'Vel_std' or column == 'Version' or column == 'Dist_traveled' or column == 'Size' or column == 'Alpha_horizontal_start' or column == 'Alpha_horizontal_end' or column == 'Alpha_vertical_start' or column == 'Alpha_vertical_end' or column == 'Dist_traject':
        return 'REAL'
    elif column == 'RS_ID':
        return 'INT'
    else:
        return 'TEXT'


def create_moth_table(moths, data, con):
    sql_create = 'CREATE TABLE moth_records(uid INTEGER PRIMARY KEY, system,time,'
    clean_moth = patsc.clean_moth_json_entry(moths[-1], data)  # select the last moth as a prototype for the columns. Last because that probably is in last version
    for s in list(clean_moth.keys())[1:]:
        sql_create = sql_create + s + ' ' + get_column_data_type(s) + ','
    sql = sql_create[:-1] + ')'
    con.execute(sql)
    con.execute('CREATE INDEX idx_moth_sys_name ON moth_records(system);')
    con.commit()


def check_if_table_exists(table_name_prefix, con):
    cur = con.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='" + table_name_prefix + "_records'")
    return cur.fetchone()[0]


def get_insect_chance_columns():
    sql_str = 'SELECT LIA_name FROM insects WHERE LIA_name IS NOT NULL'
    with patsc.open_systems_db() as con:
        columns = con.execute(sql_str).fetchall()
        columns = [col[0] for col in columns]
    return columns


def store_moths(data, dry_run):
    moths = data["moths"]
    if not len(moths):
        return 0
    with patsc.open_data_db() as con:
        if not check_if_table_exists('moth', con):
            create_moth_table(moths, data, con)
        cur = con.cursor()
        columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]

        # this section really only is needed once to upgrade the db (and can be removed if the main db is upgraded and old jsons are not in use anymore)
        # v1.0:
        if 'RS_ID' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN RS_ID INT')
        if 'duration' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN duration REAL')
        if 'Vel_mean' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Vel_mean REAL')
        if 'Vel_max' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Vel_max REAL')
        if 'Vel_std' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Vel_std REAL')
        # v1.1/v1.2:
        if 'Version' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Version REAL')
        if 'Mode' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Mode TEXT')
        if 'Video_Filename' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Video_Filename TEXT')
        # v1.3:
        if 'Dist_traveled' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Dist_traveled REAL')
        if 'Size' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Size REAL')
        if 'Alpha_horizontal_start' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Alpha_horizontal_start REAL')
        if 'Alpha_horizontal_end' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Alpha_horizontal_end REAL')
        if 'Alpha_vertical_start' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Alpha_vertical_start REAL')
        if 'Alpha_vertical_end' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Alpha_vertical_end REAL')
        # v1.4:
        if 'Dist_traject' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Dist_traject REAL')
        # v1.5:
        if 'Folder' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Folder TEXT')
        # v1.8:
        if 'Wing_beat' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Wing_beat REAL')
        # v1.9:
        if 'Monster' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN Monster INT')
        # v1.10: implemented full widt third maximum
        # v1.11: Insect labeling from trajectory
        if 'LIA_insect' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN LIA_insect INT')
        # v1.12: Added a version str for the insect labeling
        if 'LIA_version' not in columns:
            cur.execute('ALTER TABLE moth_records ADD COLUMN LIA_version TEXT')
        # v1.13: Added all the different insect probabilities found by LIA
        for col in get_insect_chance_columns():
            if col not in columns:
                cur.execute(f'ALTER TABLE moth_records ADD COLUMN {col} REAL')

        sql_insert = ''
        for moth in moths:
            date = moth['time']

            moth = patsc.clean_moth_json_entry(moth, data)

            if sql_insert == '':
                sql_insert = 'INSERT INTO moth_records(system,time,'
                sql_values = ') VALUES(?,?,'
                for s in list(moth.keys())[1:]:
                    sql_insert = sql_insert + s + ','
                    sql_values = sql_values + '?,'
                sql_insert = sql_insert[:-1] + sql_values[:-1] + ')'
            if not dry_run:
                cur.execute(sql_insert, (data["system"], date, *list(moth.values())[1:]))
            else:
                print(sql_insert + ' ' + ' ,'.join([data["system"], date, *list(moth.values())[1:]]))
        con.commit()
    return len(moths)


def create_modes_table(con):
    sql_create = 'CREATE TABLE mode_records(uid INTEGER PRIMARY KEY,system TEXT,start_datetime TEXT,end_datetime TEXT,op_mode TEXT)'
    con.execute(sql_create)
    con.execute('CREATE INDEX idx_mode_sys_name ON mode_records(system,start_datetime);')
    con.commit()


def store_mode(data, dry_run):
    with patsc.open_data_db() as con:
        if not check_if_table_exists('mode', con):
            create_modes_table(con)
        cur = con.cursor()
        mode_data = data["mode"]
        sql_insert = 'INSERT INTO mode_records(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'
        for entry in mode_data:
            sub_entries = []  # there may be another nested level here, in case of waiting for darkness
            if type(entry) == list:
                sub_entries = entry
            else:
                sub_entries = [entry]
            for sub_entry in sub_entries:
                dt_from = sub_entry['from']
                dt_till = sub_entry['till']
                if not dry_run:
                    cur.execute(sql_insert, (data["system"], dt_from, dt_till, sub_entry['mode']))
                else:
                    print(sql_insert + ' ' + ' ,'.join([data["system"], dt_from, dt_till, sub_entry['mode']]))
        con.commit()
        return len(mode_data)


def load_systems(cur):
    sql_str = '''SELECT DISTINCT system FROM mode_records'''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = [d[0] for d in systems]
    return systems


def todatetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")


def concat_modes(start_date):
    start_date = start_date.strftime('%Y%m%d_%H%M%S')
    with patsc.open_data_db() as con:
        if not check_if_table_exists('mode', con):
            return
        cur = con.cursor()
        systems = load_systems(cur)
        columns = [i[1] for i in cur.execute('PRAGMA table_info(mode_records)')]
        entry_id = columns.index('uid')
        t_start_id = columns.index('start_datetime')
        t_end_id = columns.index('end_datetime')
        mode_id = columns.index('op_mode')
        sql_insert = 'INSERT INTO mode_records(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'
        sql_delete = 'DELETE FROM mode_records WHERE uid='

        pbar = tqdm(systems, desc='Concatenating modes db')
        for system in pbar:
            sql_str = "SELECT * FROM mode_records WHERE system='" + system + "' and start_datetime > '" + start_date + "' ORDER BY start_datetime"
            cur.execute(sql_str)
            modes = cur.fetchall()
            if len(modes) < 2:
                continue
            i = 1
            start_entrie = None
            while i < len(modes):
                entry = modes[i]
                prev_entry = modes[i - 1]
                if (prev_entry[t_start_id] == entry[t_start_id] and prev_entry[t_end_id] == entry[t_end_id]) or entry[t_start_id] > entry[t_end_id] or entry[mode_id] == 'down':
                    # remove duplicates and corrupted entries
                    cur.execute(sql_delete + str(entry[entry_id]))
                elif prev_entry[mode_id] == entry[mode_id] and todatetime(entry[t_start_id]) - todatetime(prev_entry[t_end_id]) < datetime.timedelta(minutes=10):
                    # found two or more entries that can be merged.
                    if start_entrie is None:
                        start_entrie = prev_entry
                        cur.execute(sql_delete + str(prev_entry[entry_id]))
                    cur.execute(sql_delete + str(entry[entry_id]))
                elif start_entrie is not None:
                    # found the end of a session. Merge previous entries
                    cur.execute(sql_insert, (system, start_entrie[t_start_id], prev_entry[t_end_id], start_entrie[mode_id]))
                    start_entrie = None
                i += 1
            if start_entrie is not None:
                cur.execute(sql_insert, (system, start_entrie[t_start_id], entry[t_end_id], start_entrie[mode_id]))
        con.commit()


def remove_double_data(table_name_prefix, start_date):
    start_date = start_date.strftime('%Y%m%d_%H%M%S')
    with patsc.open_data_db() as con:
        cur = con.cursor()
        if not check_if_table_exists(table_name_prefix, con):
            return
        systems = load_systems(cur)
        columns = [i[1] for i in cur.execute('PRAGMA table_info(' + table_name_prefix + '_records)')]
        v_id = -1
        uid_id = columns.index('uid')
        if 'Version' in columns:
            v_id = columns.index('Version')
        if 'time' in columns:  # work around slightly annoying situation that we have time+duration in the moth table, and start_datetime and end_datetime in all other tables:
            t_id = columns.index('time')
            t2_id = columns.index('duration')
            order_by = '"time", "duration"'
            start_date_sql_str = 'time'
        else:
            t_id = columns.index('start_datetime')
            t2_id = columns.index('end_datetime')
            order_by = '"start_datetime", "end_datetime"'
            start_date_sql_str = 'start_datetime'

        tot_doubles = 0
        pbar_systems = tqdm(systems, leave=False)
        for system in pbar_systems:
            pbar_systems.set_description('Cleaning double ' + table_name_prefix + ' FROM db, system: ' + system)

            sql_str = 'SELECT count(system) FROM ' + table_name_prefix + '_records WHERE system="' + system + '" AND ' + start_date_sql_str + ' > "' + start_date + '"'
            cur.execute(sql_str)
            totn = int(cur.fetchall()[0][0])
            pbar_entries = tqdm(total=2 * totn, leave=False)

            sql_str = 'SELECT * FROM ' + table_name_prefix + '_records WHERE system="' + system + '" AND ' + start_date_sql_str + ' > "' + start_date + '" ORDER BY ' + order_by
            cur.execute(sql_str)

            prev_entry = cur.fetchone()
            entry = cur.fetchone()
            doubles_uids = []
            while entry is not None:
                pbar_entries.update(1)

                if (prev_entry[t_id] == entry[t_id] and prev_entry[t2_id] == entry[t2_id]):
                    if v_id >= 0:
                        if entry[v_id] > prev_entry[v_id]:
                            doubles_uids.append(prev_entry[uid_id])
                            prev_entry = entry
                        else:
                            doubles_uids.append(entry[uid_id])
                    else:
                        doubles_uids.append(entry[uid_id])
                else:
                    prev_entry = entry

                entry = cur.fetchone()

            if (len(doubles_uids)):
                tot_doubles += len(doubles_uids)
                for entry in doubles_uids:
                    pbar_entries.update(1)
                    sql_del_str = 'DELETE FROM ' + table_name_prefix + '_records' + ' WHERE uid=' + str(entry)
                    cur.execute(sql_del_str)
            pbar_entries.close()
        con.commit()
        print("Found and deleted " + str(tot_doubles) + " doubles in " + table_name_prefix + " records.")


def create_hunt_table(hunt, con):
    sql_create_table = 'CREATE TABLE hunt_records(uid INTEGER PRIMARY KEY,system,start_datetime,end_datetime,'
    for s in list(hunt.keys())[2:]:
        sql_create_table = sql_create_table + s + ' ' + get_column_data_type(s) + ','
    sql_create_table = sql_create_table[:-1] + ')'
    con.execute(sql_create_table)
    con.execute('CREATE INDEX idx_hunt_sys_name ON hunt_records(system);')
    con.commit()


def store_hunts(data, dry_run):
    hunts = data["hunts"]
    if not len(hunts):
        return 0
    with patsc.open_data_db() as con:
        cur = con.cursor()
        hunt_table_exist = check_if_table_exists('hunt', con)
        sql_insert = ''
        hunts = data["hunts"]
        for hunt in hunts:
            if not hunt_table_exist:
                create_hunt_table(hunt, con)
                hunt_table_exist = True

            dt_from = hunt['from']
            dt_till = hunt['till']
            if sql_insert == '':
                sql_insert = 'INSERT INTO hunt_records(system,start_datetime,end_datetime,'
                sql__insert_values = ') VALUES(?,?,?,'
                for s in list(hunt.keys())[2:]:
                    sql_insert = sql_insert + s + ','
                    sql__insert_values = sql__insert_values + '?,'
                sql_insert = sql_insert[:-1] + sql__insert_values[:-1] + ')'

            if not dry_run:
                cur.execute(sql_insert, (data["system"], dt_from, dt_till, *list(hunt.values())[2:]))
            else:
                print(sql_insert + ' ' + ' ,'.join([data["system"], dt_from, dt_till, *list(hunt.values())[2:]]))
        con.commit()
    return len(hunts)


def count_errors(data):
    if 'errors' in data:
        errors = data['errors']
        return len(errors)
    else:
        return 0


def process_json(data, dry_run):
    n_modes = store_mode(data, dry_run)
    n_moths = store_moths(data, dry_run)
    n_hunts = store_hunts(data, dry_run)
    n_errors = count_errors(data)
    return n_moths, n_modes, n_hunts, n_errors


def jsons_to_db(input_folder, dry_run):
    files = patsc.natural_sort([fp for fp in glob.glob(os.path.expanduser(input_folder + "/*.json"))])
    pbar = tqdm(files)
    first_date = datetime.datetime.now()
    for filename in pbar:
        pbar.set_description('DB update: ' + os.path.basename(filename))
        flag_fn = filename[:-4] + 'processed'
        if not os.path.exists(flag_fn) or dry_run:
            with open(filename) as json_file:
                with open(flag_fn, 'w') as flag_f:
                    if os.stat(filename).st_size < 40000000:
                        try:
                            data = json.load(json_file)
                            try:
                                data_start = todatetime(data['from'])
                            except Exception:  # issue #1002
                                data_start = datetime.datetime.now() - datetime.timedelta(days=365)
                            if data_start < first_date:
                                first_date = data_start
                            min_required_version = 1.0
                            if "version" in data and float(data["version"]) >= min_required_version:
                                n_moths, n_modes, n_hunts, n_errors = process_json(data, dry_run)
                                if n_modes:
                                    flag_f.write('OK')
                                    flag_f.write('. Insect detections: ' + str(n_moths))
                                    flag_f.write('. Hunts: ' + str(n_hunts))
                                    flag_f.write('. Errors from jsons: ' + str(n_errors))
                                    flag_f.write('.\n')
                                else:
                                    flag_f.write('PROCESS NOT RUNNING')
                            elif "version" in data:
                                flag_f.write('WRONG VERSION ' + data["version"] + '. Want: ' + min_required_version + '\n')
                            else:
                                flag_f.write('NO VERSION DETECTED')
                        except JSONDecodeError:
                            flag_f.write('JSONDecodeError \n')
                        except Exception as error:
                            flag_f.write('ERROR: ' + str(error))
                    else:
                        flag_f.write('File size too big \n')
    if not dry_run:
        concat_modes(first_date)
        remove_double_data('moth', first_date)
        remove_double_data('hunt', first_date)


if __name__ == "__main__":
    jsons_to_db('~/patsc/jsons/', False)
