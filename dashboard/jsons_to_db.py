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


def create_detections_table(con):
    sql_create = 'CREATE TABLE detections(uid INTEGER PRIMARY KEY, system TEXT,start_datetime TEXT, duration REAL, rs_id INT, filename TEXT, vel_mean REAL, vel_std REAL, vel_max REAL, version TEXT, mode TEXT, video_filename TEXT, dis_traveled REAL, dis_traject REAL, size REAL, alpha_horizontal_start REAL, alpha_horizontal_end REAL, alpha_vertical_start REAL, alpha_vertical_end REAL, folder TEXT, human_classification TEXT, wing_beat REAL, monster INT, lia_insect INT, lia_version TEXT, chance_chrysodeixis_chalcites REAL, chance_duponchelia_fovealis REAL,chance_tuta_aboluta REAL,chance_opogona_sacchari REAL,chance_plutella_xylostella REAL);'
    con.execute(sql_create)
    con.execute('CREATE INDEX idx_detections_sys_name ON detections(system);')
    con.commit()


def check_if_table_exists(table_name_prefix, con):
    cur = con.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='" + table_name_prefix + "'")
    return cur.fetchone()[0]


def detection_chance_columns():
    sql_str = 'SELECT lia_label FROM detections WHERE lia_label IS NOT NULL'
    with patsc.open_systems_db() as con:
        columns = con.execute(sql_str).fetchall()
        columns = [col[0] for col in columns]
    return columns


def store_detections(data, dry_run):
    if not patsc.check_verion(data['version'], 2):  # legacy v1
        detections = data["moths"]
    else:
        detections = data["detections"]
    if not len(detections):
        return 0
    with patsc.open_data_db() as con:
        if not check_if_table_exists('detections', con):
            create_detections_table(con)
        cur = con.cursor()

        sql_insert = ''
        for detection in detections:
            if not patsc.check_verion(data['version'], 2):  # legacy v1
                start_datetime = detection['time']
            else:
                start_datetime = detection['start_datetime']

            if sql_insert == '':
                sql_insert = 'INSERT INTO detections(system,start_datetime,'
                sql_values = ') VALUES(?,?,'
                for key in list(detection.keys())[1:]:
                    sql_insert = sql_insert + key.lower() + ','  # lower -> legacy v1
                    sql_values = sql_values + '?,'
                sql_insert = sql_insert[:-1] + sql_values[:-1] + ')'
            if not dry_run:
                cur.execute(sql_insert, (data["system"], start_datetime, *list(detection.values())[1:]))
            else:
                print(sql_insert + ' ' + ' ,'.join([data["system"], start_datetime, *list(detection.values())[1:]]))
        con.commit()
    return len(detections)


def create_status_table(con):
    sql_create = 'CREATE TABLE status(uid INTEGER PRIMARY KEY,system TEXT,start_datetime TEXT,end_datetime TEXT,op_mode TEXT)'
    con.execute(sql_create)
    con.execute('CREATE INDEX idx_status_sys_name ON status(system,start_datetime);')
    con.commit()


def store_status(data, dry_run):
    with patsc.open_data_db() as con:
        if not check_if_table_exists('status', con):
            create_status_table(con)
        cur = con.cursor()
        status_data = data["mode"]
        sql_insert = 'INSERT INTO status(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'
        for entry in status_data:
            sub_entries = []  # there may be another nested level here, in case of waiting for darkness
            if type(entry) == list:
                sub_entries = entry
            else:
                sub_entries = [entry]
            for sub_entry in sub_entries:
                if not patsc.check_verion(data['version'], 2):  # legacy v1
                    start_datetime = sub_entry['from']
                    end_datetime = sub_entry['till']
                else:
                    start_datetime = sub_entry['start_datetime']
                    end_datetime = sub_entry['end_datetime']
                if not dry_run:
                    cur.execute(sql_insert, (data["system"], start_datetime, end_datetime, sub_entry['mode']))
                else:
                    print(sql_insert + ' ' + ' ,'.join([data["system"], start_datetime, end_datetime, sub_entry['mode']]))
        con.commit()
        return len(status_data)


def load_systems(cur):
    sql_str = '''SELECT DISTINCT system FROM status'''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = [d[0] for d in systems]
    return systems


def to_datetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")


def concat_statuses(start_date):
    start_date = start_date.strftime('%Y%m%d_%H%M%S')
    with patsc.open_data_db() as con:
        if not check_if_table_exists('status', con):
            return
        cur = con.cursor()
        systems = load_systems(cur)
        columns = [i[1] for i in cur.execute('PRAGMA table_info(status)')]
        entry_id = columns.index('uid')
        t_start_id = columns.index('start_datetime')
        t_end_id = columns.index('end_datetime')
        mode_id = columns.index('op_mode')
        sql_insert = 'INSERT INTO status(system,start_datetime,end_datetime,op_mode) VALUES(?,?,?,?)'
        sql_delete = 'DELETE FROM status WHERE uid='

        pbar = tqdm(systems, desc='Concatenating status db')
        for system in pbar:
            sql_str = "SELECT * FROM status WHERE system='" + system + "' and start_datetime > '" + start_date + "' ORDER BY start_datetime"
            cur.execute(sql_str)
            statuses = cur.fetchall()
            if len(statuses) < 2:
                continue
            i = 1
            start_entrie = None
            while i < len(statuses):
                entry = statuses[i]
                prev_entry = statuses[i - 1]
                if (prev_entry[t_start_id] == entry[t_start_id] and prev_entry[t_end_id] == entry[t_end_id]) or entry[t_start_id] > entry[t_end_id] or entry[mode_id] == 'down':
                    # remove duplicates and corrupted entries
                    cur.execute(sql_delete + str(entry[entry_id]))
                elif prev_entry[mode_id] == entry[mode_id] and to_datetime(entry[t_start_id]) - to_datetime(prev_entry[t_end_id]) < datetime.timedelta(minutes=10):
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


def remove_double_data(table_name, start_date):
    start_date = start_date.strftime('%Y%m%d_%H%M%S')
    with patsc.open_data_db() as con:
        cur = con.cursor()
        if not check_if_table_exists(table_name, con):
            return
        systems = load_systems(cur)
        columns = [i[1] for i in cur.execute('PRAGMA table_info(' + table_name + ')')]
        v_id = -1
        uid_id = columns.index('uid')
        if 'version' in columns:
            v_id = columns.index('version')
        t_id = columns.index('start_datetime')
        if 'duration' in columns:
            t2_id = columns.index('duration')
        else:
            t2_id = columns.index('end_datetime')

        tot_doubles = 0
        pbar_systems = tqdm(systems, leave=False)
        for system in pbar_systems:
            pbar_systems.set_description('Cleaning double ' + table_name + ' FROM db, system: ' + system)

            sql_str = 'SELECT count(system) FROM ' + table_name + ' WHERE system="' + system + '" AND start_datetime > "' + start_date + '"'
            cur.execute(sql_str)
            totn = int(cur.fetchall()[0][0])
            pbar_entries = tqdm(total=2 * totn, leave=False)

            sql_str = 'SELECT * FROM ' + table_name + ' WHERE system="' + system + '" AND start_datetime > "' + start_date + '" ORDER BY "start_datetime", "end_datetime"'
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
                    sql_del_str = 'DELETE FROM ' + table_name + ' WHERE uid=' + str(entry)
                    cur.execute(sql_del_str)
            pbar_entries.close()
        con.commit()
        print("Found and deleted " + str(tot_doubles) + " doubles in " + table_name + " records.")


def create_flights_table(con):
    # TODO implement
    pass


def store_flights(data, dry_run):
    if not patsc.check_verion(data['version'], 2):  # legacy v1
        return 0
    flights = data["flights"]
    if not len(flights):
        return 0
    with patsc.open_data_db() as con:
        cur = con.cursor()
        if not check_if_table_exists('flights', con):
            create_flights_table(con)
        sql_insert = ''
        flights = data["flights"]
        for flight in flights:
            start_datetime = flight['start_datetime']
            duration = flight['duration']
            if sql_insert == '':
                sql_insert = 'INSERT INTO flights(system,start_datetime,duration,'
                sql__insert_values = ') VALUES(?,?,?,'
                for key in list(flight.keys())[2:]:
                    sql_insert = sql_insert + key + ','
                    sql__insert_values = sql__insert_values + '?,'
                sql_insert = sql_insert[:-1] + sql__insert_values[:-1] + ')'

            if not dry_run:
                cur.execute(sql_insert, (data["system"], start_datetime, duration, *list(flight.values())[2:]))
            else:
                print(sql_insert + ' ' + ' ,'.join([data["system"], start_datetime, duration, *list(flight.values())[2:]]))
        con.commit()
    return len(flights)


def count_errors(data):
    if 'errors' in data:
        errors = data['errors']
        return len(errors)
    else:
        return 0


def process_json(data, dry_run):
    n_statuses = store_status(data, dry_run)
    n_detections = store_detections(data, dry_run)
    n_flights = store_flights(data, dry_run)
    n_errors = count_errors(data)
    return n_detections, n_statuses, n_flights, n_errors


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
                                if 'start_datetime' in data:
                                    data_start = to_datetime(data['start_datetime'])
                                else:  # legacy v1
                                    data_start = to_datetime(data['from'])
                            except Exception:  # issue #1002
                                data_start = datetime.datetime.now() - datetime.timedelta(days=365)
                            if data_start < first_date:
                                first_date = data_start
                            min_required_version = 1.0  # legacy v1
                            if float(data["version"]) >= min_required_version:
                                n_detections, n_statuses, n_flights, n_errors = process_json(data, dry_run)
                                if n_statuses:
                                    flag_f.write('OK')
                                    flag_f.write('. Statuses: ' + str(n_statuses))
                                    flag_f.write('. Detections: ' + str(n_detections))
                                    flag_f.write('. Flights: ' + str(n_flights))
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
        concat_statuses(first_date)
        remove_double_data('detections', first_date)
        remove_double_data('flights', first_date)


if __name__ == "__main__":
    jsons_to_db('~/patsc/jsons/', False)
