#!/usr/bin/env python3
import datetime
import os
import re
from typing import List, Dict, Tuple
from dateutil.relativedelta import relativedelta
import numpy as np
import pandas as pd
import pandas_ta  # noqa used for trendline
from enum import Enum
import dash
from dash import dcc
from dash import html
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from dash.dependencies import Output, Input, State
import dash_bootstrap_components as dbc
from flask_login import current_user
from urllib.parse import quote as urlquote
import patsc.lib.lib_patsc as pc


customer_dict: Dict[str, List[Tuple[str, str]]] = {}
detection_columns: List[str] = []
heatmap_max = 50
classification_options = ['Not classified', 'Moth!', 'Empty/nothing to see', 'Other insect', 'Plant', 'Other false positive']
scatter_columns = {'duration': 'Duration (s)',
                   'vel_mean': 'Velocity mean (m/s)',
                   'vel_max': 'Velocity max (m/s)',
                   'dist_traveled': 'Distance traveled (m)',
                   'dist_traject': 'Distance trajectory (m)',
                   'size': 'Size (m)',
                   'wing_beat': 'Wing beat (Hz)'}


class Heatmap_Cell(Enum):
    selected_cell = -3
    system_down_cell = -2
    system_offline_cell = -1


def init_lia_options():
    detection_classes = []
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            with pc.open_meta_db() as con:
                sql_str = 'SELECT DISTINCT detection_classes.name, lia_label FROM detection_classes' \
                    + ' JOIN crop_detection_class_connection ON crop_detection_class_connection.class_id = detection_classes.class_id' \
                    + ' JOIN crops ON crops.crop_id = crop_detection_class_connection.crop_id' \
                    + ' JOIN customers ON customers.crop_id = crops.crop_id' \
                    + ' JOIN user_customer_connection ON user_customer_connection.customer_id = customers.customer_id' \
                    + ' JOIN users ON users.user_id = user_customer_connection.user_id' \
                    + ' WHERE users.name = "' + username + '" AND lia_label NOT NULL ORDER BY detection_classes.name'
                detections = con.execute(sql_str)
                for name, LIA_label in detections:
                    detection_classes.append({'label': name, 'value': {'label': name, 'lia_label': LIA_label}})

    detection_classes.append({'label': 'No filter', 'value': {'label': 'No filter', 'lia_label': ''}})
    detection_classes.append({'label': 'Anomalies', 'value': {'label': 'Anomalies', 'lia_label': ''}})
    return detection_classes


def init_classic_options():
    detection_classes = []
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            with pc.open_meta_db() as con:
                sql_str = 'SELECT DISTINCT detection_classes.name,avg_size,std_size,floodfill_avg_size,floodfill_std_size FROM detection_classes' \
                    + ' JOIN crop_detection_class_connection ON crop_detection_class_connection.class_id = detection_classes.class_id' \
                    + ' JOIN crops ON crops.crop_id = crop_detection_class_connection.crop_id' \
                    + ' JOIN customers ON customers.crop_id = crops.crop_id' \
                    + ' JOIN user_customer_connection ON user_customer_connection.customer_id = customers.customer_id' \
                    + ' JOIN users ON users.user_id = user_customer_connection.user_id' \
                    + ' WHERE users.name = "' + username + '" ORDER BY detection_classes.name'
                detections = con.execute(sql_str)
                for name, avg_size, std_size, floodfill_avg_size, floodfill_std_size in detections:
                    detection_classes.append({'label': name, 'value': {'label': name, 'avg_size': avg_size, 'std_dev': std_size, 'floodfill_avg_size': floodfill_avg_size, 'floodfill_std_dev': floodfill_std_size}})

    detection_classes.append({'label': 'No size filter', 'value': {'label': 'No size filter', 'avg_size': 0, 'std_dev': 0, 'floodfill_avg_size': 0, 'floodfill_std_dev': 0}})
    detection_classes.append({'label': 'Anomalies', 'value': {'label': 'Anomalies', 'avg_size': 0, 'std_dev': 0, 'floodfill_avg_size': 0, 'floodfill_std_dev': 0}})
    return detection_classes


def init_dropdowns():
    customer_dict, demo = pc.load_customers()
    customer_value = None
    customer_style = {'width': '30%', 'display': 'inline-block'}
    sys_style = {'width': '70%', 'display': 'inline-block'}
    detection_classes_style = {'width': '70%', 'display': 'inline-block', 'float': 'right'}
    filter_style = {'width': '0%', 'display': 'none', 'float': 'right'}
    customer_options, sys_options = pc.init_system_and_customer_options(customer_dict, demo)

    if len(customer_dict.keys()) == 1 or demo:
        customer_value = [list(customer_dict)[0]]
        customer_style = {'width': '0%', 'display': 'none'}
        sys_style = {'width': '100%', 'display': 'inline-block'}

    if 'Pats' in customer_dict.keys():
        detection_classes_style = {'width': '40%', 'display': 'inline-block', 'float': 'right'}
        filter_style = {'width': '30%', 'display': 'inline-block', 'float': 'right'}

    return sys_options, sys_style, customer_options, customer_value, customer_style, detection_classes_style, filter_style


def load_systems(username):
    with pc.open_meta_db() as con:
        sql_str = '''SELECT DISTINCT systems.system FROM systems,user_customer_connection,users
                     WHERE  systems.customer_id = user_customer_connection.customer_id AND user_customer_connection.user_id = users.user_id AND users.name = ?
                     ORDER BY systems.system_id '''
        systems = con.execute(sql_str, (username,)).fetchall()
    authorized_systems = [d[0] for d in systems]
    return authorized_systems


def create_lia_filter_sql_str(detection_class_info):
    if 'Anomalies' in detection_class_info['label']:
        return 'monster'
    duration_str = ' AND duration > 1 AND duration < 10'
    detection_str = ' dist_traveled > 0.15 AND dist_traveled < 4 '
    if detection_class_info['lia_label']:
        detection_str += ' AND ' + detection_class_info['lia_label'] + ' > 0.5 '  # 0.5 depends on normalization of result.

    return '((' + detection_str + duration_str + ') OR monster)'


def create_classic_filter_sql_str(start_date, detection_class_info):
    if 'Anomalies' in detection_class_info['label']:
        return 'monster'
    duration_str = ' AND duration > 1 AND duration < 10'
    detection_str = ''
    close_detection_str = ''
    if start_date < datetime.datetime.strptime('20210101_000000', '%Y%m%d_%H%M%S'):  # new (size) filtering was introduced on 24th december 2020 #648
        detection_str = '((version="1.0" AND start_datetime < "20210101_000000") OR ('
        close_detection_str = '))'
    min_size = detection_class_info['avg_size'] - detection_class_info['std_dev']
    max_size = detection_class_info['avg_size'] + 2 * detection_class_info['std_dev']
    floodfill_min_size = detection_class_info['floodfill_avg_size'] - detection_class_info['floodfill_std_dev']
    floodfill_max_size = detection_class_info['floodfill_avg_size'] + 2 * detection_class_info['floodfill_std_dev']
    if detection_class_info['avg_size'] > 0:
        detection_str = detection_str + f'''dist_traveled > 0.15 AND dist_traveled < 4 AND
                                      CASE WHEN CAST (substr(version, 0, instr(version, '.')) AS INTEGER) >= 1 AND CAST (substr(version, instr(version, '.')+1) AS INTEGER) >= 10
                                      THEN size > {floodfill_min_size} AND size < {floodfill_max_size}
                                      ELSE  size > {min_size} AND size < {max_size} END''' + close_detection_str
    else:
        detection_str = detection_str + '''dist_traveled > 0.15 AND dist_traveled < 4 ''' + close_detection_str

    return '((' + detection_str + duration_str + ') OR monster)'


def use_proto(start_date):
    return start_date <= datetime.datetime.strptime('20210401_000000', '%Y%m%d_%H%M%S')


def create_system_filter_sql_str(start_date, system):
    if use_proto(start_date):  # remove proto #533 closed in march 2021, so most systems probably were updated in april
        return f'''(system = '{system}' OR system = '{system.replace('pats', 'pats-proto')}') '''
    else:
        return f'''system = '{system}' '''


def load_detections_df(systems, start_datetime, end_datetime, detection_class_info, selected_filter, columns=[]):
    columns.extend(['system', 'start_datetime', 'monster'])
    detection_df = pd.DataFrame()
    monster_df = pd.DataFrame()
    systems = installation_datetimes(systems)
    with pc.open_data_db() as con:
        for (system, installation_datetime_str) in systems:
            try:
                installation_datetime = datetime.datetime.strptime(installation_datetime_str, '%Y%m%d_%H%M%S')
                real_start_date = max([installation_datetime, start_datetime])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_datetime

            time_str = f'''start_datetime > '{(real_start_date-datetime.timedelta(minutes=5)).strftime('%Y%m%d_%H%M%S')}' AND start_datetime <= '{(end_datetime+datetime.timedelta(minutes=5)).strftime('%Y%m%d_%H%M%S')}' '''  # with added monster window
            system_str = create_system_filter_sql_str(start_datetime, system)
            if selected_filter == 'LIA':
                detection_str = create_lia_filter_sql_str(detection_class_info)
            else:
                detection_str = create_classic_filter_sql_str(start_datetime, detection_class_info)
            sql_str = f'''SELECT {', '.join(set(columns))} FROM detections
                        WHERE {system_str}
                        AND {time_str}
                        AND {detection_str}'''  # nosec see #952
            detection_sys_df = pd.read_sql_query(sql_str, con)
            detection_sys_df['start_datetime'] = pd.to_datetime(detection_sys_df['start_datetime'], format='%Y%m%d_%H%M%S')

            monster_sys_df = detection_sys_df.loc[detection_sys_df['monster'] == 1]

            if 'Anomalies' not in detection_class_info['label']:
                detection_sys_df = detection_sys_df.loc[detection_sys_df['monster'] != 1]
                detection_sys_df = detection_sys_df[(detection_sys_df['start_datetime'] > real_start_date) & (detection_sys_df['start_datetime'] < end_datetime)]  # remove the added monster window added to detect monster just outside the user selected window
                detection_sys_df = pc.window_filter_monster(detection_sys_df, monster_sys_df)
                monster_sys_df = monster_sys_df[(monster_sys_df['start_datetime'] > real_start_date) & (monster_sys_df['start_datetime'] < end_datetime)]  # remove the added monster window added to detect monster just outside the user selected window
                detection_df = pd.concat([detection_df, detection_sys_df])
            else:
                monster_sys_df = monster_sys_df[(monster_sys_df['start_datetime'] > real_start_date) & (monster_sys_df['start_datetime'] < end_datetime)]  # remove the added monster window added to detect monster just outside the user selected window
                detection_df = pd.concat([detection_df, monster_sys_df])

            monster_df = pd.concat([monster_df, monster_sys_df])

    if use_proto(start_datetime):
        detection_df['system'].replace({'-proto': ''}, regex=True, inplace=True)
    return detection_df, monster_df


def load_detections_of_hour(systems, start_date, end_date, hour, detection_class_info):
    systems = installation_datetimes(systems)
    hour_strs = [str(hour - 1).rjust(2, '0'), str(hour).rjust(2, '0'), str(hour + 1).rjust(2, '0')]
    detections_df = pd.DataFrame()
    with pc.open_data_db() as con:
        for (system, installation_datetime) in systems:
            try:
                installation_datetime = datetime.datetime.strptime(installation_datetime, '%Y%m%d_%H%M%S')
                real_start_date = max([installation_datetime, start_date])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_date

            system_str = create_system_filter_sql_str(start_date, system)
            if 'lia_label' in detection_class_info:
                detection_str = create_lia_filter_sql_str(detection_class_info)
            else:
                detection_str = create_classic_filter_sql_str(start_date, detection_class_info)

            sql_str = f'''SELECT detections.* FROM detections
                WHERE {system_str}
                AND start_datetime > :start_time AND start_datetime <= :end_time
                AND start_datetime REGEXP '({hour_strs[0]}5(?=[56789])...$)|({hour_strs[1]}....$)|({hour_strs[2]}0(?=[01234])...$)'
                AND {detection_str}'''  # nosec see #952
            detection_sys_df = pd.read_sql_query(sql_str, con, params={'start_time': real_start_date.strftime('%Y%m%d_%H%M%S'), 'end_time': end_date.strftime('%Y%m%d_%H%M%S')})

            detection_sys_df['start_datetime'] = pd.to_datetime(detection_sys_df['start_datetime'], format='%Y%m%d_%H%M%S')

            monster_sys_df = detection_sys_df.loc[detection_sys_df['monster'] == 1]
            if 'Anomalies' not in detection_class_info['label']:
                detection_sys_df = detection_sys_df.loc[detection_sys_df['monster'] != 1]
                detection_sys_df = pc.window_filter_monster(detection_sys_df, monster_sys_df)
                detection_sys_df = detection_sys_df.loc[detection_sys_df['start_datetime'].dt.hour == hour]
                detections_df = pd.concat([detections_df, detection_sys_df])
            else:
                monster_sys_df = monster_sys_df.loc[monster_sys_df['start_datetime'].dt.hour == hour]
                detections_df = pd.concat([detections_df, monster_sys_df])

    if use_proto(start_date):
        detections_df['system'].replace({'-proto': ''}, regex=True, inplace=True)
    return detections_df


def remove_unauthoirized_system(selected_systems):  # this is solely a security related check
    if not selected_systems:
        return selected_systems
    username = current_user.username
    systems = load_systems(username)
    authorized_systems = []
    for s in selected_systems:
        if s in systems:
            authorized_systems.append(s)
    return authorized_systems


def load_detection_data(selected_systems, start_date, end_date, detection_class_info, selected_filter):
    # We count detections from 12 in the afternoon to 12 in the afternoon. Therefore we shift the data in the for loops with 12 hours.
    # So a detection at 23:00 on 14-01 is counted at 14-01 and a detection at 2:00 on 15-01 also at 14-01
    # A detection seen at 13:00 on 14-01 still belongs to 14-01 but when seen at 11:00 on 14-01 is counted at 13-01
    start_datetime = start_date + datetime.timedelta(hours=12)
    end_datetime = end_date + datetime.timedelta(hours=12)
    detection_df, monster_df = load_detections_df(selected_systems, start_datetime, end_datetime, detection_class_info, selected_filter)  # Shift to include the detections of today until 12:00 in the afternoon.
    unique_dates = pd.date_range(start_date, end_date - datetime.timedelta(days=1), freq='d')

    hist_data = pd.DataFrame(index=unique_dates, columns=selected_systems)
    hist_24h_data = pd.DataFrame(index=range(0, 24), columns=selected_systems)
    for system, customer in (detection_df[['start_datetime']] - datetime.timedelta(hours=12)).groupby(detection_df.system):
        hist_data[system] = customer.groupby(customer.start_datetime.dt.date).count()
        hist_24h_data[system] = customer.groupby(customer.start_datetime.dt.hour).count()
    hist_data.fillna(0, inplace=True)
    hist_24h_data.fillna(0, inplace=True)

    heatmap_detection_counts = pd.DataFrame(np.zeros((24, len(unique_dates))), index=range(24), columns=(unique_dates.strftime('%Y%m%d_%H%M%S')))  # pylint: disable=no-member
    for date, hours in (detection_df[['start_datetime']] - datetime.timedelta(hours=12)).groupby((detection_df.start_datetime - datetime.timedelta(hours=12)).dt.date):
        if date.strftime('%Y-%m-%d') in unique_dates:
            heatmap_detection_counts[date.strftime('%Y%m%d_%H%M%S')] = (hours.groupby(hours.start_datetime.dt.hour).count())
    heatmap_detection_counts = ((heatmap_detection_counts.fillna(0)).to_numpy()).T

    heatmap_monster_df = pd.DataFrame(np.zeros((24, len(unique_dates))), index=range(24), columns=(unique_dates.strftime('%Y%m%d_%H%M%S')))  # pylint: disable=no-member
    for date, hours in (monster_df[['start_datetime']] - datetime.timedelta(hours=12)).groupby((monster_df.start_datetime - datetime.timedelta(hours=12)).dt.date):
        if date.strftime('%Y-%m-%d') in unique_dates:
            heatmap_monster_df[date.strftime('%Y%m%d_%H%M%S')] = (hours.groupby(hours.start_datetime.dt.hour).count())
    heatmap_monster_counts = ((heatmap_monster_df.fillna(0)).to_numpy()).T

    return unique_dates, heatmap_detection_counts, heatmap_monster_counts, hist_data, hist_24h_data


def convert_status_to_number(status):
    if status == 'monitoring':
        return 0
    elif status == 'hunt' or status == 'deployed':
        return -1
    elif status == 'waypoint':
        return -2
    elif status == 'crippled':
        return -3
    elif status == 'wait_for_dark':
        return -4
    elif status == 'down':
        return -5
    else:
        return -666


def insert_down_time(statuses, start_date, end_date, t_start_id, t_end_id):
    down_statuses = []
    if not len(statuses):
        down_statuses.append([start_date.strftime('%Y%m%d_%H%M%S'), end_date.strftime('%Y%m%d_%H%M%S'), 'down'])
        return down_statuses
    prev_end_time = statuses[0][t_start_id]
    for i in range(0, len(statuses)):
        if datetime.datetime.strptime(statuses[i][t_start_id], '%Y%m%d_%H%M%S') - datetime.timedelta(minutes=10) > datetime.datetime.strptime(prev_end_time, '%Y%m%d_%H%M%S'):
            down_statuses.append([prev_end_time, statuses[i][t_start_id], 'down'])
        down_statuses.append(statuses[i])
        prev_end_time = statuses[i][t_end_id]
    return down_statuses


def installation_datetimes(systems):
    with pc.open_meta_db() as con:
        username = current_user.username
        systems = con.execute('''SELECT systems.system, systems.installation_date FROM systems,customers
        JOIN user_customer_connection ON systems.customer_id = user_customer_connection.customer_id
        JOIN users ON users.user_id = user_customer_connection.user_id WHERE
        systems.customer_id = customers.customer_id AND users.name = ? AND systems.system IN (%s)
        ORDER BY systems.system_id''' % ('?,' * len(systems))[:-1], (username, *systems)).fetchall()  # nosec see #952
        return systems


def load_status_data(unique_dates, heatmap_detection_counts, heatmap_monster_counts, systems, start_date, end_date):
    if not len(unique_dates):
        return [], []

    start_datetime = start_date + datetime.timedelta(hours=12)
    end_datetime = end_date + datetime.timedelta(hours=12)

    heatmap_status_data = heatmap_detection_counts.copy()
    heatmap_status_data.fill(-666)
    heatmap_sysdown_counts = heatmap_detection_counts.copy()
    heatmap_sysdown_counts.fill(0)
    t_start_id = 0
    t_end_id = 1
    status_id = 2
    systems = installation_datetimes(systems)

    for (sys, installation_datetime) in systems:
        try:
            installation_datetime = datetime.datetime.strptime(installation_datetime, '%Y%m%d_%H%M%S')
            real_start_datetime = max([installation_datetime, start_date])
        except ValueError as e:
            print(f'ERROR startdate {sys}: ' + str(e))
            real_start_datetime = start_datetime
        start_datetime_str = real_start_datetime.strftime('%Y%m%d_%H%M%S')
        end_datetime_str = end_datetime.strftime('%Y%m%d_%H%M%S')
        sql_str = 'SELECT start_datetime,end_datetime,op_mode FROM status WHERE system="' + sys + '" AND end_datetime > "' + start_datetime_str + '" AND start_datetime < "' + end_datetime_str + '" ORDER BY start_datetime'  # nosec see #952
        with pc.open_data_db() as con:
            statuses = con.execute(sql_str).fetchall()
        statuses = insert_down_time(statuses, real_start_datetime, end_datetime, t_start_id, t_end_id)

        for status in statuses:
            status_start = datetime.datetime.strptime(status[t_start_id], '%Y%m%d_%H%M%S')
            status_end = datetime.datetime.strptime(status[t_end_id], '%Y%m%d_%H%M%S')
            if status_end > status_start:  # there were some wrong logs. This check can possibly be removed at some point.
                tot_hours = round((status_end - status_start).total_seconds() / 3600)
            else:
                continue

            if status_start < start_datetime:
                # the status entrie started before the selected start_datetime.
                start_day = 0
                start_hour = 0
                tot_hours -= round((start_datetime - status_start).seconds / 3600)
            else:
                start_day = (status_start.date() - start_datetime.date()).days
                if status_start.hour < 12:
                    start_day -= 1
                    start_hour = status_start.hour + 12
                else:
                    start_hour = status_start.hour - 12

            status = convert_status_to_number(status[status_id])
            for hour in range(0, tot_hours):
                heatmap_hour_id = (start_hour + hour) % 24
                extra_days = ((start_hour + hour) - heatmap_hour_id) / 24
                if start_day + int(extra_days) >= len(unique_dates):
                    break
                if heatmap_status_data[start_day + int(extra_days), heatmap_hour_id] < convert_status_to_number('down'):  # Multi system mode mixing is ... complex
                    heatmap_status_data[start_day + int(extra_days), heatmap_hour_id] = status
                if status == convert_status_to_number('down'):
                    heatmap_sysdown_counts[start_day + int(extra_days), heatmap_hour_id] += 1
                if status == convert_status_to_number('monitoring') and heatmap_status_data[start_day + int(extra_days), heatmap_hour_id] <= -1:
                    heatmap_status_data[start_day + int(extra_days), heatmap_hour_id] = status

    for i in range(0, heatmap_status_data.shape[0]):
        for j in range(0, heatmap_status_data.shape[1]):
            if heatmap_status_data[i, j] == convert_status_to_number('down') and not heatmap_detection_counts[i, j] and not heatmap_monster_counts[i, j]:
                heatmap_detection_counts[i, j] = Heatmap_Cell.system_down_cell.value
            elif heatmap_status_data[i, j] < -1 and not heatmap_detection_counts[i, j] and not heatmap_monster_counts[i, j]:
                heatmap_detection_counts[i, j] = Heatmap_Cell.system_offline_cell.value
    return heatmap_detection_counts, heatmap_sysdown_counts


def natural_sort_systems(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(line, key=alphanum_key)


def create_heatmap(unique_dates, detection_counts, monster_counts, sysdown_counts, xlabels, selected_heat):
    detection_count_hover_label = pd.DataFrame(detection_counts).astype(int).astype(str)
    detection_count_hover_label[detection_count_hover_label == str(Heatmap_Cell.system_down_cell.value)] = 'NA'
    detection_count_hover_label[detection_count_hover_label == str(Heatmap_Cell.system_offline_cell.value)] = 'NA'
    monster_count_hover_label = pd.DataFrame(monster_counts).astype(int).astype(str)
    sysdown_count_hover_label = pd.DataFrame(sysdown_counts).astype(int).astype(str)
    heatmap_data = np.clip(detection_counts, Heatmap_Cell.system_down_cell.value, heatmap_max)

    if selected_heat:
        selected_heat = pd.read_json(selected_heat, orient='split')
        for _, cel in selected_heat.iterrows():
            x = cel['x']
            y = (unique_dates.strftime('%d-%m-%Y').tolist()).index(cel['start_date'])
            if heatmap_data[y, x] >= 0:
                heatmap_data[y, x] = Heatmap_Cell.selected_cell.value

    heatmap_dates = pd.DataFrame()
    for i in range(0, 24):
        if i >= 12:
            heatmap_dates[i] = pd.DataFrame((unique_dates + pd.DateOffset(1)) .strftime('%d-%m-%Y'))
        else:
            heatmap_dates[i] = pd.DataFrame(unique_dates.strftime('%d-%m-%Y'))

    hm = go.Heatmap(
        x=xlabels,
        y=unique_dates.strftime('%d-%m-%Y'),
        z=heatmap_data,
        customdata=np.stack((detection_count_hover_label, monster_count_hover_label, sysdown_count_hover_label, heatmap_dates), axis=-1),
        zmin=Heatmap_Cell.selected_cell.value,
        zmid=heatmap_max / 2,
        zmax=heatmap_max,
        colorscale=[
            [0, 'rgba(128, 128, 200, 0.65)'],  # selected cell
            [1 / (heatmap_max + 3), 'rgba(5,5,75,1.0)'],  # system down
            [2 / (heatmap_max + 3), 'rgba(0, 0, 0, 1.0)'],  # system inactive
            [3 / (heatmap_max + 3), 'rgba(0,255,0, 1.0)'],
            [1, 'rgba(255,0,0, 1.0)']
        ],
        hovertemplate='<b>Insects: %{customdata[0]}</b><br>' +
        'Date: %{customdata[3]}<br>' +
        'Time: %{x}<br>' +
        'Anomalies: %{customdata[1]}<br>' +
        'Systems down: %{customdata[2]}' +
        '<extra></extra>'
    )
    fig = go.Figure(data=hm)
    fig['layout']['xaxis']['side'] = 'top'
    fig['layout']['xaxis']['tickangle'] = 45

    if len(unique_dates) > 14:
        h = len(unique_dates) * 15 + 200
    elif len(unique_dates):
        h = len(unique_dates) * 30 + 200
    else:
        h = 10
    fig.update_layout(
        height=h,
        title_text='Activity summed per hour',
        clickmode='event+select'
    )
    style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
    return fig, style


def update_heatmap(selected_heat, unselected_heat, fig: go.Heatmap):
    heatmap_data = np.array(fig['data'][0]['z'])
    unique_dates = fig['data'][0]['y']
    if selected_heat:
        selected_heat = pd.read_json(selected_heat, orient='split')
        for _, cel in selected_heat.iterrows():
            x = cel['x']
            y = unique_dates.index(cel['start_date'])
            if heatmap_data[y, x] >= 0:
                heatmap_data[y, x] = Heatmap_Cell.selected_cell.value - heatmap_data[y, x]
    if unselected_heat:
        unselected_heat = pd.read_json(unselected_heat, orient='split')
        for _, cel in unselected_heat.iterrows():
            x = cel['x']
            y = unique_dates.index(cel['start_date'])
            if heatmap_data[y, x] <= Heatmap_Cell.selected_cell.value:
                heatmap_data[y, x] = Heatmap_Cell.selected_cell.value - heatmap_data[y, x]
    fig['data'][0]['z'] = heatmap_data
    return fig


def create_hist(df_hist, unique_dates, system_labels):
    fig = go.Figure()
    fig.update_yaxes(rangemode="nonnegative")
    bar_totals = df_hist.sum(axis=1).astype(int)
    for cnt, sys in enumerate(system_labels.keys()):
        hist_data = df_hist[sys]
        sys_str = [system_labels[sys]] * len(hist_data)
        sys_names = [sys] * len(hist_data)
        hist = go.Bar(
            x=unique_dates.strftime('%d-%m-%Y'),
            y=hist_data,
            customdata=np.transpose([sys_str, sys_names, hist_data.astype(int), bar_totals]),
            marker_color=px.colors.qualitative.Vivid[cnt % (len(px.colors.qualitative.Vivid))],
            name=system_labels[sys],
            hovertemplate='<b>%{customdata[0]}</b><br>Count: %{customdata[2]} / %{customdata[3]}<br><extra></extra>'
        )

        fig.add_trace(hist)

    if len(unique_dates) > 20:
        df_sum = df_hist.sum(axis=1).to_frame('total_counts')
        df_sum.ta.ema(close='total_counts', length=10, append=True)
        df_sum = df_sum.iloc[10:]
        unique_dates_for_ema10_trend = unique_dates[10:]
        trend_line_ema_10 = go.Scatter(
            x=unique_dates_for_ema10_trend.strftime('%d-%m-%Y'),
            y=df_sum['EMA_10'],
            mode="lines",
            marker_color='white',
            opacity=0.75,
            name='Trend',
            line_shape='spline'
        )
        fig.add_trace(trend_line_ema_10)

    # this trend line is a bit simpler, and it starts from the start, but I think the ema follows the real trend a bit better.
    # if len(unique_dates) > 10:
    #     filtered_hist_data = df_hist.sum(axis=1).rolling(5, win_type='gaussian', min_periods=1, center=True).mean(std=3)
    #     trend_line_gauss = go.Scatter(
    #         x=unique_dates.strftime('%d-%m-%Y'),
    #         y=filtered_hist_data,
    #         mode="lines",
    #         marker_color='yellow',
    #         name='Trend'
    #     )
    #     fig.add_trace(trend_line_gauss)

    fig.update_layout(
        title_text='Activity summed per day',
        xaxis_title_text='Days',
        yaxis_title_text='Counts',
        barmode='stack',
        clickmode='event+select'
    )
    hist_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
    return fig, hist_style


def create_24h_hist(hist_24h_data, hour_labels, system_labels):
    fig = go.Figure()
    fig.update_yaxes(rangemode="nonnegative")
    bar_totals = hist_24h_data.sum(axis=1).astype(int)
    for cnt, sys in enumerate(system_labels.keys()):
        hist_data = hist_24h_data[sys]
        sys_str = [system_labels[sys]] * len(hist_data)
        sys_names = [sys] * len(hist_data)
        hist = go.Bar(
            x=hour_labels,
            y=hist_data,
            customdata=np.transpose([sys_str, sys_names, hist_data.astype(int), bar_totals]),
            marker_color=px.colors.qualitative.Vivid[cnt % (len(px.colors.qualitative.Vivid))],
            name=system_labels[sys],
            hovertemplate='<b>%{customdata[0]}</b><br>Count: %{customdata[2]} / %{customdata[3]}<br><extra></extra>'
        )
        fig.add_trace(hist)

    fig.update_layout(
        title_text='Activity throughout the day',
        xaxis_title_text='Hours',
        yaxis_title_text='Counts',
        barmode='stack',
        clickmode='event+select'
    )
    hist_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
    return fig, hist_style


def classification_to_symbol(x):
    if not x:
        return 0
    return classification_options.index(x)


def remove_nones_classification(x):
    if not x:
        return classification_options[0]
    return x


def video_available_to_symbol(x):
    if not x or x.startswith('NA'):
        return 0
    return 300  # see "Custom Marker Symbols" https://plotly.com/python/marker-style/


def create_scatter_hover(df, selected_filter, chance_columns, current_labels):
    customdata = np.stack(
        (df['system'] + '/' + df['folder'] + '/' + df['filename'],
            df['video_filename'],
            df['human_classification'],
            current_labels,
            df['uid'],
            df['time_for_humans'],
         ), axis=-1)

    hovertemplate = '<b>System %{customdata[3]}</b><br><br>'\
        + 'x: %{x}<br>'\
        + 'y: %{y}<br>'\
        + 't: %{customdata[5]}<br>'\
        + 'File: %{customdata[0]}<br>'\
        + 'Video: %{customdata[1]}<br>'\
        + 'Ground truth: %{customdata[2]}<br>'
    if selected_filter == 'LIA':
        len_custom = customdata.shape[1]
        chance_stack = np.stack(df[chance_columns].values, axis=0)
        index_top3 = chance_stack.argsort(axis=1)[:, -3:][:, ::-1]
        chance_col_stack = np.broadcast_to(np.array(chance_columns), chance_stack.shape)
        chance_col_stack_top3 = chance_col_stack[np.arange(chance_col_stack.shape[0])[:, None], index_top3]
        chance_stack_top3 = chance_stack[np.arange(chance_stack.shape[0])[:, None], index_top3]
        customdata = np.concatenate([customdata, chance_col_stack_top3, chance_stack_top3], axis=1)
        hovertemplate += ''.join(['%{customdata[' + str(i + len_custom) + ']}' + ': %{customdata[' + str(i + len_custom + 3) + ']:.2f}<br>' for i in range(3)])

    hovertemplate += '<extra></extra>'

    return hovertemplate, customdata


def create_scatter(detections, system_labels, scatter_x_value, scatter_y_value, selected_filter, chance_columns):
    df_scatter = detections
    df_scatter['system_ids'] = df_scatter['system'].str.replace('pats-proto', '').str.replace('pats', '').astype(int)
    df_scatter['system_color'] = df_scatter['system'].apply(list(system_labels.keys()).index).astype(int)
    df_scatter['classification_symbol'] = df_scatter['human_classification'].apply(classification_to_symbol)
    df_scatter['human_classification'] = df_scatter['human_classification'].apply(remove_nones_classification)
    df_scatter['video_symbol'] = df_scatter['video_filename'].apply(video_available_to_symbol)
    df_scatter['symbol'] = df_scatter['video_symbol'] + df_scatter['classification_symbol']

    scat_fig = go.Figure()
    scat_fig.update_yaxes({'range': (np.max(df_scatter[scatter_y_value].min() - 0.005, 0), df_scatter[scatter_y_value].max() + 0.005)})
    for sys in system_labels.keys():
        df_scatter_sys = df_scatter[df_scatter['system'] == sys]
        for classification in classification_options:
            tmp1 = list(df_scatter_sys['human_classification'] == classification)  # which rows have the same classification
            df = df_scatter_sys[tmp1]  # retrieve only those rows that have the same classficication
            df = df.fillna('')
            if not df.empty:
                df['time_for_humans'] = df['start_datetime'].dt.strftime('%H:%M:%S %d-%m-%Y')
                current_labels = [system_labels[sys] for x in df[scatter_x_value]]
                hovertemplate, customdata = create_scatter_hover(df, selected_filter, chance_columns, current_labels)

                scatter = go.Scattergl(
                    name=system_labels[sys] + ', ' + classification,
                    showlegend=True,
                    x=df[scatter_x_value],
                    y=df[scatter_y_value],
                    mode='markers',
                    customdata=customdata,
                    marker=dict(
                        cmin=0,
                        cmax=9,
                        color=px.colors.qualitative.Vivid[df['system_color'].values[0] % (len(px.colors.qualitative.Vivid))],
                        symbol=df_scatter['symbol'],
                        size=10,
                        line=dict(width=2, color="rgba(0,0, 0, 255)")
                    ),
                    hovertemplate=hovertemplate
                )
                scat_fig.add_trace(scatter)

    scat_fig.update_layout(
        title_text='Selected insects:',
        xaxis_title=scatter_columns[scatter_x_value],
        yaxis_title=scatter_columns[scatter_y_value],
        clickmode='event+select',
        legend_title_text='Legend',
    )
    return scat_fig


def download_log(selected_detection, detection_columns):
    sys_name = selected_detection[detection_columns.index('system')].replace('-proto', '')
    log_fn = selected_detection[detection_columns.index('filename')]
    log_folder = selected_detection[detection_columns.index('folder')]
    if not log_folder or log_fn == 'unknown':
        return ''
    target_log_fn = './static/' + log_folder + '_' + sys_name + '_' + log_fn
    if not os.path.isfile(target_log_fn):
        rsync_src = sys_name + ':pats/data/processed/' + log_folder + '/logging/' + log_fn
        cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_log_fn]
        cmd_result = pc.execute(cmd)
        if cmd_result == 23:  # Error 23 means that the file doesn't exist on the system so try again without logging
            rsync_src = sys_name + ':pats/data/processed/' + log_folder + '/' + log_fn
            cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_log_fn]
            cmd_result = pc.execute(cmd)
    return target_log_fn


def file_download_link(filename):
    location = '/{}'.format(urlquote(filename))
    return html.A('Download log', href=location, style={'display': 'block'}), {'textAlign': 'center', 'width': '25%', 'margin': 'auto', 'display': 'block'}


def create_path_plot(target_log_fn, selected_detection, detection_columns):
    df_ilog = pd.read_csv(target_log_fn, delimiter=';')
    target_log_fn = '/' + target_log_fn
    rows_without_tracking = df_ilog[df_ilog['n_frames_tracking_insect'] == 0].index
    df_ilog = df_ilog.drop(rows_without_tracking)
    scatter = go.Scatter3d(
        x=-df_ilog['sposX_insect'],
        y=-df_ilog['sposZ_insect'],
        z=df_ilog['sposY_insect'],
        text=df_ilog['time' if 'time' in df_ilog else 'elapsed'],
        mode='markers',
        name='Flight path',
        hovertemplate='<b>t= %{text}</b><br>x= %{x}<br>y= %{y}<br>z= %{z}'
    )
    camera_pos = go.Scatter3d(
        x=(0,), y=(0,), z=(0,),
        mode='markers',
        marker=dict(
            symbol='circle',
            color='red'),
        name='Camera position',
        hovertemplate='Camera position'
    )
    fig = go.Figure(data=[scatter, camera_pos])
    title_str = 'Flight path of detecton at: ' + datetime.datetime.strptime(str(selected_detection[detection_columns.index('start_datetime')]), '%Y%m%d_%H%M%S').strftime('%d-%m-%Y %H:%M:%S')
    fig.update_layout(
        title_text=title_str,
        scene=dict(
            aspectmode='data'
        )
    )
    style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
    return fig, style


def download_video(selected_detection, detection_columns):
    video_fn = selected_detection[detection_columns.index('video_filename')]
    sys_name = selected_detection[detection_columns.index('system')].replace('-proto', '')

    target_video_mp4_fn = ''
    if video_fn and not video_fn.startswith('NA'):
        target_video_mkv_fn = 'static/' + selected_detection[detection_columns.index('folder')] + '_' + sys_name + '_' + video_fn
        target_video_mp4_fn = target_video_mkv_fn[0:-3] + 'mp4'

        if not os.path.isfile(target_video_mkv_fn) and not os.path.isfile(target_video_mp4_fn):
            rsync_src = sys_name + ':pats/data/processed/' + selected_detection[detection_columns.index('folder')] + '/logging/render_' + video_fn
            cmd = ['rsync --timeout=5 -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_video_mkv_fn]
            cmd_result = pc.execute(cmd)
            if cmd_result == 23:  # Error 23 means that the file doesn't exist on the system so try again without logging
                rsync_src = sys_name + ':pats/data/processed/' + selected_detection[detection_columns.index('folder')] + '/render_' + video_fn
                cmd = ['rsync --timeout=5 -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_video_mkv_fn]
                pc.execute(cmd)
        if not os.path.isfile(target_video_mp4_fn) and os.path.isfile(target_video_mkv_fn):
            mp4_to_mkv_cmd = ['ffmpeg -y -i ' + target_video_mkv_fn + ' -c:v copy -an ' + target_video_mkv_fn[0:-3] + 'mp4']
            pc.execute(mp4_to_mkv_cmd)
        if not os.path.isfile(target_video_mp4_fn):
            return ''
        target_video_mp4_fn = '/' + target_video_mp4_fn

    return target_video_mp4_fn


def dash_application():
    print("Starting PATS-C dashboard!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/pats-c/', title='PATS-C')

    pio.templates.default = 'plotly_dark'

    @ app.callback(
        Output('systems_dropdown', 'value'),
        Input('customers_dropdown', 'value'))
    def select_customer(selected_customer):  # pylint: disable=unused-variable
        customer_dict, demo = pc.load_customers()
        value = []
        if selected_customer:
            for customer in selected_customer:
                if demo:  # dirty demo hack part deux
                    systems = customer_dict[customer][0][0]
                else:
                    systems = [d[0] for d in customer_dict[customer]]
                value.extend(systems)
        return value

    @ app.callback(
        Output('detection_class_dropdown', 'options'),
        Output('detection_class_dropdown', 'value'),
        Input('filter_dropdown', 'value'),
        State('detection_class_dropdown', 'value'))
    def init_detection_options(selected_filter, detection_class_info):
        detections_options = []
        if selected_filter == 'size':
            detections_options = init_classic_options()
        elif selected_filter == 'LIA':
            detections_options = init_lia_options()

        new_value = None
        if detection_class_info:
            for option in detections_options:
                if option['label'] == detection_class_info['label']:
                    new_value = option['value']

        if not new_value:
            new_value = detections_options[0]['value']

        return detections_options, new_value

    def update_selected_heat(clickData_hm, selected_heat):
        if not clickData_hm:
            return None, None
        else:
            columns = ['x', 'start_date', 'hour']  # start_date -> python bug prevents me from using 'date' https://stackoverflow.com/questions/48369578/prevent-pandas-to-json-from-adding-time-component-to-date-object
            unselected_heat = pd.DataFrame(columns=columns)
            for cel in clickData_hm['points']:
                if cel['z'] == Heatmap_Cell.system_offline_cell.value or cel['z'] == Heatmap_Cell.system_down_cell.value:
                    return None, selected_heat  # if a black cell is clicked, interpret that as cancelling the whole selection

                x = hour_labels.index(cel['x'])

                unclicked = False

                if not selected_heat:
                    selected_heat = pd.DataFrame(columns=columns)
                else:
                    selected_heat = pd.read_json(selected_heat, orient='split')
                if cel['z'] <= Heatmap_Cell.selected_cell.value:
                    for index_heat, heat in selected_heat.iterrows():
                        if int(cel['x'].replace('h', '')) == heat['hour'] and cel['y'] == heat['start_date']:
                            unselected_heat = pd.concat([unselected_heat, heat.to_frame().T])
                            selected_heat = selected_heat.drop(index=index_heat)
                            unclicked = True
                            break

                if not unclicked:
                    df_row = pd.DataFrame([[x, cel['y'], int(hour_labels[x].replace('h', ''))]], columns=columns)
                    if selected_heat.empty:
                        selected_heat = df_row
                    else:
                        selected_heat = pd.concat([selected_heat, df_row], ignore_index=True)

            selected_heat = selected_heat.to_json(date_format='iso', orient='split')
            unselected_heat = unselected_heat.to_json(date_format='iso', orient='split')
        return selected_heat, unselected_heat

    @ app.callback(
        Output('hete_kaart', 'figure'),
        Output('staaf_kaart', 'figure'),
        Output('staaf24h_kaart', 'figure'),
        Output('hete_kaart', 'style'),
        Output('staaf_kaart', 'style'),
        Output('staaf24h_kaart', 'style'),
        Output('selected_heatmap_data', 'children'),
        Output('Loading_animation', 'style'),
        Output('download_csv_container', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('detection_class_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        State('filter_dropdown', 'value'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown', 'options'),
        State('hete_kaart', 'figure'),
        State('staaf_kaart', 'figure'),
        State('staaf24h_kaart', 'figure'),
        State('hete_kaart', 'style'),
        State('staaf_kaart', 'style'),
        State('staaf24h_kaart', 'style'))
    def update_ui_hist_and_heat(start_date, end_date, selected_systems, selected_detection_class, clickData_hm, selected_filter, selected_heat, system_options, hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style):  # pylint: disable=unused-variable
        loading_animation_style = {'display': 'block'}
        system_labels = {x['value']: x['label'] for x in system_options if x['value'] in selected_systems}
        download_csv_style = {'display': 'none'}

        prop_id = dash.callback_context.triggered[0]['prop_id']
        if prop_id == 'date_range_picker.value' or prop_id == 'systems_dropdown.value' or prop_id == 'detection_class_dropdown.value':
            selected_heat = None
            clickData_hm = None
        selected_systems = remove_unauthoirized_system(selected_systems)
        if start_date and end_date:
            end_date = datetime.datetime.strptime(end_date, '%Y-%m-%d')
            start_date = datetime.datetime.strptime(start_date, '%Y-%m-%d')
            end_date = datetime.datetime.combine(end_date, datetime.datetime.min.time())  # the 12:00 O'clock start time is not included here. (so not a datetime, but a date)
            start_date = datetime.datetime.combine(start_date, datetime.datetime.min.time())  # idem
        if not selected_systems or not selected_detection_class or not end_date or not start_date or (end_date - start_date).days < 1:
            hm_style = {'display': 'none'}
            hist_style = {'display': 'none'}
            hist24h_style = {'display': 'none'}
            return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style, download_csv_style

        if prop_id != 'hete_kaart.clickData' and prop_id != 'classification_dropdown.value':
            unique_dates, heatmap_detection_counts, heatmap_monster_counts, df_hist, hist_24h_data = load_detection_data(selected_systems, start_date, end_date, selected_detection_class, selected_filter)
            if not len(unique_dates):
                hm_style = {'display': 'none'}
                hist_style = {'display': 'none'}
                hist24h_style = {'display': 'none'}
                return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style, download_csv_style

            hist_fig, hist_style = create_hist(df_hist, unique_dates, system_labels)
            hist24h_fig, hist24h_style = create_24h_hist(hist_24h_data, hour_labels, system_labels)

            heatmap_detection_counts, sysdown_heatmap_counts = load_status_data(unique_dates, heatmap_detection_counts, heatmap_monster_counts, selected_systems, start_date, end_date)  # add status info to heatmap, which makes the heatmap show when the system was online (color) or offline (black)
            hm_fig, hm_style = create_heatmap(unique_dates, heatmap_detection_counts, heatmap_monster_counts, sysdown_heatmap_counts, hour_labels, selected_heat)
        elif prop_id == 'hete_kaart.clickData' or prop_id == 'classification_dropdown.value':
            selected_heat, unselected_heat = update_selected_heat(clickData_hm, selected_heat)
            hm_fig = update_heatmap(selected_heat, unselected_heat, hm_fig)
        loading_animation_style = {'display': 'none'}
        download_csv_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%', 'textAlign': 'center'}

        return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style, download_csv_style

    @app.callback(
        Output("download_csv", "data"),
        State('date_range_picker', 'start_date'),
        State('date_range_picker', 'end_date'),
        State('systems_dropdown', 'value'),
        State('detection_class_dropdown', 'value'),
        State('filter_dropdown', 'value'),
        Input("btn_csv", "n_clicks"),
        prevent_initial_call=True,
    )
    def download_csv_click(start_date_str, end_date_str, systems, detection_class_info, selected_filter, n_clicks):
        if not start_date_str or not end_date_str or not systems or not selected_filter:
            return []

        end_date = datetime.datetime.strptime(end_date_str, '%Y-%m-%d')
        start_date = datetime.datetime.strptime(start_date_str, '%Y-%m-%d')
        end_datetime = datetime.datetime.combine(end_date, datetime.datetime.min.time()) + datetime.timedelta(hours=12)
        start_datetime = datetime.datetime.combine(start_date, datetime.datetime.min.time()) + datetime.timedelta(hours=12)

        csv_fn = ''
        for sys_name in systems:
            csv_fn += sys_name + '_'
        csv_fn += pc.datetime_to_str(start_datetime) + '_' + pc.datetime_to_str(end_datetime) + '_' + selected_filter + '.csv'

        columns = ['system', 'start_datetime', 'duration', 'vel_mean', 'vel_std', 'vel_max', 'dist_traveled', 'dist_traject', 'size', 'monster']
        detection_df, _ = load_detections_df(systems, start_datetime, end_datetime, detection_class_info, selected_filter, columns)
        detection_df.drop('monster', 1, inplace=True)

        return dcc.send_data_frame(detection_df.to_csv, csv_fn, index=False)

    @ app.callback(
        Output('verstrooide_kaart', 'figure'),
        Output('verstrooide_kaart', 'style'),
        Output('scatter_dropdown_container', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('detection_class_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'selectedData'),
        Input('staaf24h_kaart', 'selectedData'),
        Input('scatter_x_dropdown', 'value'),
        Input('scatter_y_dropdown', 'value'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown', 'options'),
        State('filter_dropdown', 'value'))
    def update_ui_scatter(start_date_str, end_date_str, selected_systems, selected_detection_class, clickData_hm, hist_selected_bars, hist24h_selected_bars, scatter_x_value, scatter_y_value, selected_heat, system_options, selected_filter):  # pylint: disable=unused-variable
        scat_fig = go.Figure(data=go.Scatter())
        scat_style = {'display': 'none'}
        scat_axis_select_style = {'display': 'none'}
        if system_options and selected_systems:
            system_labels = {x['value']: x['label'] for x in system_options if x['value'] in selected_systems}
        else:
            system_labels = None

        selected_systems = remove_unauthoirized_system(selected_systems)
        if not selected_systems or not selected_detection_class:
            return scat_fig, scat_style, scat_axis_select_style

        prop_id = dash.callback_context.triggered[0]['prop_id']
        axis_change = prop_id == 'scatter_x_dropdown.value' or prop_id == 'scatter_y_dropdown.value'
        if prop_id == 'staaf_kaart.selectedData' or prop_id == 'staaf24h_kaart.selectedData' or prop_id == 'hete_kaart.clickData' or axis_change:
            if prop_id == 'hete_kaart.clickData':
                selected_heat, _ = update_selected_heat(clickData_hm, selected_heat)
            elif prop_id == 'staaf_kaart.selectedData' or prop_id == 'staaf24h_kaart.selectedData':
                selected_heat = None
            detections = pd.DataFrame()
            if selected_heat and (prop_id == 'hete_kaart.clickData' or axis_change):
                selected_heat = pd.read_json(selected_heat, orient='split')
                for _, hm_cell in selected_heat.iterrows():
                    hour = hm_cell['hour']
                    if hour < 12:
                        hour += 24
                    start_datetime = datetime.datetime.strptime(hm_cell['start_date'], '%d-%m-%Y') + datetime.timedelta(hours=hour)
                    detections_hour, _ = load_detections_df(selected_systems, start_datetime, start_datetime + datetime.timedelta(hours=1), selected_detection_class, selected_filter, ['uid', 'video_filename', 'folder', 'filename', 'human_classification', scatter_x_value, scatter_y_value, *chance_columns])
                    detections = pd.concat([detections, detections_hour])
            elif hist_selected_bars and (prop_id == 'staaf_kaart.selectedData' or axis_change):
                for bar in hist_selected_bars['points']:
                    sys = bar['customdata'][1]
                    start_datetime = datetime.datetime.strptime(bar['x'], '%d-%m-%Y') + datetime.timedelta(hours=12)
                    detections_day, _ = load_detections_df([sys], start_datetime, start_datetime + datetime.timedelta(days=1), selected_detection_class, selected_filter, ['uid', 'video_filename', 'folder', 'filename', 'human_classification', scatter_x_value, scatter_y_value, *chance_columns])
                    detections = pd.concat([detections, detections_day])
            elif hist24h_selected_bars and (prop_id == 'staaf24h_kaart.selectedData' or axis_change):
                start_date = datetime.datetime.strptime(start_date_str, '%Y-%m-%d')
                end_date = datetime.datetime.strptime(end_date_str, '%Y-%m-%d')
                for bar in hist24h_selected_bars['points']:
                    sys = bar['customdata'][1]
                    hour = int(bar['x'].replace('h', ''))
                    detections_hour = load_detections_of_hour([sys], start_date, end_date, hour, selected_detection_class)
                    detections = pd.concat([detections, detections_hour])

            if not detections.empty:
                scat_fig = create_scatter(detections, system_labels, scatter_x_value, scatter_y_value, selected_filter, chance_columns)
                scat_axis_select_style = {'display': 'table', 'textAlign': 'center', 'width': '50%', 'margin': 'auto'}
                scat_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
        return scat_fig, scat_style, scat_axis_select_style

    @ app.callback(
        Output('detection_video', 'src'),
        Output('detection_video', 'style'),
        Output('route_kaart', 'figure'),
        Output('route_kaart', 'style'),
        Output('log_file_link', 'children'),
        Output('log_file_link_container', 'style'),
        Output('selected_point_scatter', 'children'),
        Output('classification_dropdown', 'value'),
        Output('classify_container', 'style'),
        Output('Loading_animation_detection', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('detection_class_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'clickData'),
        Input('staaf24h_kaart', 'clickData'),
        Input('verstrooide_kaart', 'clickData'),
        State('verstrooide_kaart', 'figure'))
    def update_detection_ui(start_date, end_date, selected_systems, selected_detections, clickData_hm, clickData_hist, clickData_hist24h, clickData_dot, scatter_fig_state):  # pylint: disable=unused-variable
        target_video_fn = ''
        video_style = {'display': 'none'}
        path_fig = go.Figure(data=go.Scatter3d())
        path_style = {'display': 'none'}
        file_link = html.A('File not available.', style={'display': 'none'})
        file_link_style = {'textAlign': 'center', 'width': '25%', 'margin': 'auto', 'display': 'none'}
        selected_detection = []
        classification = classification_options[0]
        classify_style = {'display': 'none'}
        loading_animation_style = {'display': 'block'}

        selected_systems = remove_unauthoirized_system(selected_systems)
        prop_id = dash.callback_context.triggered[0]['prop_id']
        if 'selectedpoints' not in scatter_fig_state['data'][0] or prop_id != 'verstrooide_kaart.clickData' or not selected_systems or not selected_detections:
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_detection, classification, classify_style, loading_animation_style

        sql_str = 'SELECT * FROM detections WHERE uid=:uid'
        sql_params = {'uid': str(clickData_dot['points'][0]['customdata'][4])}
        with pc.open_data_db() as con:
            entry = con.execute(sql_str, sql_params).fetchall()
        if not len(entry):
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_detection, classification, classify_style, loading_animation_style
        selected_detection = entry[0]

        target_log_fn = download_log(selected_detection, detection_columns)
        if not os.path.isfile(target_log_fn):
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_detection, classification, classify_style, loading_animation_style
        file_link, file_link_style = file_download_link(target_log_fn)

        if 'human_classification' in detection_columns:
            classification = selected_detection[detection_columns.index('human_classification')]
        if not classification:
            classification = classification_options[0]
        classify_style = {'display': 'block', 'textAlign': 'center', 'width': '25%', 'margin': 'auto'}

        path_fig, path_style = create_path_plot(target_log_fn, selected_detection, detection_columns)

        target_video_fn = download_video(selected_detection, detection_columns)
        loading_animation_style = {'display': 'none'}
        if target_video_fn != '':
            video_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}

        return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_detection, classification, classify_style, loading_animation_style

    @ app.callback(
        Output(component_id='classification_hidden', component_property='children'),
        Input('classification_dropdown', 'value'),
        Input('selected_point_scatter', 'children'))
    def classification_value(selected_classification, selected_detection):  # pylint: disable=unused-variable
        prop_id = dash.callback_context.triggered[0]['prop_id']
        if not selected_detection or prop_id != 'classification_dropdown.value':
            return []

        current_classification = selected_detection[detection_columns.index('human_classification')]
        if not current_classification:
            current_classification = classification_options[0]
        if current_classification != selected_classification:
            sql_str = 'UPDATE detections SET human_classification=:selected_classification WHERE uid=:uid', {'selected_classification': selected_classification, 'uid': str(selected_detection[detection_columns.index('uid')])}
            with pc.open_data_db() as con:
                con.execute(*sql_str)

            # also save user classifications to a seperate database, because the detections database sometimes needs to be rebuild from the logs/jsons
            with pc.open_classification_db() as con_class:
                table_exists = con_class.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='classification_records' ''').fetchone()[0]
                if not table_exists:
                    sql_create = 'CREATE TABLE classification_records(uid INTEGER PRIMARY KEY,moth_uid INTEGER,system TEXT,time TEXT,duration REAL,user TEXT,classification TEXT)'
                    con_class.execute(sql_create)
                    con_class.commit()

                username = current_user.username
                sql_insert = 'INSERT INTO classification_records(moth_uid,system,time,duration,user,classification) VALUES(?,?,?,?,?,?)'
                data = [selected_detection[detection_columns.index('uid')], str(selected_detection[detection_columns.index('system')]), str(selected_detection[detection_columns.index('start_datetime')]), str(selected_detection[detection_columns.index('duration')]), username, selected_classification]
                con_class.execute(sql_insert, data)
                con_class.commit()

        return selected_classification

    with pc.open_data_db() as con:
        detection_columns = [i[1] for i in con.execute('PRAGMA table_info(detections)')]
        chance_columns = [col for col in detection_columns if 'chance_' in col]
        if 'human_classification' not in detection_columns:
            con.execute('ALTER TABLE detections ADD COLUMN human_classification TEXT')

            if os.path.exists(pc.db_classification_path):
                with pc.open_classification_db() as con_class:
                    classification_columns = [i[1] for i in con_class.execute('PRAGMA table_info(classification_records)')]
                    sql_str = 'SELECT * FROM classification_records'
                    classification_data = con_class.execute(sql_str).fetchall()

                for entry in classification_data:
                    sql_where = ' WHERE uid=' + str(entry[classification_columns.index('moth_uid')]) + ' AND time="' + str(entry[classification_columns.index('start_datetime')]) + '" AND duration=' + str(entry[classification_columns.index('duration')])
                    sql_str = 'UPDATE detections SET human_classification="' + entry[classification_columns.index('classification')] + '"' + sql_where
                    con.execute(sql_str)
                con.commit()
                print('Re-added cliassification results to main db. Added ' + str(len(classification_data)) + ' classifications.')

    hour_labels = []
    for i in range(0, 24):
        hour_labels.append(str((i + 12) % 24) + 'h')

    def make_layout():
        fig_hm = go.Figure(data=go.Heatmap())
        fig_hist = go.Figure(data=go.Histogram())
        fig_hist24h = go.Figure(data=go.Histogram())
        fig_scatter = go.Figure(data=go.Scatter())
        fig_path = go.Figure(data=go.Scatter3d())
        system_options, system_style, customer_options, customer_value, customer_style, detections_style, filter_style = init_dropdowns()
        return html.Div([
            dbc.Navbar
            ([
                dbc.Col(html.H1(children='PATS-C')),
                dbc.Col(html.H1()),
                dbc.Col(
                    dbc.Nav(dbc.NavItem(dbc.NavLink("Sign out", href="/logout", external_link=True)), navbar=True),
                    width="auto",
                ),
            ], color="#222", dark=True,),
            html.Div
            ([
                html.Div
                ([
                    html.Div('Customer:'),
                    html.Div(dcc.Dropdown(
                        id='customers_dropdown',
                        options=customer_options,
                        value=customer_value,
                        clearable=True,
                        multi=True,
                        placeholder='Select customer'
                    ), className='dash-bootstrap'),
                ], style=customer_style),
                html.Div
                ([
                    html.Div('Systems:'),
                    html.Div(dcc.Dropdown(
                        id='systems_dropdown',
                        options=system_options,
                        multi=True,
                        placeholder='Select systems'
                    ), className='dash-bootstrap'),
                ], style=system_style),
            ], style={'display': 'block', 'textAlign': 'left', 'width': '80%', 'margin': 'auto'}),
            html.Div
            ([
                html.Div
                ([
                    html.Div('Date range: '),
                    html.Div(dcc.DatePickerRange(
                        id='date_range_picker',
                        clearable=True,
                        minimum_nights=7,
                        with_portal=True,
                        display_format='DD MMM \'YY',
                        min_date_allowed=datetime.date(2020, 9, 1),
                        max_date_allowed=datetime.datetime.today(),
                        start_date=(datetime.date.today() - relativedelta(months=1)),
                        end_date=datetime.date.today(),
                        number_of_months_shown=3,
                        start_date_placeholder_text="Start period",
                        reopen_calendar_on_clear=True,
                        end_date_placeholder_text="End period",
                        persistence=True
                    ), className='dash-bootstrap')
                ], style={'width': '30%', 'display': 'inline-block'}),
                html.Div
                ([
                    html.Div('Filter:'),
                    html.Div(dcc.Dropdown(
                        id='filter_dropdown',
                        multi=False,
                        placeholder='Select filter',
                        options=[{'label': 'size', 'value': 'size'}, {'label': 'LIA', 'value': 'LIA'}],
                        value='size',
                        clearable=False,
                    ), className='dash-bootstrap'),
                ], style=filter_style),
                html.Div
                ([
                    html.Div('Insects:'),
                    html.Div(dcc.Dropdown(
                        id='detection_class_dropdown',
                        multi=False,
                        placeholder='Select insect'
                    ), className='dash-bootstrap'),
                ], style=detections_style),
            ], style={'display': 'block', 'textAlign': 'left', 'width': '80%', 'margin': 'auto'}),
            html.Br(),
            html.Div
            ([
                dcc.Loading(
                    children=html.Div([html.Br(), html.Br(), html.Br()], id='Loading_animation', style={'display': 'none'}),
                    type='default'
                ),
                dcc.Graph(id='staaf_kaart', style={'display': 'none', 'margin-left': 'auto', 'margin-right': 'auto', 'textAlign': 'center', 'width': '80%'}, figure=fig_hist)
            ]),
            html.Div
            ([
                dcc.Graph(id='staaf24h_kaart', style={'display': 'none', 'margin-left': 'auto', 'margin-right': 'auto', 'textAlign': 'center', 'width': '80%'}, figure=fig_hist24h)
            ]),
            html.Div
            ([
                dcc.Graph(id='hete_kaart', style={'display': 'none'}, figure=fig_hm)
            ]),
            html.Div
            ([
                html.Br(),
                dbc.Button("Export CSV", id="btn_csv", color='light'),
                dcc.Download(id="download_csv"),
                html.Br()
            ], style={'display': 'none'}, id="download_csv_container"),
            html.Br(),
            html.Div
            ([
                html.Br(),
                html.Div('Select x axis:', style={'display': 'table-cell'}),
                html.Div(dcc.Dropdown(
                    id='scatter_x_dropdown',
                    options=[{'label': scatter_columns[key], 'value': key} for key in scatter_columns],
                    value='duration',
                    clearable=False
                ), style={'display': 'table-cell'}, className='dash-bootstrap'),
                html.Div('Select y axis:', style={'display': 'table-cell'}),
                html.Div(dcc.Dropdown(
                    id='scatter_y_dropdown',
                    options=[{'label': scatter_columns[key], 'value': key} for key in scatter_columns],
                    value='size',
                    clearable=False
                ), style={'display': 'table-cell'}, className='dash-bootstrap'),
            ], style={'display': 'none', 'textAlign': 'center', 'width': '50%', 'margin': 'auto'}, id='scatter_dropdown_container'),
            html.Div
            ([
                dcc.Graph(id='verstrooide_kaart', style={'display': 'none'}, figure=fig_scatter)
            ]),
            html.Div
            ([
                dcc.Loading(
                    children=html.Div([html.Br(), html.Br(), html.Br()], id='Loading_animation_detection', style={'display': 'none'}),
                    type='default'
                ),
                dcc.Graph(id='route_kaart', style={'display': 'none'}, figure=fig_path),
                html.Video(id='detection_video', style={'display': 'none'}, controls=True, loop=True, autoPlay=True),
            ]),
            html.Div
            ([
                html.Div
                ([
                    'Human classification:'
                ]),
                html.Div(dcc.Dropdown(
                    id='classification_dropdown',
                    options=[{'label': c, 'value': c} for c in classification_options],
                    value=classification_options[0],
                    clearable=False,
                ), className='dash-bootstrap'),
            ], style={'display': 'none', 'textAlign': 'center', 'width': '25%', 'margin': 'auto'}, id='classify_container'),
            html.Div
            ([
                html.Br(),
                html.Ul(id='log_file_link'),
                html.Br(),
                html.Br()
            ], style={'display': 'none'}, id='log_file_link_container'),
            html.Div(id='selected_heatmap_data', style={'display': 'none'}),
            html.Div(id='selected_point_scatter', style={'display': 'none'}),
            html.Div(id='classification_hidden', style={'display': 'none'})
        ])

    app.layout = make_layout

    return app
