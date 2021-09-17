#!/usr/bin/env python3
import datetime
import math
import os
import re
from typing import List, Dict, Tuple
from dateutil.relativedelta import relativedelta
import numpy as np
import pandas as pd
from enum import Enum

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from dash.dependencies import Output, Input, State
import dash_bootstrap_components as dbc
import pats_c.lib.lib_patsc as patsc

from flask_login import current_user
from urllib.parse import quote as urlquote

customer_dict: Dict[str, List[Tuple[str, str]]] = {}
insect_columns: List[str] = []
heatmap_max = 50
classification_options = ['Not classified', 'Moth!', 'Empty/nothing to see', 'Other insect', 'Plant', 'Other false positive']
scatter_columns = {'duration': 'Duration (s)',
                   'Vel_mean': 'Velocity mean (m/s)',
                   'Vel_max': 'Velocity max (m/s)',
                   'Dist_traveled': 'Distance traveled (m)',
                   'Dist_traject': 'Distance trajectory (m)',
                   'Size': 'Size (m)',
                   'Wing_beat': 'Wing beat (Hz)'}


class Heatmap_Cell(Enum):
    selected_cell = -3
    system_down_cell = -2
    system_offline_cell = -1


def load_systems_customer(customer_name, cur):
    sql_str = '''SELECT system,location,crops.name FROM systems
                 JOIN customers ON customers.customer_id = systems.customer_id
                 JOIN crops ON crops.crop_id = customers.crop_id
                 WHERE customers.name = :customer_name
                 ORDER BY system_id''', {'customer_name': customer_name}
    systems = cur.execute(*sql_str).fetchall()
    return systems


def load_customers():
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            demo = 'demo' in username
            with patsc.open_systems_db() as con:
                sql_str = '''SELECT customers.name FROM customers
                             JOIN user_customer_connection ON user_customer_connection.customer_id=customers.customer_id
                             JOIN users ON users.user_id=user_customer_connection.user_id
                             WHERE users.name = :username
                             ORDER BY customers.name''', {'username': username}
                cur = con.execute(*sql_str)
                customers = cur.fetchall()
                customer_dict = {}
                for customer in customers:
                    customer_dict[customer[0]] = load_systems_customer(customer[0], cur)
            return customer_dict, demo
    return {}, False


def init_insects_dropdown():
    insect_options = []
    date_style = {'width': '30%', 'display': 'inline-block'}
    insect_style = {'width': '70%', 'display': 'inline-block', 'float': 'right'}
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            with patsc.open_systems_db() as con:
                sql_str = '''SELECT DISTINCT insects.name,avg_size,std_size FROM insects
                JOIN crop_insect_connection ON crop_insect_connection.insect_id = insects.insect_id
                JOIN crops ON crops.crop_id = crop_insect_connection.crop_id
                JOIN customers ON customers.crop_id = crops.crop_id
                JOIN user_customer_connection ON user_customer_connection.customer_id = customers.customer_id
                JOIN users ON users.user_id = user_customer_connection.user_id
                WHERE users.name = :username ORDER BY insects.name'''
                insects = con.execute(sql_str, (username,))
                for name, avg_size, std_size in insects:
                    insect_options.append({'label': name, 'value': {'label': name, 'avg_size': avg_size, 'std_dev': std_size}})

    insect_options.append({'label': 'No size filter', 'value': {'label': 'No size filter', 'avg_size': 0, 'std_dev': 0}})
    insect_options.append({'label': 'Anomalies', 'value': {'label': 'Anomalies', 'avg_size': 0, 'std_dev': 0}})
    return insect_options, date_style, insect_style


def init_system_and_customer_dropdown():
    customer_dict, demo = load_customers()
    sys_options = []
    customer_options = []
    customer_value = None
    customer_style = {'width': '30%', 'display': 'inline-block'}
    sys_style = {'width': '70%', 'display': 'inline-block'}

    for customer in customer_dict.keys():
        if not (customer == 'Maintance' or customer == 'Admin' or customer == 'Unassigned_systems' or customer == 'Deactivated_systems'):
            customer_options.append({'label': customer, 'value': customer})
            for i, (system, location, crop) in enumerate(customer_dict[customer]):
                if customer == 'Pats':
                    sys_options.append({'label': system, 'value': system, 'title': system})
                elif demo:
                    sys_options.append({'label': 'Demo ' + crop, 'value': system, 'title': system})
                    break  # This break makes sure that only one system per customer is added for the demo user, like Bram wanted.
                elif location:
                    if len(customer_dict.keys()) == 1:
                        sys_options.append({'label': location, 'value': system, 'title': system})
                    else:
                        sys_options.append({'label': customer + ' ' + location, 'value': system, 'title': system})
                else:
                    sys_options.append({'label': customer + ' ' + str(i + 1), 'value': system, 'title': system})
    if len(customer_dict.keys()) == 1 or demo:
        customer_value = [list(customer_dict)[0]]
        customer_style = {'width': '0%', 'display': 'none'}
        sys_style = {'width': '100%', 'display': 'inline-block'}
    return sys_options, sys_style, customer_options, customer_value, customer_style


def load_systems(username):
    with patsc.open_systems_db() as con:
        sql_str = '''SELECT DISTINCT systems.system FROM systems,user_customer_connection,users
                     WHERE  systems.customer_id = user_customer_connection.customer_id AND user_customer_connection.user_id = users.user_id AND users.name = ?
                     ORDER BY systems.system_id '''
        systems = con.execute(sql_str, (username,)).fetchall()
    authorized_systems = [d[0] for d in systems]
    return authorized_systems


def create_insect_filter_sql_str(start_date, insect_type):
    if 'Anomalies' in insect_type['label']:
        return 'Monster'
    duration_str = ' AND duration > 1 AND duration < 10'
    insect_str = ''
    close_insect_str = ''
    if start_date < datetime.datetime.strptime('20210101_000000', '%Y%m%d_%H%M%S'):  # new (size) filtering was introduced on 24th december 2020 #648
        insect_str = '((Version="1.0" AND time < "20210101_000000") OR ('
        close_insect_str = '))'
    min_size = insect_type['avg_size'] - insect_type['std_dev']
    max_size = insect_type['avg_size'] + 2 * insect_type['std_dev']
    if insect_type['avg_size'] > 0:
        insect_str = insect_str + f'''Dist_traveled > 0.15 AND Dist_traveled < 4 AND Size > {min_size} AND Size < {max_size} ''' + close_insect_str
    else:
        insect_str = insect_str + '''Dist_traveled > 0.15 AND Dist_traveled < 4 ''' + close_insect_str

    return '((' + insect_str + duration_str + ') OR Monster)'


def use_proto(start_date):
    return start_date <= datetime.datetime.strptime('20210401_000000', '%Y%m%d_%H%M%S')


def create_system_filter_sql_str(start_date, system):
    if use_proto(start_date):  # remove proto #533 closed in march 2021, so most systems probably were updated in april
        return '''(system = :system OR system = :system_proto''', {'system': system, 'system_proto': system.replace('pats', 'pats-proto')}
    else:
        return '''system = :system''', {'system': system}


def window_filter_monster(insect_df, monster_df):
    # optimisation oppurtunity:
    # monster_times = monster_df['time'].values
    # insects_with_monsters = [np.sum((monster_times > insect_time - pd.Timedelta(minutes=5)*(monster_times < insect_time + pd.Timedelta(minutes=5)) for insect_time in insect_df['time'].value]
    # insect_without_monsters = insect_df.loc[!insects_with_monsters]

    monster_id = 0
    insect_id = 0
    associated_monster_ids = []
    while insect_id < len(insect_df) and monster_id < len(monster_df):
        insect = insect_df.iloc[insect_id]
        while monster_id < len(monster_df):
            monster = monster_df.iloc[monster_id]
            monster_start = monster['time'] - datetime.timedelta(minutes=5)
            monster_end = monster['time'] + datetime.timedelta(minutes=5)
            if insect['time'] > monster_start and insect['time'] <= monster_end:
                associated_monster_ids.append(insect_id)
                insect_id += 1
                break
            elif insect['time'] > monster_end:
                monster_id += 1
            else:
                insect_id += 1
                break
    return insect_df.drop(insect_df.index[associated_monster_ids])


def load_insect_df(systems, start_date, end_date, insect_type):
    insect_df = pd.DataFrame()
    monster_df = pd.DataFrame()
    systems = installation_dates(systems)
    with patsc.open_data_db() as con:
        for (system, installation_date) in systems:
            try:
                installation_date = datetime.datetime.strptime(installation_date, '%Y%m%d_%H%M%S')
                real_start_date = max([installation_date, start_date])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_date

            time_str = f'''time > "{(real_start_date-datetime.timedelta(minutes=5)).strftime('%Y%m%d_%H%M%S')}" AND time <= "{(end_date+datetime.timedelta(minutes=5)).strftime('%Y%m%d_%H%M%S')}"'''  # with added monster window
            system_str, system_params = create_system_filter_sql_str(start_date, system)
            insect_str = create_insect_filter_sql_str(start_date, insect_type)
            sql_str = f'''SELECT moth_records.* FROM moth_records
                         WHERE {system_str}
                         AND {time_str}
                         AND {insect_str}
                         ORDER BY time'''  # nosec see #952
            insect_sys_df = pd.read_sql_query(sql_str, con, params=system_params)

            insect_sys_df['time'] = pd.to_datetime(insect_sys_df['time'], format='%Y%m%d_%H%M%S')

            monster_sys_df = insect_sys_df.loc[insect_sys_df['Monster'] == 1]
            monster_sys_df = monster_sys_df[(monster_sys_df['time'] > real_start_date) & (monster_sys_df['time'] < end_date)]  # remove the added monster window added to detect monster just outside the user selected window
            monster_df = monster_df.append(monster_sys_df)

            if 'Anomalies' not in insect_type['label']:
                insect_sys_df = insect_sys_df.loc[insect_sys_df['Monster'] != 1]
                insect_sys_df = insect_sys_df[(insect_sys_df['time'] > real_start_date) & (insect_sys_df['time'] < end_date)]  # remove the added monster window added to detect monster just outside the user selected window
                insect_sys_df = window_filter_monster(insect_sys_df, monster_sys_df)
                insect_df = insect_df.append(insect_sys_df)
            else:
                insect_df = insect_df.append(monster_sys_df)

    if use_proto(start_date):
        insect_df['system'].replace({'-proto': ''}, regex=True, inplace=True)
    return insect_df, monster_df


def load_insects_of_hour(systems, start_date, end_date, hour, insect_type):
    systems = installation_dates(systems)
    hour_str = str(hour)
    if len(hour_str) == 1:
        hour_str = '0' + hour_str
    insect_df = pd.DataFrame()
    with patsc.open_data_db() as con:
        for (system, installation_date) in systems:
            try:
                installation_date = datetime.datetime.strptime(installation_date, '%Y%m%d_%H%M%S')
                real_start_date = max([installation_date, start_date])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_date

            system_str, system_params = create_system_filter_sql_str(start_date, system)
            insect_str = create_insect_filter_sql_str(start_date, insect_type)

            sql_str = f'''SELECT moth_records.* FROM moth_records
                WHERE {system_str}
                AND time > :start_time AND time <= :end_time
                AND time LIKE "_________{hour_str}____"
                AND {insect_str}'''  # nosec see #952
            insect_df = insect_df.append(pd.read_sql_query(sql_str, con, params={**system_params, 'start_time': real_start_date.strftime('%Y%m%d_%H%M%S'), 'end_time': end_date.strftime('%Y%m%d_%H%M%S')}))
    insect_df['time'] = pd.to_datetime(insect_df['time'], format='%Y%m%d_%H%M%S')
    if use_proto(start_date):
        insect_df['system'].replace({'-proto': ''}, regex=True, inplace=True)
    return insect_df


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


def system_sql_str(systems):
    if not isinstance(systems, list):
        systems = [systems]
    systems_str = ' ('
    for system in systems:
        systems_str = systems_str + 'system="' + system + '" OR ' + 'system="' + system.replace('pats', 'pats-proto') + '" OR '
    systems_str = systems_str[:-4] + ') '
    return systems_str


def load_insect_data(selected_systems, start_date, end_date, insect_type):
    # We count insects from 12 in the afternoon to 12 in the afternoon. Therefore we shift the data in the for loops with 12 hours.
    # So a insect at 23:00 on 14-01 is counted at 14-01 and a insect at 2:00 on 15-01 also at 14-01
    # A insect seen at 13:00 on 14-01 still belongs to 14-01 but when seen at 11:00 on 14-01 is counted at 13-01
    end_date = datetime.datetime.combine(end_date, datetime.datetime.min.time())
    start_date = datetime.datetime.combine(start_date, datetime.datetime.min.time())
    insect_df, monster_df = load_insect_df(selected_systems, start_date, end_date + datetime.timedelta(hours=12), insect_type)  # Shift to include the insects of today until 12:00 in the afternoon.
    unique_dates = pd.date_range(start_date, end_date - datetime.timedelta(days=1), freq='d')

    hist_data = pd.DataFrame(index=unique_dates, columns=selected_systems)
    hist_24h_data = pd.DataFrame(index=range(0, 23), columns=selected_systems)
    for system, customer in (insect_df[['time']] - datetime.timedelta(hours=12)).groupby(insect_df.system):
        hist_data[system] = customer.groupby(customer.time.dt.date).count()
        hist_24h_data[system] = customer.groupby(customer.time.dt.hour).count()
    hist_data.fillna(0, inplace=True)
    hist_24h_data.fillna(0, inplace=True)

    heatmap_insect_df = pd.DataFrame(np.zeros((24, len(unique_dates))), index=range(24), columns=(unique_dates.strftime('%Y%m%d_%H%M%S')))  # pylint: disable=no-member
    for date, hours in (insect_df[['time']] - datetime.timedelta(hours=12)).groupby((insect_df.time - datetime.timedelta(hours=12)).dt.date):
        if date.strftime('%Y-%m-%d') in unique_dates:
            heatmap_insect_df[date.strftime('%Y%m%d_%H%M%S')] = (hours.groupby(hours.time.dt.hour).count())
    heatmap_insect_counts = ((heatmap_insect_df.fillna(0)).to_numpy()).T

    heatmap_monster_df = pd.DataFrame(np.zeros((24, len(unique_dates))), index=range(24), columns=(unique_dates.strftime('%Y%m%d_%H%M%S')))  # pylint: disable=no-member
    for date, hours in (monster_df[['time']] - datetime.timedelta(hours=12)).groupby((monster_df.time - datetime.timedelta(hours=12)).dt.date):
        if date.strftime('%Y-%m-%d') in unique_dates:
            heatmap_monster_df[date.strftime('%Y%m%d_%H%M%S')] = (hours.groupby(hours.time.dt.hour).count())
    heatmap_monster_counts = ((heatmap_monster_df.fillna(0)).to_numpy()).T

    return unique_dates, heatmap_insect_counts, heatmap_monster_counts, [], hist_data, hist_24h_data


def convert_mode_to_number(mode):
    if mode == 'monitoring':
        return 0
    elif mode == 'hunt' or mode == 'deployed':
        return -1
    elif mode == 'waypoint':
        return -2
    elif mode == 'crippled':
        return -3
    elif mode == 'wait_for_dark':
        return -4
    elif mode == 'down':
        return -5
    else:
        return -666


def insert_down_time(modes, start_date, end_date, t_start_id, t_end_id):
    modes_with_down = []
    if not len(modes):
        modes_with_down.append([start_date.strftime('%Y%m%d_%H%M%S'), end_date.strftime('%Y%m%d_%H%M%S'), 'down'])
        return modes_with_down
    prev_end_time = modes[0][t_start_id]
    for i in range(0, len(modes)):
        if datetime.datetime.strptime(modes[i][t_start_id], '%Y%m%d_%H%M%S') - datetime.timedelta(minutes=10) > datetime.datetime.strptime(prev_end_time, '%Y%m%d_%H%M%S'):
            modes_with_down.append([prev_end_time, modes[i][t_start_id], 'down'])
        modes_with_down.append(modes[i])
        prev_end_time = modes[i][t_end_id]
    return modes_with_down


def installation_dates(systems):
    with patsc.open_systems_db() as con:
        username = current_user.username
        systems = con.execute('''SELECT systems.system, systems.installation_date FROM systems,customers
        JOIN user_customer_connection ON systems.customer_id = user_customer_connection.customer_id
        JOIN users ON users.user_id = user_customer_connection.user_id WHERE
        systems.customer_id = customers.customer_id AND users.name = ? AND systems.system IN (%s)
        ORDER BY systems.system_id''' % ('?,' * len(systems))[:-1], (username, *systems)).fetchall()  # nosec see #952
        return systems


def load_mode_data(unique_dates, heatmap_insect_counts, heatmap_monster_counts, systems, start_date, end_date):
    if not len(unique_dates):
        return [], []

    modemap_data = heatmap_insect_counts.copy()
    modemap_data.fill(-666)
    sysdown_heatmap_counts = heatmap_insect_counts.copy()
    sysdown_heatmap_counts.fill(0)
    t_start_id = 0
    t_end_id = 1
    mode_id = 2
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time()) + datetime.timedelta(hours=12) - datetime.timedelta(days=(end_date - start_date).days)
    systems = installation_dates(systems)

    for (sys, installation_date) in systems:
        installation_date = datetime.datetime.strptime(installation_date, '%Y%m%d_%H%M%S')
        start_date = max([installation_date, start_date])
        start_date_str = start_date.strftime('%Y%m%d_%H%M%S')
        sql_str = f'SELECT start_datetime,end_datetime,op_mode FROM mode_records WHERE system="{sys}" AND start_datetime > :start_date ORDER BY start_datetime', {'start_date': start_date_str, 'sys': sys}  # nosec see #952
        with patsc.open_data_db() as con:
            modes = con.execute(*sql_str).fetchall()
        modes = insert_down_time(modes, start_date, end_date, t_start_id, t_end_id)

        for mode in modes:
            mode_start = datetime.datetime.strptime(mode[t_start_id], '%Y%m%d_%H%M%S') - datetime.timedelta(hours=12)
            mode_end = datetime.datetime.strptime(mode[t_end_id], '%Y%m%d_%H%M%S') - datetime.timedelta(hours=12)
            if mode_end > mode_start:  # there were some wrong logs. This check can possibly be removed at some point.
                tot_hours = math.ceil((mode_end - mode_start).total_seconds() / 3600)
            else:
                continue

            start_day = (mode_start.date() - start_date.date()).days
            start_hour = mode_start.hour

            m = convert_mode_to_number(mode[mode_id])
            for hour in range(0, tot_hours):
                nh = (start_hour + hour) % 24
                extra_days = ((start_hour + hour) - nh) / 24
                if start_day + int(extra_days) >= len(unique_dates):
                    break
                if modemap_data[start_day + int(extra_days), nh] < convert_mode_to_number('down'):  # TODO: we need to discuss this. Multi system mode mixing is ... complex
                    modemap_data[start_day + int(extra_days), nh] = m
                if m == convert_mode_to_number('down'):
                    sysdown_heatmap_counts[start_day + int(extra_days), nh] += 1

        for i in range(0, modemap_data.shape[0]):
            for j in range(0, modemap_data.shape[1]):
                if modemap_data[i, j] == convert_mode_to_number('down') and not heatmap_insect_counts[i, j] and not heatmap_monster_counts[i, j]:
                    heatmap_insect_counts[i, j] = Heatmap_Cell.system_down_cell.value
                elif modemap_data[i, j] < -1 and not heatmap_insect_counts[i, j] and not heatmap_monster_counts[i, j]:
                    heatmap_insect_counts[i, j] = Heatmap_Cell.system_offline_cell.value
    return heatmap_insect_counts, sysdown_heatmap_counts


def natural_sort_systems(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(line, key=alphanum_key)


def create_heatmap(unique_dates, insect_counts, monster_counts, sysdown_counts, xlabels, selected_heat):
    insect_count_hover_label = pd.DataFrame(insect_counts).astype(int).astype(str)
    insect_count_hover_label[insect_count_hover_label == str(Heatmap_Cell.system_down_cell.value)] = 'NA'
    insect_count_hover_label[insect_count_hover_label == str(Heatmap_Cell.system_offline_cell.value)] = 'NA'
    monster_count_hover_label = pd.DataFrame(monster_counts).astype(int).astype(str)
    sysdown_count_hover_label = pd.DataFrame(sysdown_counts).astype(int).astype(str)
    heatmap_data = np.clip(insect_counts, Heatmap_Cell.system_down_cell.value, heatmap_max)

    if selected_heat:
        selected_heat = pd.read_json(selected_heat, orient='split')
        for _, cel in selected_heat.iterrows():
            x = cel['x']
            y = (unique_dates.strftime('%d-%m-%Y').tolist()).index(cel['lalaladate'])
            if heatmap_data[y, x] >= 0:
                heatmap_data[y, x] = Heatmap_Cell.selected_cell.value

    hm = go.Heatmap(
        x=xlabels,
        y=unique_dates.strftime('%d-%m-%Y'),
        z=heatmap_data,
        customdata=np.stack((insect_count_hover_label, monster_count_hover_label, sysdown_count_hover_label), axis=-1),
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
        'Date: %{y}<br>' +
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


def create_hist(df_hist, unique_dates, system_labels):
    fig = go.Figure()
    fig.update_yaxes(rangemode="nonnegative")
    cnt = 0
    bar_totals = df_hist.sum(axis=1).astype(int)
    for sys in system_labels.keys():
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
        cnt += 1
        fig.add_trace(hist)

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
    cnt = 0
    bar_totals = hist_24h_data.sum(axis=1).astype(int)

    for sys in system_labels.keys():
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
        cnt += 1
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


def create_scatter(insects, system_labels, scatter_x_value, scatter_y_value):
    df_scatter = insects
    df_scatter['system_ids'] = df_scatter['system'].str.replace('pats-proto', '').str.replace('pats', '').astype(int)
    df_scatter['system_color'] = df_scatter['system'].apply(list(system_labels.keys()).index).astype(int)
    df_scatter['classification_symbol'] = df_scatter['Human_classification'].apply(classification_to_symbol)
    df_scatter['Human_classification'] = df_scatter['Human_classification'].apply(remove_nones_classification)
    df_scatter['video_symbol'] = df_scatter['Video_Filename'].apply(video_available_to_symbol)
    df_scatter['symbol'] = df_scatter['video_symbol'] + df_scatter['classification_symbol']

    scat_fig = go.Figure()
    scat_fig.update_yaxes({'range': (df_scatter[scatter_y_value].min() - 0.005, df_scatter[scatter_y_value].max() + 0.005)})
    for sys in system_labels.keys():
        df_scatter_sys = df_scatter[df_scatter['system'] == sys]
        for classification in classification_options:
            tmp1 = list(df_scatter_sys['Human_classification'] == classification)  # which rows have the same classification
            df = df_scatter_sys[tmp1]  # retrieve only those rows that have the same classficication
            df = df.fillna('')
            if not df.empty:
                # df['time'].apply(lambda x: x.strftime('%H:%M:%S %d-%m-%Y'))
                df['time_for_humans'] = df['time'].dt.strftime('%H:%M:%S %d-%m-%Y')
                current_labels = [system_labels[sys] for x in df[scatter_x_value]]
                scatter = go.Scattergl(
                    name=system_labels[sys] + ', ' + classification,
                    showlegend=True,
                    x=df[scatter_x_value],
                    y=df[scatter_y_value],
                    mode='markers',
                    customdata=np.stack(
                        (df['system'] + '/' + df['Folder'] + '/' + df['Filename'],
                         df['Video_Filename'],
                         df['Human_classification'],
                         current_labels,
                         df['uid'],
                         df['time_for_humans'],
                         ), axis=-1),
                    marker=dict(
                        cmin=0,
                        cmax=9,
                        color=df['system_color'],
                        colorscale=px.colors.qualitative.Vivid,
                        symbol=df_scatter['symbol'],
                        size=10,
                        line=dict(width=2, color="rgba(0,0, 0, 255)")
                    ),
                    hovertemplate='<b>System %{customdata[3]}</b><br><br>' +
                    'x: %{x}<br>' +
                    'y: %{y}<br>' +
                    't: %{customdata[5]}<br>' +
                    'File: %{customdata[0]}<br>' +
                    'Video: %{customdata[1]}<br>' +
                    'Ground truth: %{customdata[2]}<br>' +
                    '<extra></extra>'
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


def download_log(selected_insect, insect_columns):
    sys_name = selected_insect[insect_columns.index('system')].replace('-proto', '')
    log_fn = selected_insect[insect_columns.index('Filename')]
    log_folder = selected_insect[insect_columns.index('Folder')]
    if not log_folder or log_fn == 'unknown':
        return ''
    target_log_fn = './static/' + log_folder + '_' + sys_name + '_' + log_fn
    if not os.path.isfile(target_log_fn):
        rsync_src = sys_name + ':pats/data/processed/' + log_folder + '/logging/' + log_fn
        cmd = ['rsync --timeout=5 -az -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_log_fn]
        patsc.execute(cmd)
    return target_log_fn


def file_download_link(filename):
    location = '/{}'.format(urlquote(filename))
    return html.A('Download log', href=location, style={'display': 'block'}), {'textAlign': 'center', 'width': '25%', 'margin': 'auto', 'display': 'block'}


def create_path_plot(target_log_fn, selected_insect, insect_columns):
    df_ilog = pd.read_csv(target_log_fn, delimiter=';')
    target_log_fn = '/' + target_log_fn
    rows_without_tracking = df_ilog[df_ilog['n_frames_tracking_insect'] == 0].index
    df_ilog = df_ilog.drop(rows_without_tracking)
    scatter = go.Scatter3d(
        x=-df_ilog['sposX_insect'],
        y=-df_ilog['sposZ_insect'],
        z=df_ilog['sposY_insect'],
        text=df_ilog['time'],
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
    title_str = 'Insect flight path of detecton at: ' + datetime.datetime.strptime(str(selected_insect[insect_columns.index('time')]), '%Y%m%d_%H%M%S').strftime('%d-%m-%Y %H:%M:%S')
    fig.update_layout(
        title_text=title_str,
        scene=dict(
            aspectmode='data'
        )
    )
    style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
    return fig, style


def download_video(selected_insect, insect_columns):
    video_fn = selected_insect[insect_columns.index('Video_Filename')]
    sys_name = selected_insect[insect_columns.index('system')].replace('-proto', '')

    target_video_mp4_fn = ''
    if video_fn and not video_fn.startswith('NA'):
        target_video_mkv_fn = 'static/' + selected_insect[insect_columns.index('Folder')] + '_' + sys_name + '_' + video_fn
        target_video_mp4_fn = target_video_mkv_fn[0:-3] + 'mp4'

        if not os.path.isfile(target_video_mkv_fn) and not os.path.isfile(target_video_mp4_fn):
            rsync_src = sys_name + ':pats/data/processed/' + selected_insect[insect_columns.index('Folder')] + '/logging/render_' + video_fn
            cmd = ['rsync --timeout=5 -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + target_video_mkv_fn]
            patsc.execute(cmd)
        if not os.path.isfile(target_video_mp4_fn) and os.path.isfile(target_video_mkv_fn):
            mp4_to_mkv_cmd = ['ffmpeg -y -i ' + target_video_mkv_fn + ' -c:v copy -an ' + target_video_mkv_fn[0:-3] + 'mp4']
            patsc.execute(mp4_to_mkv_cmd)
        if not os.path.isfile(target_video_mp4_fn):
            return ''
        target_video_mp4_fn = '/' + target_video_mp4_fn

    return target_video_mp4_fn


def dash_application():
    print("Starting PATS-C dash application!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/pats-c/', title='PATS-C')

    pio.templates.default = 'plotly_dark'

    @ app.callback(
        Output('systems_dropdown', 'value'),
        Input('customers_dropdown', 'value'))
    def select_system_customer(selected_customer):  # pylint: disable=unused-variable
        customer_dict, _ = load_customers()
        value = []
        if selected_customer:
            for customer in selected_customer:
                systems = [d[0] for d in customer_dict[customer]]
                value.extend(systems)
        return value

    def update_selected_heat(clickData_hm, selected_heat):
        if not clickData_hm:
            return None
        else:
            for cel in clickData_hm['points']:
                if cel['z'] == Heatmap_Cell.system_offline_cell.value or cel['z'] == Heatmap_Cell.system_down_cell.value:
                    return None  # if a black cell is clicked, interpret that as cancelling the whole selection

                x = hour_labels.index(cel['x'])

                unclicked = False
                columns = ['x', 'lalaladate', 'hour']  # lalaladate -> python bug prevents me from using 'date' https://stackoverflow.com/questions/48369578/prevent-pandas-to-json-from-adding-time-component-to-date-object
                if not selected_heat:
                    selected_heat = pd.DataFrame(columns=columns)
                else:
                    selected_heat = pd.read_json(selected_heat, orient='split')
                for index, _ in selected_heat.iterrows():
                    if cel['z'] == Heatmap_Cell.selected_cell.value:
                        selected_heat = selected_heat.drop(index=index)
                        unclicked = True
                        break

                if not unclicked:
                    df_row = pd.DataFrame([[x, cel['y'], int(hour_labels[x].replace('h', ''))]], columns=columns)
                    if selected_heat.empty:
                        selected_heat = df_row
                    else:
                        selected_heat = selected_heat.append(df_row, ignore_index=True)

            selected_heat = selected_heat.to_json(date_format='iso', orient='split')
        return selected_heat

    @ app.callback(
        Output('hete_kaart', 'figure'),
        Output('staaf_kaart', 'figure'),
        Output('staaf24h_kaart', 'figure'),
        Output('hete_kaart', 'style'),
        Output('staaf_kaart', 'style'),
        Output('staaf24h_kaart', 'style'),
        Output('selected_heatmap_data', 'children'),
        Output('Loading_animation', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('insects_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown', 'options'))
    def update_ui_hist_and_heat(start_date, end_date, selected_systems, insect_types, clickData_hm, selected_heat, system_options):  # pylint: disable=unused-variable
        hm_fig = go.Figure(data=go.Heatmap())
        hist_fig = go.Figure(data=go.Histogram())
        hist24h_fig = go.Figure(data=go.Histogram())
        hm_style = {'display': 'none'}
        hist_style = {'display': 'none'}
        hist24h_style = {'display': 'none'}
        loading_animation_style = {'display': 'block'}
        system_labels = {x['value']: x['label'] for x in system_options if x['value'] in selected_systems}

        prop_id = dash.callback_context.triggered[0]['prop_id']
        if prop_id == 'date_range_picker.value' or prop_id == 'systems_dropdown.value' or prop_id == 'insects_dropdown.value':
            selected_heat = None
            clickData_hm = None
        selected_systems = remove_unauthoirized_system(selected_systems)
        if start_date and end_date:
            end_date = datetime.datetime.strptime(end_date, '%Y-%m-%d')
            start_date = datetime.datetime.strptime(start_date, '%Y-%m-%d')
        if not selected_systems or not insect_types or not end_date or not start_date or (end_date - start_date).days < 1:
            return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style
        unique_dates, heatmap_insect_counts, heatmap_monster_counts, _, df_hist, hist_24h_data = load_insect_data(selected_systems, start_date, end_date, insect_types)
        if not len(unique_dates):
            return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style

        hist_fig, hist_style = create_hist(df_hist, unique_dates, system_labels)
        hist24h_fig, hist24h_style = create_24h_hist(hist_24h_data, hour_labels, system_labels)

        if prop_id == 'hete_kaart.clickData' or prop_id == 'classification_dropdown.value':
            selected_heat = update_selected_heat(clickData_hm, selected_heat)

        heatmap_insect_counts, sysdown_heatmap_counts = load_mode_data(unique_dates, heatmap_insect_counts, heatmap_monster_counts, selected_systems, start_date, end_date)  # add mode info to heatmap, which makes the heatmap show when the system was online (color) or offline (black)
        hm_fig, hm_style = create_heatmap(unique_dates, heatmap_insect_counts, heatmap_monster_counts, sysdown_heatmap_counts, hour_labels, selected_heat)

        loading_animation_style = {'display': 'none'}

        return hm_fig, hist_fig, hist24h_fig, hm_style, hist_style, hist24h_style, selected_heat, loading_animation_style

    @ app.callback(
        Output('verstrooide_kaart', 'figure'),
        Output('verstrooide_kaart', 'style'),
        Output('scatter_dropdown_container', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('insects_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'selectedData'),
        Input('staaf24h_kaart', 'selectedData'),
        Input('scatter_x_dropdown', 'value'),
        Input('scatter_y_dropdown', 'value'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown', 'options'))
    def update_ui_scatter(start_date, end_date, selected_systems, insect_types, clickData_hm, hist_selected_bars, hist24h_selected_bars, scatter_x_value, scatter_y_value, selected_heat, system_options):  # pylint: disable=unused-variable
        scat_fig = go.Figure(data=go.Scatter())
        scat_style = {'display': 'none'}
        scat_axis_select_style = {'display': 'none'}
        if system_options and selected_systems:
            system_labels = {x['value']: x['label'] for x in system_options if x['value'] in selected_systems}
        else:
            system_labels = None

        selected_systems = remove_unauthoirized_system(selected_systems)
        if not selected_systems or not insect_types:
            return scat_fig, scat_style, scat_axis_select_style

        prop_id = dash.callback_context.triggered[0]['prop_id']
        if prop_id == 'staaf_kaart.selectedData' or prop_id == 'staaf24h_kaart.selectedData' or prop_id == 'hete_kaart.clickData' or prop_id == 'scatter_x_dropdown.value' or prop_id == 'scatter_y_dropdown.value':
            if prop_id == 'hete_kaart.clickData':
                selected_heat = update_selected_heat(clickData_hm, selected_heat)
            elif prop_id == 'staaf_kaart.selectedData' or prop_id == 'staaf24h_kaart.selectedData':
                selected_heat = None
            insects = pd.DataFrame()
            if selected_heat:
                selected_heat = pd.read_json(selected_heat, orient='split')
                for _, hm_cell in selected_heat.iterrows():
                    hour = hm_cell['hour']
                    if hour < 12:
                        hour += 24
                    start_date = datetime.datetime.strptime(hm_cell['lalaladate'], '%d-%m-%Y') + datetime.timedelta(hours=hour)
                    insects_hour, _ = load_insect_df(selected_systems, start_date, start_date + datetime.timedelta(hours=1), insect_types)
                    insects = insects.append(insects_hour)
            elif hist_selected_bars:
                for bar in hist_selected_bars['points']:
                    sys = bar['customdata'][1]
                    start_date = datetime.datetime.strptime(bar['x'], '%d-%m-%Y') + datetime.timedelta(hours=12)
                    insects_day, _ = load_insect_df([sys], start_date, start_date + datetime.timedelta(days=1), insect_types)
                    insects = insects.append(insects_day)
            elif hist24h_selected_bars:
                start_date = datetime.datetime.strptime(start_date, '%Y-%m-%d')
                end_date = datetime.datetime.strptime(end_date, '%Y-%m-%d')
                for bar in hist24h_selected_bars['points']:
                    sys = bar['customdata'][1]
                    hour = int(bar['x'].replace('h', ''))
                    insects_hour, _ = load_insects_of_hour([sys], start_date, end_date, hour, insect_types)
                    insects = insects.append(insects_hour)

            if not insects.empty:
                scat_fig = create_scatter(insects, system_labels, scatter_x_value, scatter_y_value)
                scat_axis_select_style = {'display': 'table', 'textAlign': 'center', 'width': '50%', 'margin': 'auto'}
                scat_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
        return scat_fig, scat_style, scat_axis_select_style

    @ app.callback(
        Output('insect_video', 'src'),
        Output('insect_video', 'style'),
        Output('route_kaart', 'figure'),
        Output('route_kaart', 'style'),
        Output('log_file_link', 'children'),
        Output('log_file_link_container', 'style'),
        Output('selected_scatter_insect', 'children'),
        Output('classification_dropdown', 'value'),
        Output('classify_container', 'style'),
        Output('Loading_animation_insect', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('insects_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'clickData'),
        Input('staaf24h_kaart', 'clickData'),
        Input('verstrooide_kaart', 'clickData'),
        State('verstrooide_kaart', 'figure'))
    def update_insect_ui(start_date, end_date, selected_systems, selected_insects, clickData_hm, clickData_hist, clickData_hist24h, clickData_dot, scatter_fig_state):  # pylint: disable=unused-variable
        target_video_fn = ''
        video_style = {'display': 'none'}
        path_fig = go.Figure(data=go.Scatter3d())
        path_style = {'display': 'none'}
        file_link = html.A('File not available.', style={'display': 'none'})
        file_link_style = {'textAlign': 'center', 'width': '25%', 'margin': 'auto', 'display': 'none'}
        selected_insect = []
        classification = classification_options[0]
        classify_style = {'display': 'none'}
        loading_animation_style = {'display': 'block'}

        selected_systems = remove_unauthoirized_system(selected_systems)
        prop_id = dash.callback_context.triggered[0]['prop_id']
        if 'selectedpoints' not in scatter_fig_state['data'][0] or prop_id != 'verstrooide_kaart.clickData' or not selected_systems or not selected_insects:
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_insect, classification, classify_style, loading_animation_style

        sql_str = 'SELECT * FROM moth_records WHERE uid=:uid'
        sql_params = {'uid': str(clickData_dot['points'][0]['customdata'][4])}
        with patsc.open_data_db() as con:
            entry = con.execute(sql_str, sql_params).fetchall()
        if not len(entry):
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_insect, classification, classify_style, loading_animation_style
        selected_insect = entry[0]

        target_log_fn = download_log(selected_insect, insect_columns)
        if not os.path.isfile(target_log_fn):
            return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_insect, classification, classify_style, loading_animation_style
        file_link, file_link_style = file_download_link(target_log_fn)

        if 'Human_classification' in insect_columns:
            classification = selected_insect[insect_columns.index('Human_classification')]
        if not classification:
            classification = classification_options[0]
        classify_style = {'display': 'block', 'textAlign': 'center', 'width': '25%', 'margin': 'auto'}

        path_fig, path_style = create_path_plot(target_log_fn, selected_insect, insect_columns)

        target_video_fn = download_video(selected_insect, insect_columns)
        loading_animation_style = {'display': 'none'}
        if target_video_fn != '':
            video_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}

        return target_video_fn, video_style, path_fig, path_style, file_link, file_link_style, selected_insect, classification, classify_style, loading_animation_style

    @ app.callback(
        Output(component_id='classification_hidden', component_property='children'),
        Input('classification_dropdown', 'value'),
        Input('selected_scatter_insect', 'children'))
    def classification_value(selected_classification, selected_insect):  # pylint: disable=unused-variable
        prop_id = dash.callback_context.triggered[0]['prop_id']
        if not selected_insect or prop_id != 'classification_dropdown.value':
            return []

        current_classification = selected_insect[insect_columns.index('Human_classification')]
        if not current_classification:
            current_classification = classification_options[0]
        if current_classification != selected_classification:
            sql_str = 'UPDATE moth_records SET Human_classification=:selected_classification WHERE uid=:uid', {'selected_classification': selected_classification, 'uid': str(selected_insect[insect_columns.index('uid')])}
            with patsc.open_data_db() as con:
                con.execute(*sql_str)

            # also save user classifications to a seperate database, because the insect database sometimes needs to be rebuild from the logs/jsons
            with patsc.open_classification_db() as con_class:
                table_exists = con_class.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='classification_records' ''').fetchone()[0]
                if not table_exists:
                    sql_create = 'CREATE TABLE classification_records(uid INTEGER PRIMARY KEY,moth_uid INTEGER,system TEXT,time TEXT,duration REAL,user TEXT,classification TEXT)'
                    con_class.execute(sql_create)
                    con_class.commit()

                username = current_user.username
                sql_insert = 'INSERT INTO classification_records(moth_uid,system,time,duration,user,classification) VALUES(?,?,?,?,?,?)'
                data = [selected_insect[insect_columns.index('uid')], str(selected_insect[insect_columns.index('system')]), str(selected_insect[insect_columns.index('time')]), str(selected_insect[insect_columns.index('duration')]), username, selected_classification]
                con_class.execute(sql_insert, data)
                con_class.commit()

        return selected_classification

    if not os.path.exists(os.path.expanduser('~/patsc/static/')):
        os.makedirs(os.path.expanduser('~/patsc/static/'))

    with patsc.open_data_db() as con:
        insect_columns = [i[1] for i in con.execute('PRAGMA table_info(moth_records)')]
        if 'Human_classification' not in insect_columns:
            con.execute('ALTER TABLE moth_records ADD COLUMN Human_classification TEXT')

            if os.path.exists(patsc.db_classification_path):
                with patsc.open_classification_db() as con_class:
                    classification_columns = [i[1] for i in con_class.execute('PRAGMA table_info(classification_records)')]
                    sql_str = 'SELECT * FROM classification_records'
                    classification_data = con_class.execute(sql_str).fetchall()

                for entry in classification_data:
                    sql_where = ' WHERE uid=' + str(entry[classification_columns.index('moth_uid')]) + ' AND time="' + str(entry[classification_columns.index('time')]) + '" AND duration=' + str(entry[classification_columns.index('duration')])
                    sql_str = 'UPDATE moth_records SET Human_classification="' + entry[classification_columns.index('classification')] + '"' + sql_where
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
        system_options, system_style, customer_options, customer_value, customer_style = init_system_and_customer_dropdown()
        insect_options, date_style, insect_style = init_insects_dropdown()
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
                ], style=date_style),
                html.Div
                ([
                    html.Div('Insects:'),
                    html.Div(dcc.Dropdown(
                        id='insects_dropdown',
                        options=insect_options,
                        multi=False,
                        value=insect_options[0]['value'] if insect_options else "",
                        placeholder='Select insects'
                    ), className='dash-bootstrap'),
                ], style=insect_style),
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
                    value='Size',
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
                    children=html.Div([html.Br(), html.Br(), html.Br()], id='Loading_animation_insect', style={'display': 'none'}),
                    type='default'
                ),
                dcc.Graph(id='route_kaart', style={'display': 'none'}, figure=fig_path),
                html.Video(id='insect_video', style={'display': 'none'}, controls=True, loop=True, autoPlay=True),
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
            html.Div(id='selected_scatter_insect', style={'display': 'none'}),
            html.Div(id='classification_hidden', style={'display': 'none'})
        ])

    app.layout = make_layout

    return app
