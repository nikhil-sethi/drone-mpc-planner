#!/usr/bin/env python3
import socket, json,shutil
import time, argparse
import pickle, glob, os, re
import numpy as np
import datetime
import sqlite3
from tqdm import tqdm
import math

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd
import plotly.graph_objects as go
import numpy as np
import flask

systems = None
selected_systems = ''
selected_dayrange = 7
unique_dates = []
heatmap_data = []
modemap_data = []
xlabels=[]
db_path = ''

def open_db():
    global db_path
    conn = None
    cur = None
    try:
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()
    except Exception as e:
        print(e)
    return conn,cur

def load_systems():
    global systems,selected_systems
    conn,cur = open_db()
    sql_str = '''SELECT DISTINCT system from mode_records ORDER BY system '''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = natural_sort_systems(systems)
    systems = [d[0] for d in systems]
    selected_systems = [systems[0]]

def system_sql_str(systems):
    if not isinstance(systems, list):
        systems = [systems]
    systems_str = ''
    for system in systems:
        systems_str = systems_str + 'system="' + system + '" OR '
    systems_str = systems_str[:-3]
    return systems_str

def load_moth_data():
    global unique_dates,heatmap_data,selected_systems,selected_dayrange
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()
    sql_str = 'SELECT MIN(time) from moth_records where ' + systems_str
    if selected_dayrange > 0:
        start_day = (datetime.datetime.now() - datetime.timedelta(days=selected_dayrange)).strftime("%Y%m%d_%H%M%S")
        sql_str += 'and time > "' + start_day + '"'
    cur.execute(sql_str)
    first_date = cur.fetchone()[0]
    sql_str = 'SELECT MAX(time) from moth_records where ' + systems_str
    if selected_dayrange > 0:
        sql_str += 'and time > "' + start_day + '"'
    cur.execute(sql_str)
    end_date = cur.fetchone()[0]
    first_date = datetime.datetime.strptime(first_date, "%Y%m%d_%H%M%S")
    end_date = datetime.datetime.strptime(end_date, "%Y%m%d_%H%M%S")
    d = first_date
    unique_dates = []
    while d <= end_date:
        unique_dates.append(d.strftime("%d-%m-%Y"))
        d += datetime.timedelta(days=1)

    heatmap_data = np.zeros((len(unique_dates),24))
    sql_str = 'SELECT time from moth_records where ' + systems_str
    if selected_dayrange > 0:
        sql_str += 'and time > "' + start_day + '"'
    cur.execute(sql_str)
    moths = cur.fetchall()
    for moth in moths:
        d = datetime.datetime.strptime(moth[0], "%Y%m%d_%H%M%S")
        day = (datetime.datetime(d.year, d.month, d.day) - first_date).days
        day = (d - first_date).days
        hour = d.hour
        heatmap_data[day,(hour+12)%24] += 1

def convert_mode_to_number(mode):
    if  mode == 'monitoring':
        return 0
    elif  mode == 'hunt' or mode == 'deployed':
        return -1
    elif  mode == 'waypoint':
        return -2
    elif  mode == 'crippled':
        return -3
    elif mode == 'wait_for_dark':
        return -4
    else:
        return -666

def load_mode_data():
    global unique_dates,heatmap_data,modemap_data,selected_systems,selected_dayrange
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()
    sql_str = 'SELECT op_mode,start_datetime,end_datetime from mode_records where ' + systems_str
    if selected_dayrange > 0:
        start_day = (datetime.datetime.now() - datetime.timedelta(days=selected_dayrange)).strftime("%Y%m%d_%H%M%S")
        sql_str += 'and start_datetime > "' + start_day + '"'
    sql_str += ' ORDER BY "start_datetime"'
    cur.execute(sql_str)
    modes = cur.fetchall()
    first_date = datetime.datetime.strptime(unique_dates[0],"%d-%m-%Y")

    modemap_data = heatmap_data.copy()
    modemap_data.fill(-666)
    for mode in modes:
        d_mode_start = datetime.datetime.strptime(mode[1], "%Y%m%d_%H%M%S")
        d_mode_end = datetime.datetime.strptime(mode[2], "%Y%m%d_%H%M%S")
        len_hours = math.ceil((d_mode_end-d_mode_start).seconds/3600)
        day = (datetime.datetime(d_mode_start.year, d_mode_start.month, d_mode_start.day) - first_date).days
        hour = d_mode_start.hour
        m = convert_mode_to_number(mode[0])
        for h in range(0,len_hours):
            nh = hour+h
            nh = nh % 24
            extra_days=((hour+h) - nh)/24
            if day+int(extra_days) >= len(unique_dates):
                break
            modemap_data[day+int(extra_days),nh] = m

    for i in range(0,modemap_data.shape[0]):
        for j in range(0,modemap_data.shape[1]):
            if modemap_data[i,j] < -1:
                heatmap_data[i,(j+12)%24] = -1

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def create_heatmap():
    global unique_dates,heatmap_data,xlabels
    fig = go.Figure(data=go.Heatmap(
            x=xlabels,
            y=unique_dates,
            z = heatmap_data,
            type = 'heatmap',
            zmin=-1,
            zmax=19,

            colorscale = [
                        [0, 'rgba(0, 0, 0, 0.85)'],
                        [1/20, 'rgba(6,154,121, 0.85)'],
                        [20/20, 'rgba(255,0,0, 0.85)']
                        ],

            colorbar = dict(lenmode='pixels', len=400,yanchor='top',y=1)
            ))
    fig['layout']['xaxis']['side'] = 'top'
    fig['layout']['xaxis']['tickangle'] = 45
    h = len(unique_dates) * 30
    if h < 400:
        h=400
    fig.update_layout(
        height=h,

    )
    return fig


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dash app that shows Pats monitoring and system analysis')
    parser.add_argument('--db', help="Path to the sql database", default='~/pats.db')
    args = parser.parse_args()
    db_path = os.path.expanduser(args.db)
else:
    db_path = os.path.expanduser('~/pats.db')

load_systems()
load_moth_data()
load_mode_data()

for i in range(0,24):
    xlabels.append(str((i+12)%24)+'h')

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
server = flask.Flask(__name__)
app = dash.Dash(__name__, external_stylesheets=external_stylesheets, server=server)

fig=create_heatmap()
dateranges = ['Last week', 'Last month', 'Last year', 'All']

app.layout = html.Div(children=[
    html.H1(children='Pats Dash'),
    html.Div([
            html.Div('Select systems:'),
            dcc.Dropdown(
                id='systems-dropdown',
                options=[{"label": system, "value": system} for system in systems],
                value=systems[0],
                multi=True
            ),
        ], style={'width': '49%', 'display': 'inline-block'}),
    html.Div([
            html.Div('Select range:'),
            dcc.Dropdown(
                id='date_range-dropdown',
                options=[{"label": daterange, "value": daterange} for daterange in dateranges],
                value=dateranges[0],
                searchable=False,
                clearable=False
            ),
        ], style={'width': '49%', 'display': 'inline-block'}),
    html.Div([
            dcc.Graph(id='hete-kaart',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig)
        ])
])

@app.callback(
    dash.dependencies.Output('hete-kaart', 'figure'),
    [dash.dependencies.Input('date_range-dropdown', 'value'),
    dash.dependencies.Input('systems-dropdown', 'value')]
)
def update_output(daterange_value,system_value):
    global selected_systems,selected_dayrange
    if daterange_value == 'Last week':
        selected_dayrange = 7
    elif daterange_value == 'Last month':
        selected_dayrange = 31
    elif daterange_value == 'Last year':
        selected_dayrange = 365
    elif daterange_value == 'All':
        selected_dayrange = 0

    selected_systems = system_value

    load_moth_data()
    load_mode_data()
    fig=create_heatmap()
    return fig

if __name__ == '__main__':
    app.run_server()