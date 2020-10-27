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
hist_data = []
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
    sql_str = '''SELECT DISTINCT system FROM mode_records ORDER BY system '''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = natural_sort_systems(systems)
    systems = [d[0] for d in systems]
    selected_systems = [systems[0]]

def system_sql_str(systems):
    if not isinstance(systems, list):
        systems = [systems]
    systems_str = ' ('
    for system in systems:
        systems_str = systems_str + 'system="' + system + '" OR '
    systems_str = systems_str[:-4] + ') '
    return systems_str

def load_moth_data():
    global unique_dates,heatmap_data,hist_data,selected_systems,selected_dayrange
    if not len(selected_systems):
        heatmap_data = []
        unique_dates = []
        return
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()

    sql_str = 'SELECT MIN(time) FROM moth_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=selected_dayrange)
    start_date_str = start_date.strftime("%Y%m%d_%H%M%S")
    sql_str += 'AND time>"' + start_date_str + '"'
    cur.execute(sql_str)
    first_date = cur.fetchone()[0]

    sql_str = 'SELECT MAX(time) FROM moth_records WHERE ' + systems_str
    sql_str += 'AND time > "' + start_date_str + '"'
    cur.execute(sql_str)
    end_date = cur.fetchone()[0]
    if first_date == None or end_date == None:
        heatmap_data = []
        unique_dates = []
        return

    end_date = datetime.datetime.strptime(end_date, "%Y%m%d_%H%M%S")
    d = start_date
    unique_dates = []
    while d <= end_date:
        unique_dates.append(d.strftime("%d-%m-%Y"))
        d += datetime.timedelta(days=1)

    heatmap_data = np.zeros((len(unique_dates),24))
    hist_data = np.zeros((len(unique_dates)))
    sql_str = 'SELECT time FROM moth_records WHERE ((duration > 1 AND duration < 10 AND Dist_traveled > 0.15 AND Dist_traveled < 4) OR (Version="1.0" AND duration > 1 AND duration < 10)) AND '
    sql_str += systems_str
    if selected_dayrange > 0:
        sql_str += 'AND time > "' + start_date_str + '"'
    cur.execute(sql_str)
    moths = cur.fetchall()
    cnt = 0
    for moth in moths:
        d = datetime.datetime.strptime(moth[0], "%Y%m%d_%H%M%S")-datetime.timedelta(hours=12)
        day = (d.date() - start_date.date()).days
        hour = d.hour
        heatmap_data[day,hour] += 1
        hist_data[day] +=1

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
    if not len(selected_systems):
        modemap_data = []
        return
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()

    sql_str = 'SELECT op_mode,start_datetime,end_datetime FROM mode_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=selected_dayrange)
    start_date_str = start_date.strftime("%Y%m%d_%H%M%S")
    sql_str += 'AND start_datetime > "' + start_date_str + '"'
    sql_str += ' ORDER BY "start_datetime"'
    cur.execute(sql_str)
    modes = cur.fetchall()
    if not len(unique_dates):
        modemap_data = []
        return
    first_date = datetime.datetime.strptime(unique_dates[0],"%d-%m-%Y")

    modemap_data = heatmap_data.copy()
    modemap_data.fill(-666)
    for mode in modes:
        d_mode_start = datetime.datetime.strptime(mode[1], "%Y%m%d_%H%M%S")-datetime.timedelta(hours=12)
        d_mode_end = datetime.datetime.strptime(mode[2], "%Y%m%d_%H%M%S")-datetime.timedelta(hours=12)
        len_hours = math.ceil((d_mode_end-d_mode_start).seconds/3600)

        day = (d_mode_start.date() - start_date.date()).days
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
            if modemap_data[i,j] < -1 and heatmap_data[i,j] == 0:
                heatmap_data[i,j] = -1

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)



def create_heatmap():
    global unique_dates,heatmap_data,xlabels
    hm = go.Heatmap(
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

            colorbar = dict(lenmode='pixels', len=400,yanchor='top',y=1),

            hovertemplate='Time: %{x}<br>Count: %{z}<extra></extra>'

            )
    fig = go.Figure(data=hm)
    fig['layout']['xaxis']['side'] = 'top'
    fig['layout']['xaxis']['tickangle'] = 45
    h = len(unique_dates) * 30
    if h < 400:
        h=400
    if not len(unique_dates):
        h=10
    fig.update_layout(
        height=h,
        clickmode='event+select'
    )

    return fig

def create_hist():
    global selected_systems,selected_dayrange,hist_data,unique_dates

    hist = go.Bar(
        x = unique_dates,
        y = hist_data,
        marker=dict(
            color='green'
        )
    )

    fig = go.Figure(data=hist)

    fig.update_layout(
        title_text='Moth counts',
        xaxis_title_text='Days [s]',
        yaxis_title_text='Counts',
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

fig_hm=create_heatmap()
fig_hist=create_hist()
dateranges = ['Last week', 'Last month', 'Last year']
video_fn = '/static/render_moth34.mp4'

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
            dcc.Graph(id='mot-hist',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig_hm)
        ]),
    html.Div([
            dcc.Graph(id='hete-kaart',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig_hist)
        ]),
    # html.Div([
    #     html.Video(id='insect_video',src=video_fn,controls=True)
    #     ])
])

@app.callback(
    dash.dependencies.Output('hete-kaart', 'figure'),
    dash.dependencies.Output('mot-hist', 'figure'),
    dash.dependencies.Input('date_range-dropdown', 'value'),
    dash.dependencies.Input('systems-dropdown', 'value')
)
def update_output(daterange_value,system_value):
    global selected_systems,selected_dayrange
    if daterange_value == 'Last week':
        selected_dayrange = 7
    elif daterange_value == 'Last month':
        selected_dayrange = 31
    elif daterange_value == 'Last year':
        selected_dayrange = 365

    selected_systems = system_value

    load_moth_data()
    load_mode_data()
    fig_hm=create_heatmap()
    fig_hist=create_hist()
    return fig_hm,fig_hist


# @app.callback(
#     dash.dependencies.Output('insect_video', 'src'),
#     dash.dependencies.Input('hete-kaart', 'on_click')
# )
# def heatmap_onclick(heaptmap_value):
#     return '/static/videoResult_vp9_vbr.mkv'

if __name__ == '__main__':
    app.run_server()