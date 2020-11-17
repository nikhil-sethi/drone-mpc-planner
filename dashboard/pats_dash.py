#!/usr/bin/env python3
import socket, json,shutil,subprocess,sqlite3
import datetime,time, argparse,math,pickle, glob, os, re
from dash_core_components.Dropdown import Dropdown
import numpy as np
import pandas as pd

import dash,dash_auth,flask
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from dash.dependencies import Output, Input, State
import dash_bootstrap_components as dbc
import dash_daq as daq

from werkzeug.security import generate_password_hash, check_password_hash
from flask import Flask, send_from_directory
from urllib.parse import quote as urlquote

db_path = ''
db_classification_path = ''
heatmap_max=50


server = flask.Flask(__name__)
app = dash.Dash(__name__, server=server)
app.title = 'PATS-C'

pio.templates.default = 'plotly_dark'

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
def open_classification_db():
    global db_classification_path
    conn = None
    cur = None
    try:
        conn = sqlite3.connect(db_classification_path)
        cur = conn.cursor()
    except Exception as e:
        print(e)
    return conn,cur

def load_systems(username):
    _,cur = open_db()
    sql_str = '''SELECT DISTINCT system FROM mode_records ORDER BY system '''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = natural_sort_systems(systems)
    systems = [d[0] for d in systems]

    authorized_systems = []
    user_pass_dict,user_group_dict = read_cred_db()

    group = user_group_dict[username]
    for sys in systems:
        sys_id = int(sys.replace('pats','').replace('-proto',''))
        if (group == 'admin' or group == 'pats') and (sys_id != 25 and sys_id != 26 and sys_id != 10):
            authorized_systems.append(sys)
        elif group == 'koppertcress' and (sys_id > 10 and sys_id <= 21 or sys_id ==4 or sys_id == 6):
            authorized_systems.append(sys)
        elif (group == 'wur') and (sys_id == 23 or sys_id == 24):
            authorized_systems.append(sys)
        elif (group == 'holstein') and (sys_id == 2 or sys_id == 22):
            authorized_systems.append(sys)
        elif (group == 'vde') and (sys_id == 7):
            authorized_systems.append(sys)
        elif (group == 'lyprauta') and (sys_id == 27):
            authorized_systems.append(sys)

    return authorized_systems

def system_sql_str(systems):
    if not isinstance(systems, list):
        systems = [systems]
    systems_str = ' ('
    for system in systems:
        systems_str = systems_str + 'system="' + system + '" OR '
    systems_str = systems_str[:-4] + ') '
    return systems_str

def execute(cmd,retry=1):
    p_result = None
    n=0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
        for stdout_line in iter(popen.stdout.readline, ''):
            p_result = popen.poll()
            if p_result or p_result==0:
                n = n+1
                break
            print(str(cmd) + ' resulted in error:')
            print(stdout_line.decode('utf-8'),end ='')

            time.sleep(0.1)
        popen.stdout.close()

def load_moth_data(selected_systems,selected_dayrange):
    systems_str = system_sql_str(selected_systems)
    _,cur = open_db()

    sql_str = 'SELECT MIN(time) FROM moth_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=selected_dayrange)
    start_date_str = start_date.strftime('%Y%m%d_%H%M%S')
    sql_str += 'AND time>"' + start_date_str + '"'
    cur.execute(sql_str)
    first_date = cur.fetchone()[0]

    sql_str = 'SELECT MAX(time) FROM moth_records WHERE ' + systems_str
    sql_str += 'AND time > "' + start_date_str + '"'
    cur.execute(sql_str)
    end_date = cur.fetchone()[0]
    if not first_date or not end_date:
        return [],[],[],[],[]

    end_date = datetime.datetime.strptime(end_date, '%Y%m%d_%H%M%S')
    d = start_date-datetime.timedelta(hours=12)
    unique_dates = []
    while d <= end_date - datetime.timedelta(hours=12):
        unique_dates.append(d.strftime('%d-%m-%Y'))
        d += datetime.timedelta(days=1)

    moth_columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]


    heatmap_data = np.zeros((len(unique_dates),24))
    heatmap_data_lists = np.empty((len(unique_dates),24),dtype=object)

    sql_str = 'SELECT * FROM moth_records WHERE ((duration > 1 AND duration < 10 AND Dist_traveled > 0.15 AND Dist_traveled < 4) OR (Version="1.0" AND duration > 1 AND duration < 10)) AND '
    sql_str += systems_str
    if selected_dayrange > 0:
        sql_str += 'AND time > "' + start_date_str + '"'
    cur.execute(sql_str)
    moths = cur.fetchall()


    start_date_col = moth_columns.index('time')
    for moth in moths:
        d = datetime.datetime.strptime(moth[start_date_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        day = (d.date() - start_date.date()).days
        hour = d.hour
        heatmap_data[day,hour] += 1
        if not heatmap_data_lists[day,hour]:
            heatmap_data_lists[day,hour] = [moth]
        else:
            heatmap_data_lists[day,hour].append(moth)


    hist_data = np.zeros((len(unique_dates),len(selected_systems)))
    hist_data = pd.DataFrame(hist_data,columns=selected_systems)
    start_date_col = moth_columns.index('time')
    for moth in moths:
        d = datetime.datetime.strptime(moth[start_date_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        day = (d.date() - start_date.date()).days
        hist_data[moth[moth_columns.index('system')]][day] += 1


    return unique_dates,heatmap_data,heatmap_data_lists,hist_data,moth_columns

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

def load_mode_data(unique_dates,heatmap_data,selected_systems,selected_dayrange):
    if not len(unique_dates):
        return [],[]
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()

    sql_str = 'SELECT * FROM mode_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=selected_dayrange)
    start_date_str = start_date.strftime('%Y%m%d_%H%M%S')
    sql_str += 'AND start_datetime > "' + start_date_str + '"'
    sql_str += ' ORDER BY "start_datetime"'
    cur.execute(sql_str)
    modes = cur.fetchall()

    first_date = datetime.datetime.strptime(unique_dates[0],'%d-%m-%Y')

    mode_columns = [i[1] for i in cur.execute('PRAGMA table_info(mode_records)')]
    start_col = mode_columns.index('start_datetime')
    end_col = mode_columns.index('end_datetime')
    mode_col = mode_columns.index('op_mode')

    modemap_data = heatmap_data.copy()
    modemap_data.fill(-666)
    for mode in modes:
        d_mode_start = datetime.datetime.strptime(mode[start_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        d_mode_end = datetime.datetime.strptime(mode[end_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        len_hours = math.ceil((d_mode_end-d_mode_start).seconds/3600)

        day = (d_mode_start.date() - start_date.date()).days
        hour = d_mode_start.hour

        m = convert_mode_to_number(mode[mode_col])
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
    return heatmap_data,modemap_data

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def create_heatmap(unique_dates,heatmap_counts,xlabels, selected_cells):
    hover_label = pd.DataFrame(heatmap_counts).astype(int).astype(str)
    hover_label[hover_label=='-1'] = 'NA'
    heatmap_data = np.clip(heatmap_counts, -1, heatmap_max)
    if (selected_cells):
        selected_cells = pd.read_json(selected_cells, orient='split')
        for _,cel in selected_cells.iterrows():
            x=cel['x']
            y=cel['y']
            if heatmap_data[y,x]>=0:
                heatmap_data[y,x]=-2

    hm = go.Heatmap(
            x = xlabels,
            y = unique_dates,
            z = heatmap_data,
            customdata = hover_label,
            zmin = -2,
            zmid = heatmap_max/2,
            zmax = heatmap_max,
            colorscale = [
                        [0, 'rgba(128, 128, 200, 0.65)'], #selected cell
                        [1/(heatmap_max+2), 'rgba(0, 0, 0, 1.0)'], #system inactive
                        [2/(heatmap_max+2), 'rgba(0,255,0, 1.0)'],
                        [1, 'rgba(255,0,0, 1.0)']
                        ],
            hovertemplate = 'Time: %{x}<br>' +
            'Count: %{customdata}' +
             '<extra></extra>'
            )
    fig = go.Figure(data=hm)
    fig['layout']['xaxis']['side'] = 'top'
    fig['layout']['xaxis']['tickangle'] = 45

    if len(unique_dates) > 14:
        h = len(unique_dates) * 15+200
    elif len(unique_dates):
        h = len(unique_dates) * 30+200
    else:
        h=10
    fig.update_layout(
        height=h,
        title_text='Moth counts per hour:',
        clickmode='event+select'
    )
    style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '50%'}
    return fig,style
def create_hist(df_hist,unique_dates,selected_systems):
    fig = go.Figure()
    cnt = 0
    for sys in selected_systems:
        hist_data = df_hist[sys]
        syss = [sys] * len(hist_data)
        hist = go.Bar(
            x = unique_dates,
            y = hist_data,
            customdata = np.transpose([syss,syss]), #ok, we should only need one column of syss, spend an hour or streamlining this just to conclude something weird is going on in there...
            marker_color = px.colors.qualitative.Vivid[cnt],
            name = sys.replace('proto','').replace('pats','Pats'),
            hovertemplate = '<b>System %{customdata[0]}</b><br><br>' +
                    '<extra></extra>'
        )
        cnt+=1
        fig.add_trace(hist)

    fig.update_layout(
        title_text='Moth counts per day',
        xaxis_title_text='Days',
        yaxis_title_text='Counts',
        barmode='stack',
        clickmode='event+select'
    )
    hist_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '50%'}
    return fig,hist_style

def classification_to_symbol(x):
    if not x:
        return 0
    return classification_options.index(x)
def remove_nones_classification(x):
    if not x:
        return classification_options[0]
    return x
def video_available_to_symbol(x):
    if not x or x == 'NA':
        return 1
    return 0
def create_scatter(moths,selected_systems,scatter_x_value,scatter_y_value):
    df_scatter = pd.DataFrame(moths,columns=moth_columns)
    df_scatter['system_ids'] = df_scatter['system'].str.replace('pats-proto','').astype(int)
    df_scatter['system_color'] = df_scatter['system'].apply(selected_systems.index).astype(int)
    df_scatter['classification_symbol'] = df_scatter['Human_classification'].apply(classification_to_symbol)
    df_scatter['Human_classification'] = df_scatter['Human_classification'].apply(remove_nones_classification)
    df_scatter['video_symbol'] = df_scatter['Video_Filename'].apply(video_available_to_symbol)
    df_scatter['classification_symbol'] = df_scatter['classification_symbol']

    scat_fig = go.Figure()
    for sys in selected_systems:
        df_scatter_sys = df_scatter[df_scatter['system']==sys]
        for classification in classification_options:
            tmp1 = list(df_scatter['Human_classification']==classification) #which rows have the same classification
            df = df_scatter_sys[tmp1] #retrieve only those rows that have the same classficication
            df = df.fillna('')
            if not df.empty:
                scatter = go.Scattergl(
                    name = sys.replace('proto','').replace('pats','Pats') + ', ' + classification,
                    showlegend=True,
                    x = df[scatter_x_value],
                    y = df[scatter_y_value],
                    mode = 'markers',
                    customdata = np.stack(
                        (df['system'] + '/' + df['Folder'] + '/' + df['Filename'],
                        df['Video_Filename'],
                        df['Human_classification'],
                        df['system_ids'],
                        df['uid']
                    ), axis=-1),
                    marker=dict(
                        cmin = 0,
                        cmax = 9,
                        color=df['system_color'],
                        colorscale=px.colors.qualitative.Vivid,
                        symbol=df['classification_symbol'],
                        size=10,
                        line=dict(width=2,color='DarkSlateGrey')
                    ),
                    hovertemplate = '<b>System %{customdata[3]}</b><br><br>' +
                    'x: %{x}<br>' +
                    'y: %{y}<br>' +
                    'File: %{customdata[0]}<br>' +
                    'Video: %{customdata[1]}<br>' +
                    'Ground truth: %{customdata[2]}<br>' +
                    '<extra></extra>'
                )
                scat_fig.add_trace(scatter)


    scat_fig.update_layout(
        title_text='Selected moths:',
        xaxis_title = scatter_columns[scatter_x_value],
        yaxis_title = scatter_columns[scatter_y_value],
        clickmode='event+select',
        legend_title_text='Legend',
    )
    return scat_fig

def download_log(selected_moth):
    sys_name = selected_moth[moth_columns.index('system')].replace('-proto','')
    log_fn = selected_moth[moth_columns.index('Filename')]
    log_folder = selected_moth[moth_columns.index('Folder')]
    if not log_folder or log_fn == 'unknown':
        return ''
    target_log_fn = 'static/' + log_folder + '_' + sys_name + '_' + log_fn
    if not os.path.isfile(target_log_fn):
        rsync_src = sys_name + ':data/' + log_folder + '/logging/' + log_fn
        cmd = ['rsync --timeout=5 -az ' + rsync_src + ' ' + target_log_fn]
        execute(cmd)
    return target_log_fn
def file_download_link(filename):
    location = '{}'.format(urlquote(filename))
    return html.A('Download log',href=location,style={'display': 'block'}),{'textAlign':'center','width': '25%','margin':'auto','display': 'block'}
def create_path_plot(target_log_fn):
    df_ilog = pd.read_csv(target_log_fn,delimiter=';')
    target_log_fn = '/' + target_log_fn
    rows_without_tracking = df_ilog[ df_ilog['n_frames_tracking_insect'] == 0 ].index
    df_ilog = df_ilog.drop(rows_without_tracking)
    scatter = go.Scatter3d(
        x = df_ilog['sposX_insect'],
        y = -df_ilog['sposZ_insect'],
        z = df_ilog['sposY_insect'],
        text = df_ilog['time'],
        mode='markers',
        hovertemplate = '<b>t= %{text}</b><br>x= %{x}<br>y= %{y}<br>z= %{z}'
    )
    fig = go.Figure(data=scatter)
    fig.update_layout(
        title_text='Moth path:',
         scene=dict(
                 aspectmode='data'
         )
    )
    style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '50%'}
    return fig,style
def download_video(selected_moth):
    video_fn = selected_moth[moth_columns.index('Video_Filename')]
    sys_name = selected_moth[moth_columns.index('system')].replace('-proto','')

    target_video_fn = ''
    if video_fn and video_fn != 'NA':
        target_video_fn = 'static/' + selected_moth[moth_columns.index('Folder')] + '_' + sys_name + '_' + video_fn
        if not os.path.isfile(target_video_fn):
            rsync_src = sys_name + ':data/' +selected_moth[moth_columns.index('Folder')] + '/logging/render_' + video_fn
            cmd = ['rsync --timeout=5 -a ' + rsync_src + ' ' + target_video_fn]
            execute(cmd)
            target_video_fn = '/' + target_video_fn
            if not os.path.isfile(target_video_fn[1:]):
                target_video_fn = ''
    return target_video_fn

def read_cred_db():
    cred_file = os.path.expanduser('~/.dash_auth')

    if os.path.exists(cred_file):
        user_pass_dict = {}
        user_group_dict = {}
        with open (cred_file, 'r') as creds_file:
                creds = creds_file.readlines()
                for cred in creds:
                    cred = cred.split(':')
                    user_group_dict[cred[0].strip()] = cred[1].strip()
                    user_pass_dict[cred[0].strip()] = cred[2].strip()
    else:
        user_pass_dict = {'user': 'user'}
        user_group_dict = {'user': 'admin'}
    return user_pass_dict,user_group_dict

def selected_dates(selected_daterange):
    if selected_daterange == 'Last week':
        return 7
    if selected_daterange == 'Last two weeks':
        return 14
    elif selected_daterange == 'Last month':
        return 31
    elif selected_daterange == 'Last three months':
        return 92
    else:
        return 1

@app.callback(
    Output('systems_dropdown', 'options'),
    Input('date_range_dropdown', 'options'))
def init_system_dropdown(_): #unfortunately we have to do this init through a click event, because authorization doesn't work otherwise
    username = flask.request.authorization['username']
    systems = load_systems(username)
    options = []
    for system in systems:
        options.append({'label': system, 'value': system} )
    return options

@app.callback(
    Output('hete_kaart', 'figure'),
    Output('staaf_kaart', 'figure'),
    Output('hete_kaart', 'style'),
    Output('staaf_kaart', 'style'),
    Output('selected_heatmap_data', 'children'),
    Output('Loading_animation','style'),
    Input('date_range_dropdown', 'value'),
    Input('systems_dropdown', 'value'),
    Input('hete_kaart', 'clickData'),
    State('selected_heatmap_data', 'children'))
def update_ui_hist_and_heat(selected_daterange,selected_systems,selected_hm_cells,selected_heat):
    hm_fig=go.Figure(data=go.Heatmap())
    hist_fig=go.Figure(data=go.Histogram())
    hm_style={'display': 'none'}
    hist_style={'display': 'none'}
    Loading_animation_style={'display': 'block'}

    ctx = dash.callback_context
    if ctx.triggered[0]['prop_id'] == 'date_range_dropdown.value' or ctx.triggered[0]['prop_id'] == 'systems_dropdown.value':
        selected_heat = None

    selected_dayrange = selected_dates(selected_daterange)
    if not selected_systems or not selected_systems:
        return hm_fig,hist_fig,hm_style,hist_style,selected_heat,Loading_animation_style
    unique_dates,heatmap_data,heatmap_data_lists,df_hist,_ = load_moth_data(selected_systems,selected_dayrange)
    if not len(unique_dates):
        return hm_fig,hist_fig,hm_style,hist_style,selected_heat,Loading_animation_style

    hist_fig,hist_style=create_hist(df_hist,unique_dates,selected_systems)

    if ctx.triggered[0]['prop_id'] == 'hete_kaart.clickData' or ctx.triggered[0]['prop_id'] == 'classification_dropdown.value':
        if not selected_hm_cells:
            selected_heat = None
        else:
            selected_heat = []
            for cel in selected_hm_cells['points']:
                x = xlabels.index(cel['x'])
                y = unique_dates.index(cel['y'])
                if heatmap_data_lists[y,x]:
                    selected_heat.append([x,y])

            selected_heat = pd.DataFrame(selected_heat,columns=['x','y'])
            selected_heat = selected_heat.to_json(date_format='iso', orient='split')

    heatmap_data,_ = load_mode_data(unique_dates,heatmap_data,selected_systems,selected_dayrange) #add mode info to heatmap, which makes the heatmap show when the system was online (color) or offline (black)
    hm_fig,hm_style=create_heatmap(unique_dates,heatmap_data,xlabels,selected_heat)

    Loading_animation_style = {'display': 'none'}

    return hm_fig,hist_fig,hm_style,hist_style,selected_heat,Loading_animation_style

@app.callback(
    Output('verstrooide_kaart', 'figure'),
    Output('verstrooide_kaart', 'style'),
    Output('scatter_dropdown_container', 'style'),
    Input('date_range_dropdown', 'value'),
    Input('systems_dropdown', 'value'),
    Input('hete_kaart', 'clickData'),
    Input('staaf_kaart', 'selectedData'),
    Input('scatter_x_dropdown', 'value'),
    Input('scatter_y_dropdown', 'value'),
    Input('classification_dropdown', 'value'),
    State('selected_heatmap_data', 'children'))
def update_ui_scatter(selected_daterange,selected_systems,hm_selected_cells,hist_selected_bars,scatter_x_value,scatter_y_value,classification_dropdown,selected_heat):
    scat_fig=go.Figure(data=go.Scatter())
    scat_style={'display': 'none'}
    scat_axis_select_style={'display': 'none'}

    ctx = dash.callback_context

    if not selected_systems:
        return scat_fig,scat_style,scat_axis_select_style

    if ctx.triggered[0]['prop_id'] == 'staaf_kaart.selectedData' or ctx.triggered[0]['prop_id'] == 'hete_kaart.clickData' or ctx.triggered[0]['prop_id'] == 'classification_dropdown.value' or ctx.triggered[0]['prop_id'] == 'scatter_x_dropdown.value' or ctx.triggered[0]['prop_id'] == 'scatter_y_dropdown.value':
        moths = []
        if hm_selected_cells:
            for cel in hm_selected_cells['points']:
                hour = int(cel['x'].replace('h',''))
                start_date = datetime.datetime.strptime(cel['y'], '%d-%m-%Y') + datetime.timedelta(hours=hour)
                end_date = start_date + datetime.timedelta(hours=1)
                sql_str = 'SELECT * FROM moth_records WHERE time >"' + start_date.strftime('%Y%m%d_%H%M%S') + '" AND time <= "' + end_date.strftime('%Y%m%d_%H%M%S') + '" AND ' + system_sql_str(selected_systems)
                _,cur = open_db()
                cur.execute(sql_str)
                entries = cur.fetchall()
                moths.extend(entries)

        elif hist_selected_bars:
            for bar in hist_selected_bars['points']:
                sys = bar['customdata'][0]
                start_date = datetime.datetime.strptime(bar['x'], '%d-%m-%Y') + datetime.timedelta(hours=12)
                end_date = start_date + datetime.timedelta(days=1)
                sql_str = 'SELECT * FROM moth_records WHERE system="' + sys + '" AND time >"' + start_date.strftime('%Y%m%d_%H%M%S') + '" AND time <= "' + end_date.strftime('%Y%m%d_%H%M%S') + '"'
                _,cur = open_db()
                cur.execute(sql_str)
                entries = cur.fetchall()
                moths.extend(entries)

        if moths:
            scat_fig = create_scatter(moths,selected_systems,scatter_x_value,scatter_y_value)
            scat_axis_select_style={'display': 'table','textAlign':'center','width':'50%','margin':'auto'}
            scat_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '50%'}
    return scat_fig,scat_style,scat_axis_select_style

@app.callback(
    Output('insect_video', 'src'),
    Output('insect_video', 'style'),
    Output('route_kaart', 'figure'),
    Output('route_kaart', 'style'),
    Output('log_file_link', 'children'),
    Output('log_file_link_container', 'style'),
    Output('selected_scatter_moth', 'children'),
    Output('classification_dropdown', 'value'),
    Output('classify_container', 'style'),
    Output('Loading_animation_moth','style'),
    Input('date_range_dropdown', 'value'),
    Input('systems_dropdown', 'value'),
    Input('hete_kaart', 'clickData'),
    Input('verstrooide_kaart', 'clickData'),
    State('verstrooide_kaart','figure'))
def update_moth_ui(selected_daterange,selected_systems,clickData_hm,clickData_dot,scatter_fig_state):
    target_video_fn = ''
    video_style={'display': 'none'}
    path_fig=go.Figure(data=go.Scatter3d())
    path_style={'display': 'none'}
    file_link = html.A('File not available.',style={'display': 'none'})
    file_link_style = {'textAlign':'center','width': '25%','margin':'auto','display': 'none'}
    selected_moth=None
    classification = classification_options[0]
    classify_style={'display': 'none'}
    Loading_animation_style={'display': 'block'}

    ctx = dash.callback_context
    if not 'selectedpoints' in scatter_fig_state['data'][0] or ctx.triggered[0]['prop_id'] != 'verstrooide_kaart.clickData' or not selected_systems:
        return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style

    sql_str = 'SELECT * FROM moth_records WHERE uid=' + str(clickData_dot['points'][0]['customdata'][4])
    _,cur = open_db()
    cur.execute(sql_str)
    entry = cur.fetchall()
    if not len(entry):
        return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style
    selected_moth = entry[0]

    target_log_fn = download_log(selected_moth)
    if not os.path.isfile(target_log_fn):
        return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style
    file_link,file_link_style =file_download_link(target_log_fn)

    if 'Human_classification' in moth_columns:
        classification=selected_moth[moth_columns.index('Human_classification')]
    if not classification:
        classification = classification_options[0]
    classify_style = {'display':'block','textAlign':'center','width':'25%','margin':'auto'}

    path_fig,path_style = create_path_plot(target_log_fn)

    target_video_fn = download_video(selected_moth)
    Loading_animation_style = {'display': 'none'}
    if target_video_fn != '':
        video_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '75%'}

    return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style

@app.callback(
    Output(component_id='classification_hidden', component_property='children'),
    Input('classification_dropdown', 'value'),
    Input('selected_scatter_moth', 'children'))
def classification_value(selected_classification,selected_moth):
    ctx = dash.callback_context
    if not selected_moth or ctx.triggered[0]['prop_id'] != 'classification_dropdown.value':
        return []
    conn,cur = open_db()
    cols = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]

    current_classification = selected_moth[cols.index('Human_classification')]
    if not current_classification:
        current_classification = classification_options[0]
    if current_classification != selected_classification:
        sql_str = 'UPDATE moth_records SET Human_classification="' + selected_classification + '" WHERE uid=' + str(selected_moth[cols.index('uid')])
        cur.execute(sql_str)
        conn.commit()

        #also save user classifications to a seperate database, because the moth database sometimes needs to be rebuild from the logs/jsons
        conn_class,cur_class = open_classification_db()
        cur_class.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='classification_records' ''')
        table_exists = cur_class.fetchone()[0]
        if not table_exists:
            sql_create = 'CREATE TABLE classification_records(uid INTEGER PRIMARY KEY,moth_uid INTEGER,system TEXT,time TEXT,duration REAL,user TEXT,classification TEXT)'
            cur_class.execute(sql_create)
            conn_class.commit()

        username = flask.request.authorization['username']
        sql_insert = 'INSERT INTO classification_records(moth_uid,system,time,duration,user,classification) VALUES(?,?,?,?,?,?)'
        data = [selected_moth[cols.index('uid')],str(selected_moth[cols.index('system')]),str(selected_moth[cols.index('time')]),str(selected_moth[cols.index('duration')]),username,selected_classification]
        cur_class.execute(sql_insert, data)
        conn_class.commit()

    return selected_classification

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dash app that shows PATS-C monitoring and system analysis')
    parser.add_argument('--db', help='Path to the sql database', default='~/pats.db')
    parser.add_argument('--db_classification', help='Path to the sql database', default='~/pats_human_classification.db')
    args = parser.parse_args()
    db_path = os.path.expanduser(args.db)
    db_classification_path = os.path.expanduser(args.db_classification)
else:
    db_path = os.path.expanduser('~/pats.db')
    db_classification_path = os.path.expanduser('~/pats_human_classification.db')

if not os.path.exists('static'):
    os.makedirs('static')

conn,cur = open_db()
moth_columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]
if 'Human_classification' not in moth_columns:
    cur.execute('ALTER TABLE moth_records ADD COLUMN Human_classification TEXT')

    if os.path.exists(db_classification_path):
        conn_class,cur_class = open_classification_db()
        classification_columns = [i[1] for i in cur_class.execute('PRAGMA table_info(classification_records)')]
        sql_str = 'SELECT * FROM classification_records'
        cur_class.execute(sql_str)
        classification_data = cur_class.fetchall()
        for entry in classification_data:
            sql_where=' WHERE uid=' + str(entry[classification_columns.index('moth_uid')]) + ' AND time="' + str(entry[classification_columns.index('time')]) + '" AND duration=' + str(entry[classification_columns.index('duration')])
            sql_str = 'UPDATE moth_records SET Human_classification="' + entry[classification_columns.index('classification')] + '"' + sql_where
            cur.execute(sql_str)
            tmp = cur.fetchall()
        conn.commit()
        print('Re-added cliassification results to main db. Added ' + str(len(classification_data)) + ' classifications.')

user_pass_dict,_ = read_cred_db()
auth = dash_auth.BasicAuth(app,user_pass_dict)

dateranges = ['Last week','Last two weeks', 'Last month', 'Last three months']
classification_options = ['Not classified','Moth!','Empty/nothing to see','Other insect','Plant','Other false positive']
xlabels = []
for i in range(0,24):
    xlabels.append(str((i+12)%24)+'h')

scatter_columns = { 'duration' : 'Duration',
                    'Vel_mean':'Velocity mean',
                    'Vel_max':'Velocity max',
                    'Dist_traveled':'Distance traveled',
                    'Dist_traject':'Distance trajectory',
                    'Size':'Size'}

systems = []
fig_hm = go.Figure(data=go.Heatmap())
fig_hist=go.Figure(data=go.Histogram())
fig_scatter = go.Figure(data=go.Scatter())
fig_path = go.Figure(data=go.Scatter3d())
app.layout = html.Div(children=[
    html.H1(children='PATS-C'),
    html.Div([
        html.Div([
            html.Div('Select systems:'),
            html.Div(dcc.Dropdown(
                id='systems_dropdown',
                options=[{'label': system, 'value': system} for system in systems],
                multi=True
            ), className='dash-bootstrap'),
        ], style={'width': '50%', 'display': 'inline-block'}),
        html.Div([
            html.Div('Select range:'),
            html.Div(dcc.Dropdown(
                id='date_range_dropdown',
                options=[{'label': daterange, 'value': daterange} for daterange in dateranges],
                value=dateranges[0],
                searchable=False,
                clearable=False
            ), className='dash-bootstrap'),
        ], style={'width': '50%', 'display': 'inline-block'})
    ],style={'display': 'block','textAlign':'center','width':'50%','margin':'auto'}),
    html.Div([
        dcc.Loading(
            children=html.Div([html.Br(),html.Br(),html.Br()],id='Loading_animation',style={'display': 'none'}),
            type='default'
        ),
        dcc.Graph(id='staaf_kaart',style={'display': 'none','margin-left': 'auto','margin-right': 'auto','textAlign':'center', 'width': '50%'},figure=fig_hist)
    ]),
    html.Div([
        dcc.Graph(id='hete_kaart',style={'display': 'none'}, figure=fig_hm)
    ]),
    html.Div([
        html.Br(),
        html.Div('Select x axis:',style={'display':'table-cell'}),
        html.Div(dcc.Dropdown(
            id='scatter_x_dropdown',
            options=[{'label': scatter_columns[key], 'value': key} for key in scatter_columns],
            value='duration',
            clearable=False
        ),style={'display': 'table-cell'},className='dash-bootstrap'),
        html.Div('Select y axis:',style={'display':'table-cell'}),
        html.Div(dcc.Dropdown(
            id='scatter_y_dropdown',
            options=[{'label': scatter_columns[key], 'value': key} for key in scatter_columns],
            value='Vel_mean',
            clearable=False
        ),style={'display': 'table-cell'},className='dash-bootstrap'),
    ],style={'display': 'none','textAlign':'center','width':'50%','margin':'auto'},id='scatter_dropdown_container'),
    html.Div([
        dcc.Graph(id='verstrooide_kaart',style={'display': 'none'}, figure=fig_scatter)
    ]),
    html.Div([
        dcc.Loading(
            children=html.Div([html.Br(),html.Br(),html.Br()],id='Loading_animation_moth',style={'display': 'none'}),
            type='default'
        ),
        dcc.Graph(id='route_kaart',style={'display': 'none'}, figure=fig_path),
        html.Video(id='insect_video',style={'display': 'none'},controls=True,loop=True,autoPlay=True),
    ]),
    html.Div([
        html.Div([
            'Human classification:']
            ),
        html.Div(dcc.Dropdown(
            id='classification_dropdown',
            options=[{'label': c, 'value': c} for c in classification_options],
            value=classification_options[0],
            clearable=False,
        ),className='dash-bootstrap'),
    ], style= {'display':'none','textAlign':'center','width':'25%','margin':'auto'},id='classify_container'),
    html.Div([
        html.Br(),
        html.Ul(id='log_file_link'),
        html.Br(),
        html.Br()
        ],style={'display': 'none'},id='log_file_link_container'),
    html.Div(id='selected_heatmap_data', style={'display': 'none'}),
    html.Div(id='selected_scatter_moth', style={'display': 'none'}),
    html.Div(id='classification_hidden', style={'display': 'none'})
])

if __name__ == '__main__':
    app.run_server(debug=True)
