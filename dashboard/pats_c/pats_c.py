#!/usr/bin/env python3
import datetime, math, os, re
from dateutil.relativedelta import relativedelta
import numpy as np
import pandas as pd

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from dash.dependencies import Output, Input, State
import dash_bootstrap_components as dbc
import dash_daq as daq
import pats_c.lib.lib_patsc as patsc

from flask_login import current_user
from urllib.parse import quote as urlquote

group_dict = {}
moth_columns = []
heatmap_max = 50
classification_options = ['Not classified','Moth!','Empty/nothing to see','Other insect','Plant','Other false positive']
scatter_columns = { 'duration' : 'Duration (s)',
                    'Vel_mean':'Velocity mean (m/s)',
                    'Vel_max':'Velocity max (m/s)',
                    'Dist_traveled':'Distance traveled (m)',
                    'Dist_traject':'Distance trajectory (m)',
                    'Size':'Size (m)'}

def load_systems_group(group_name,cur):
    sql_str = f'''SELECT system,location FROM systems JOIN groups ON groups.group_id = systems.group_id WHERE groups.name = "{group_name}" ORDER BY system_id'''
    systems = cur.execute(sql_str).fetchall()
    return systems

def load_groups():
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
            with patsc.open_systems_db() as con:
                sql_str = f'''SELECT groups.name FROM groups JOIN customer_group_connection ON customer_group_connection.group_id = groups.group_id JOIN customers ON customers.customer_id = customer_group_connection.customer_id WHERE customers.name = "{username}" ORDER BY groups.name'''
                cur = con.execute(sql_str)
                groups = cur.fetchall()
                group_dict = {}
                for group in groups:
                    group_dict[group[0]] = load_systems_group(group[0],cur)
            return group_dict
    return {}

def init_system_dropdown():
    group_dict = load_groups()
    sys_options = []
    group_options = []
    group_value = None
    group_style={'width': '30%', 'display': 'inline-block'}
    sys_style={'width': '70%', 'display': 'inline-block'}
    for group in group_dict.keys():
        if not ( group == 'maintance' or group == 'admin' or group == 'unassigned_systems' or group == 'deactivated_systems'):
            group_options.append({'label':group,'value':group})
            for i,(system,location) in enumerate(group_dict[group]):
                if group == 'pats':
                    sys_options.append({'label':system.replace('-proto',''),'value':system,'title':system.replace('-proto','')})
                elif location:
                    if len(group_dict.keys()) == 1:
                        sys_options.append({'label':location,'value':system,'title':system.replace('-proto','')})
                    else:
                        sys_options.append({'label':group+' '+location,'value':system,'title':system.replace('-proto','')})
                else:
                    sys_options.append({'label':group+' '+str(i+1),'value':system,'title':system.replace('-proto','')})
        if len(group_dict.keys()) == 1:
            group_value = list(group_dict.keys())
            group_style={'width': '0%', 'display': 'none'}
            sys_style={'width': '100%', 'display': 'inline-block'}
    return sys_options, sys_style, group_options, group_value, group_style

def load_systems(username):
    with patsc.open_systems_db() as con:
        sql_str = '''SELECT DISTINCT systems.system FROM systems,customer_group_connection,customers WHERE  systems.group_id = customer_group_connection.group_id AND customer_group_connection.customer_id = customers.customer_id AND customers.name = ? ORDER BY systems.system_id '''
        systems = con.execute(sql_str,(username,)).fetchall()
    authorized_systems = [d[0].replace('-proto','') for d in systems]
    return authorized_systems

def load_moth_df(selected_systems,start_date,end_date):
    username = current_user.username
    with patsc.open_systems_db() as con:
        ordered_systems = con.execute('''SELECT systems.system, groups.minimal_size, systems.installation_date FROM systems,groups
        JOIN customer_group_connection ON systems.group_id = customer_group_connection.group_id
        JOIN customers ON customers.customer_id = customer_group_connection.customer_id WHERE
        systems.group_id = groups.group_id AND customers.name = ? AND systems.system IN (%s)
        ORDER BY systems.system_id'''%('?,'*len(selected_systems))[:-1], (username,*selected_systems)).fetchall()

    moth_df = pd.DataFrame()
    with patsc.open_data_db() as con:
        for (system,min_size,installation_date) in ordered_systems:
            try:
                installation_date = datetime.datetime.strptime(installation_date,'%Y%m%d_%H%M%S')
                real_start_date = max([installation_date,start_date])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_date
            sql_str = f'''SELECT moth_records.* FROM moth_records
            WHERE (system = "{system}" OR system = "{system.replace('pats','pats-proto')}")
            AND time > "{real_start_date.strftime('%Y%m%d_%H%M%S')}" AND time <= "{end_date.strftime('%Y%m%d_%H%M%S')}"
            AND (duration > 1 AND duration < 10
            AND (Version="1.0"
            OR (Dist_traveled > 0.15 AND Dist_traveled < 4 AND Size > {min_size} )))'''
            sql_str = sql_str.replace('\n','')
            moth_df = moth_df.append(pd.read_sql_query(sql_str,con))
    moth_df['time'] = pd.to_datetime(moth_df['time'], format = '%Y%m%d_%H%M%S')
    moth_df['system'].replace({'-proto':''},regex=True,inplace=True)
    return moth_df

def load_moth_of_hour(selected_systems,start_date,end_date,hour):
    username = current_user.username
    with patsc.open_systems_db() as con:
        ordered_systems = con.execute('''SELECT systems.system, groups.minimal_size, systems.installation_date FROM systems,groups
        JOIN customer_group_connection ON systems.group_id = customer_group_connection.group_id
        JOIN customers ON customers.customer_id = customer_group_connection.customer_id WHERE
        systems.group_id = groups.group_id AND customers.name = ? AND systems.system IN (%s)
        ORDER BY systems.system_id'''%('?,'*len(selected_systems))[:-1], (username,*selected_systems)).fetchall()

    hour_str = str(hour)
    if len(hour_str)==1:
        hour_str = '0' + hour_str
    moth_df = pd.DataFrame()
    with patsc.open_data_db() as con:
        for (system,min_size,installation_date) in ordered_systems:
            try:
                installation_date = datetime.datetime.strptime(installation_date,'%Y%m%d_%H%M%S')
                real_start_date = max([installation_date,start_date])
            except ValueError as e:
                print(f'ERROR startdate {system}: ' + str(e))
                real_start_date = start_date
            sql_str = f'''SELECT moth_records.* FROM moth_records
            WHERE (system = "{system}" OR system = "{system.replace('pats','pats-proto')}")
            AND time > "{real_start_date.strftime('%Y%m%d_%H%M%S')}" AND time <= "{end_date.strftime('%Y%m%d_%H%M%S')}"
            AND time LIKE "_________{hour_str}____"
            AND (duration > 1 AND duration < 10
            AND (Version="1.0"
            OR (Dist_traveled > 0.15 AND Dist_traveled < 4 AND Size > {min_size} )))'''
            sql_str = sql_str.replace('\n','')
            moth_df = moth_df.append(pd.read_sql_query(sql_str,con))
    moth_df['time'] = pd.to_datetime(moth_df['time'], format = '%Y%m%d_%H%M%S')
    moth_df['system'].replace({'-proto':''},regex=True,inplace=True)
    return moth_df

def remove_unauthoirized_system(selected_systems): #this is solely a security related check
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
        systems_str = systems_str + 'system="' + system + '" OR ' + 'system="' + system.replace('pats','pats-proto') + '" OR '
    systems_str = systems_str[:-4] + ') '
    return systems_str

def load_moth_data(selected_systems,start_date,end_date):
    # We count moths from 12 in the afternoon to 12 in the afternoon. Therefore we shift the data in the for loops with 12 hours.
    # So a moth at 23:00 on 14-01 is counted at 14-01 and a moth at 2:00 on 15-01 also at 14-01
    # A moth seen at 13:00 on 14-01 still belongs to 14-01 but when seen at 11:00 on 14-01 is counted at 13-01
    end_date = datetime.datetime.combine(end_date, datetime.datetime.min.time())
    start_date = datetime.datetime.combine(start_date, datetime.datetime.min.time())
    moth_df = load_moth_df(selected_systems,start_date,end_date+datetime.timedelta(hours=12)) #Shift to include the moths of today until 12:00 in the afternoon.
    unique_dates = pd.date_range(start_date,end_date-datetime.timedelta(days=1),freq = 'd')

    hist_data = pd.DataFrame(index=unique_dates,columns=selected_systems)
    hist_24h_data = pd.DataFrame(index=range(0,23),columns=selected_systems)
    for system,group in (moth_df[['time']]-datetime.timedelta(hours=12)).groupby(moth_df.system):
        hist_data[system] = group.groupby(group.time.dt.date).count()
        hist_24h_data[system] = group.groupby(group.time.dt.hour).count()
    hist_data.fillna(0,inplace = True)
    hist_24h_data.fillna(0,inplace = True)

    heatmap_df = pd.DataFrame(np.zeros((24,len(unique_dates))),index=range(24),columns=(unique_dates.strftime('%Y%m%d_%H%M%S')))
    for date,hours in (moth_df[['time']]-datetime.timedelta(hours=12)).groupby((moth_df.time-datetime.timedelta(hours=12)).dt.date):
        heatmap_df[date.strftime('%Y%m%d_%H%M%S')] = (hours.groupby(hours.time.dt.hour).count())
    heatmap_data = ((heatmap_df.fillna(0)).to_numpy()).T

    return unique_dates,heatmap_data,[],hist_data,hist_24h_data

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

def load_mode_data(unique_dates,heatmap_data,selected_systems,start_date,end_date):
    if not len(unique_dates):
        return [],[]
    systems_str = system_sql_str(selected_systems)
    sql_str = 'SELECT * FROM mode_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=(end_date-start_date).days)
    start_date_str = start_date.strftime('%Y%m%d_%H%M%S')
    sql_str += 'AND start_datetime > "' + start_date_str + '"'
    sql_str += ' ORDER BY "start_datetime"'
    with patsc.open_data_db() as con:
        modes = con.execute(sql_str).fetchall()
        mode_columns = [i[1] for i in con.execute('PRAGMA table_info(mode_records)')]
    start_col = mode_columns.index('start_datetime')
    end_col = mode_columns.index('end_datetime')
    mode_col = mode_columns.index('op_mode')

    modemap_data = heatmap_data.copy()
    modemap_data.fill(-666)
    for mode in modes:
        d_mode_start = datetime.datetime.strptime(mode[start_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        d_mode_end = datetime.datetime.strptime(mode[end_col], '%Y%m%d_%H%M%S')-datetime.timedelta(hours=12)
        if d_mode_end > d_mode_start: #there were some wrong logs. This check can possibly be removed at some point.
            len_hours = math.ceil((d_mode_end-d_mode_start).seconds/3600)
        else:
            continue

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
    return heatmap_data

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def create_heatmap(unique_dates,heatmap_counts,xlabels, selected_heat):
    hover_label = pd.DataFrame(heatmap_counts).astype(int).astype(str)
    hover_label[hover_label=='-1'] = 'NA'
    heatmap_data = np.clip(heatmap_counts, -1, heatmap_max)
    if selected_heat:
        selected_heat = pd.read_json(selected_heat, orient='split')
        for _,cel in selected_heat.iterrows():
            x=cel['x']
            y = (unique_dates.strftime('%d-%m-%Y').tolist()).index(cel['lalaladate'])
            if heatmap_data[y,x]>=0:
                heatmap_data[y,x]=-2

    hm = go.Heatmap(
            x = xlabels,
            y = unique_dates.strftime('%d-%m-%Y'),
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
        title_text='Activity summed per hour',
        clickmode='event+select'
    )
    style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}
    return fig,style

def create_hist(df_hist,unique_dates,system_labels):
    fig = go.Figure()
    fig.update_yaxes(rangemode = "nonnegative")
    cnt = 0
    bar_totals = df_hist.sum(axis=1).astype(int)
    for sys in system_labels.keys():
        hist_data = df_hist[sys]
        sys_str = [system_labels[sys]] * len(hist_data)
        sys_names = [sys] * len(hist_data)
        hist = go.Bar(
            x = unique_dates.strftime('%d-%m-%Y'),
            y = hist_data,
            customdata = np.transpose([sys_str,sys_names,hist_data.astype(int),bar_totals]),
            marker_color = px.colors.qualitative.Vivid[cnt%(len(px.colors.qualitative.Vivid))],
            name = system_labels[sys],
            hovertemplate = '<b>%{customdata[0]}</b><br>Count: %{customdata[2]} / %{customdata[3]}<br><extra></extra>'
        )
        cnt+=1
        fig.add_trace(hist)

    fig.update_layout(
        title_text='Activity summed per day',
        xaxis_title_text='Days',
        yaxis_title_text='Counts',
        barmode='stack',
        clickmode='event+select'
    )
    hist_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}
    return fig,hist_style

def create_24h_hist(hist_24h_data,hour_labels,system_labels):
    fig = go.Figure()
    fig.update_yaxes(rangemode = "nonnegative")
    cnt = 0
    bar_totals = hist_24h_data.sum(axis=1).astype(int)

    for sys in system_labels.keys():
        hist_data = hist_24h_data[sys]
        sys_str = [system_labels[sys]] * len(hist_data)
        sys_names = [sys] * len(hist_data)
        hist = go.Bar(
            x = hour_labels,
            y = hist_data,
            customdata = np.transpose([sys_str,sys_names,hist_data.astype(int),bar_totals]),
            marker_color = px.colors.qualitative.Vivid[cnt%(len(px.colors.qualitative.Vivid))],
            name = system_labels[sys],
            hovertemplate = '<b>%{customdata[0]}</b><br>Count: %{customdata[2]} / %{customdata[3]}<br><extra></extra>'
        )
        cnt+=1
        fig.add_trace(hist)

    fig.update_layout(
        title_text='Activity throughout the day',
        xaxis_title_text='Hours',
        yaxis_title_text='Counts',
        barmode='stack',
        clickmode='event+select'
    )
    hist_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}
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
def create_scatter(moths,system_labels,scatter_x_value,scatter_y_value):
    df_scatter = moths
    df_scatter['system_ids'] = df_scatter['system'].str.replace('pats-proto','').str.replace('pats','').astype(int)
    df_scatter['system_color'] = df_scatter['system'].apply(list(system_labels.keys()).index).astype(int)
    df_scatter['classification_symbol'] = df_scatter['Human_classification'].apply(classification_to_symbol)
    df_scatter['Human_classification'] = df_scatter['Human_classification'].apply(remove_nones_classification)
    df_scatter['video_symbol'] = df_scatter['Video_Filename'].apply(video_available_to_symbol)
    df_scatter['classification_symbol'] = df_scatter['classification_symbol']

    scat_fig = go.Figure()
    scat_fig.update_yaxes({'range':(df_scatter[scatter_y_value].min()-0.005,df_scatter[scatter_y_value].max()+0.005)})
    for sys in system_labels.keys():
        df_scatter_sys = df_scatter[df_scatter['system']==sys]
        for classification in classification_options:
            tmp1 = list(df_scatter_sys['Human_classification']==classification) #which rows have the same classification
            df = df_scatter_sys[tmp1] #retrieve only those rows that have the same classficication
            df = df.fillna('')
            if not df.empty:
                # df['time'].apply(lambda x: x.strftime('%H:%M:%S %d-%m-%Y'))
                df['time_for_humans'] = df['time'].dt.strftime('%H:%M:%S %d-%m-%Y')
                current_labels = [system_labels[sys] for x in df[scatter_x_value]]
                scatter = go.Scattergl(
                    name = system_labels[sys] + ', ' + classification,
                    showlegend=True,
                    x = df[scatter_x_value],
                    y = df[scatter_y_value],
                    mode = 'markers',
                    customdata = np.stack(
                        (df['system'] + '/' + df['Folder'] + '/' + df['Filename'],
                        df['Video_Filename'],
                        df['Human_classification'],
                        current_labels,
                        df['uid'],
                        df['time_for_humans'],
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
                    't: %{customdata[5]}<br>' +
                    'File: %{customdata[0]}<br>' +
                    'Video: %{customdata[1]}<br>' +
                    'Ground truth: %{customdata[2]}<br>' +
                    '<extra></extra>'
                )
                scat_fig.add_trace(scatter)


    scat_fig.update_layout(
        title_text='Selected insects:',
        xaxis_title = scatter_columns[scatter_x_value],
        yaxis_title = scatter_columns[scatter_y_value],
        clickmode='event+select',
        legend_title_text='Legend',
    )
    return scat_fig

def download_log(selected_moth,moth_columns):
    sys_name = selected_moth[moth_columns.index('system')].replace('-proto','')
    log_fn = selected_moth[moth_columns.index('Filename')]
    log_folder = selected_moth[moth_columns.index('Folder')]
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
    return html.A('Download log',href=location,style={'display': 'block'}),{'textAlign':'center','width': '25%','margin':'auto','display': 'block'}
def create_path_plot(target_log_fn):
    df_ilog = pd.read_csv(target_log_fn,delimiter=';')
    target_log_fn = '/' + target_log_fn
    rows_without_tracking = df_ilog[ df_ilog['n_frames_tracking_insect'] == 0 ].index
    df_ilog = df_ilog.drop(rows_without_tracking)
    scatter = go.Scatter3d(
        x = -df_ilog['sposX_insect'],
        y = -df_ilog['sposZ_insect'],
        z = df_ilog['sposY_insect'],
        text = df_ilog['time'],
        mode = 'markers',
        name = 'Flight path',
        hovertemplate = '<b>t= %{text}</b><br>x= %{x}<br>y= %{y}<br>z= %{z}'
    )
    camera_pos = go.Scatter3d(
        x = (0,), y = (0,), z = (0,),
        mode = 'markers',
        marker = dict(
            symbol = 'circle',
            color = 'red'),
        name = 'Camera position',
        hovertemplate = 'Camera position'
    )
    fig = go.Figure(data=[scatter,camera_pos])
    fig.update_layout(
        title_text='Insect flight path',
         scene=dict(
                 aspectmode='data'
         )
    )
    style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}
    return fig,style
def download_video(selected_moth,moth_columns):
    video_fn = selected_moth[moth_columns.index('Video_Filename')]
    sys_name = selected_moth[moth_columns.index('system')].replace('-proto','')

    target_video_mp4_fn = ''
    if video_fn and video_fn != 'NA':
        target_video_mkv_fn = 'static/' + selected_moth[moth_columns.index('Folder')] + '_' + sys_name + '_' + video_fn
        target_video_mp4_fn = target_video_mkv_fn[0:-3] + 'mp4'

        if not os.path.isfile(target_video_mkv_fn) and  not os.path.isfile(target_video_mp4_fn):
            rsync_src = sys_name + ':pats/data/processed/' +selected_moth[moth_columns.index('Folder')] + '/logging/render_' + video_fn
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
    app = dash.Dash(__name__, server=False, url_base_pathname='/pats-c/',title='PATS-C')

    pio.templates.default = 'plotly_dark'

    @app.callback(
        Output('systems_dropdown','value'),
        Input('groups_dropdown','value'))
    def select_system_group(selected_group):
        group_dict = load_groups()
        value = []
        if selected_group:
            for group in selected_group:
                systems = [d[0] for d in group_dict[group]]
                value.extend(systems)
        return value

    def update_selected_heat(clickData_hm,selected_heat):
        if not clickData_hm:
            return None
        else:
            for cel in clickData_hm['points']:
                if cel['z'] == -1:
                    return None # if a black cell is clicked, interpret that as cancelling the whole selection

                x = hour_labels.index(cel['x'])

                unclicked = False
                columns=['x','lalaladate','hour'] #lalaladate -> python bug prevents me from using 'date' https://stackoverflow.com/questions/48369578/prevent-pandas-to-json-from-adding-time-component-to-date-object
                if not selected_heat:
                    selected_heat = pd.DataFrame(columns=columns)
                else:
                    selected_heat = pd.read_json(selected_heat, orient='split')
                for index, _ in selected_heat.iterrows():
                    if cel['z'] == -2:
                        selected_heat = selected_heat.drop(index=index)
                        unclicked = True
                        break

                if not unclicked:
                    df_row= pd.DataFrame([[x,cel['y'],int(hour_labels[x].replace('h',''))]],columns=columns)
                    if selected_heat.empty:
                        selected_heat = df_row
                    else:
                        selected_heat = selected_heat.append(df_row, ignore_index=True)

            selected_heat = selected_heat.to_json(date_format='iso', orient='split')
        return selected_heat

    @app.callback(
        Output('hete_kaart', 'figure'),
        Output('staaf_kaart', 'figure'),
        Output('staaf24h_kaart', 'figure'),
        Output('hete_kaart', 'style'),
        Output('staaf_kaart', 'style'),
        Output('staaf24h_kaart', 'style'),
        Output('selected_heatmap_data', 'children'),
        Output('Loading_animation','style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown','options'))
    def update_ui_hist_and_heat(start_date,end_date,selected_systems,clickData_hm,selected_heat,system_options):
        hm_fig=go.Figure(data=go.Heatmap())
        hist_fig=go.Figure(data=go.Histogram())
        hist24h_fig=go.Figure(data=go.Histogram())
        hm_style={'display': 'none'}
        hist_style={'display': 'none'}
        hist24h_style={'display': 'none'}
        Loading_animation_style={'display': 'block'}
        system_labels = {x['value'] : x['label'] for x in system_options if x['value'] in selected_systems}

        ctx = dash.callback_context
        if ctx.triggered[0]['prop_id'] == 'date_range_picker.value' or ctx.triggered[0]['prop_id'] == 'systems_dropdown.value':
            selected_heat = None
            clickData_hm = None
        selected_systems = remove_unauthoirized_system(selected_systems)
        if start_date and end_date:
            end_date = datetime.datetime.strptime(end_date, '%Y-%m-%d')
            start_date = datetime.datetime.strptime(start_date, '%Y-%m-%d')
        if not selected_systems or not end_date or not start_date or (end_date-start_date).days<1:
            return hm_fig,hist_fig,hist24h_fig,hm_style,hist_style,hist24h_style,selected_heat,Loading_animation_style
        unique_dates,heatmap_data,_,df_hist,hist_24h_data = load_moth_data(selected_systems,start_date,end_date)
        if not len(unique_dates):
            return hm_fig,hist_fig,hist24h_fig,hm_style,hist_style,hist24h_style,selected_heat,Loading_animation_style

        hist_fig,hist_style = create_hist(df_hist,unique_dates,system_labels)
        hist24h_fig,hist24h_style = create_24h_hist(hist_24h_data,hour_labels,system_labels)

        if ctx.triggered[0]['prop_id'] == 'hete_kaart.clickData' or ctx.triggered[0]['prop_id'] == 'classification_dropdown.value':
            selected_heat = update_selected_heat(clickData_hm,selected_heat)

        heatmap_data = load_mode_data(unique_dates,heatmap_data,selected_systems,start_date,end_date) #add mode info to heatmap, which makes the heatmap show when the system was online (color) or offline (black)
        hm_fig,hm_style=create_heatmap(unique_dates,heatmap_data,hour_labels,selected_heat)

        Loading_animation_style = {'display': 'none'}

        return hm_fig,hist_fig,hist24h_fig,hm_style,hist_style,hist24h_style,selected_heat,Loading_animation_style

    @app.callback(
        Output('verstrooide_kaart', 'figure'),
        Output('verstrooide_kaart', 'style'),
        Output('scatter_dropdown_container', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'selectedData'),
        Input('staaf24h_kaart', 'selectedData'),
        Input('scatter_x_dropdown', 'value'),
        Input('scatter_y_dropdown', 'value'),
        State('selected_heatmap_data', 'children'),
        State('systems_dropdown','options'))
    def update_ui_scatter(start_date,end_date,selected_systems,clickData_hm,hist_selected_bars,hist24h_selected_bars,scatter_x_value,scatter_y_value,selected_heat,system_options):
        scat_fig=go.Figure(data=go.Scatter())
        scat_style={'display': 'none'}
        scat_axis_select_style={'display': 'none'}
        if system_options and selected_systems:
            system_labels = {x['value'] : x['label'] for x in system_options if x['value'] in selected_systems}
        else:
            system_labels = None

        ctx = dash.callback_context

        selected_systems = remove_unauthoirized_system(selected_systems)
        if not selected_systems:
            return scat_fig,scat_style,scat_axis_select_style

        if ctx.triggered[0]['prop_id'] == 'staaf_kaart.selectedData' or ctx.triggered[0]['prop_id'] == 'staaf24h_kaart.selectedData' or ctx.triggered[0]['prop_id'] == 'hete_kaart.clickData' or ctx.triggered[0]['prop_id'] == 'scatter_x_dropdown.value' or ctx.triggered[0]['prop_id'] == 'scatter_y_dropdown.value':
            if ctx.triggered[0]['prop_id'] == 'hete_kaart.clickData':
                selected_heat = update_selected_heat(clickData_hm,selected_heat)
            elif ctx.triggered[0]['prop_id'] == 'staaf_kaart.selectedData' or ctx.triggered[0]['prop_id'] == 'staaf24h_kaart.selectedData':
                selected_heat = None
            moths = pd.DataFrame()
            if selected_heat:
                selected_heat = pd.read_json(selected_heat, orient='split')
                for _, hm_cell in selected_heat.iterrows():
                    hour = hm_cell['hour']
                    if hour < 12:
                        hour += 24
                    start_date = datetime.datetime.strptime(hm_cell['lalaladate'],'%d-%m-%Y') + datetime.timedelta(hours=hour)
                    moths = moths.append(load_moth_df(selected_systems,start_date,start_date + datetime.timedelta(hours=1)))
            elif hist_selected_bars:
                for bar in hist_selected_bars['points']:
                    sys = bar['customdata'][1]
                    start_date = datetime.datetime.strptime(bar['x'], '%d-%m-%Y') + datetime.timedelta(hours=12)
                    moths = moths.append(load_moth_df([sys],start_date,start_date + datetime.timedelta(days=1)))
            elif hist24h_selected_bars:
                start_date =  datetime.datetime.strptime(start_date,'%Y-%m-%d')
                end_date =  datetime.datetime.strptime(end_date,'%Y-%m-%d')
                for bar in hist24h_selected_bars['points']:
                    sys = bar['customdata'][1]
                    hour = int(bar['x'].replace('h',''))
                    moths = moths.append(load_moth_of_hour([sys],start_date,end_date,hour))

            if not moths.empty:
                scat_fig = create_scatter(moths,system_labels,scatter_x_value,scatter_y_value)
                scat_axis_select_style={'display': 'table','textAlign':'center','width':'50%','margin':'auto'}
                scat_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}
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
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('systems_dropdown', 'value'),
        Input('hete_kaart', 'clickData'),
        Input('staaf_kaart', 'clickData'),
        Input('staaf24h_kaart', 'clickData'),
        Input('verstrooide_kaart', 'clickData'),
        State('verstrooide_kaart','figure'))
    def update_moth_ui(start_date,end_date,selected_systems,clickData_hm,clickData_hist,clickData_hist24h,clickData_dot,scatter_fig_state):
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

        selected_systems = remove_unauthoirized_system(selected_systems)
        ctx = dash.callback_context
        if not 'selectedpoints' in scatter_fig_state['data'][0] or ctx.triggered[0]['prop_id'] != 'verstrooide_kaart.clickData' or not selected_systems:
            return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style

        sql_str = 'SELECT * FROM moth_records WHERE uid=' + str(clickData_dot['points'][0]['customdata'][4])
        with patsc.open_data_db() as con:
            entry = con.execute(sql_str).fetchall()
        if not len(entry):
            return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style
        selected_moth = entry[0]

        target_log_fn = download_log(selected_moth,moth_columns)
        if not os.path.isfile(target_log_fn):
            return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style
        file_link,file_link_style =file_download_link(target_log_fn)

        if 'Human_classification' in moth_columns:
            classification=selected_moth[moth_columns.index('Human_classification')]
        if not classification:
            classification = classification_options[0]
        classify_style = {'display':'block','textAlign':'center','width':'25%','margin':'auto'}

        path_fig,path_style = create_path_plot(target_log_fn)

        target_video_fn = download_video(selected_moth,moth_columns)
        Loading_animation_style = {'display': 'none'}
        if target_video_fn != '':
            video_style={'display': 'block','margin-left': 'auto','margin-right': 'auto','width': '80%'}

        return target_video_fn,video_style,path_fig,path_style,file_link,file_link_style,selected_moth,classification,classify_style,Loading_animation_style

    @app.callback(
        Output(component_id='classification_hidden', component_property='children'),
        Input('classification_dropdown', 'value'),
        Input('selected_scatter_moth', 'children'))
    def classification_value(selected_classification,selected_moth):
        ctx = dash.callback_context
        if not selected_moth or ctx.triggered[0]['prop_id'] != 'classification_dropdown.value':
            return []

        current_classification = selected_moth[moth_columns.index('Human_classification')]
        if not current_classification:
            current_classification = classification_options[0]
        if current_classification != selected_classification:
            sql_str = 'UPDATE moth_records SET Human_classification="' + selected_classification + '" WHERE uid=' + str(selected_moth[moth_columns.index('uid')])
            with patsc.open_data_db() as con:
                con.execute(sql_str)

            #also save user classifications to a seperate database, because the moth database sometimes needs to be rebuild from the logs/jsons
            with patsc.open_classification_db() as con_class:
                table_exists = con_class.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='classification_records' ''').fetchone()[0]
                if not table_exists:
                    sql_create = 'CREATE TABLE classification_records(uid INTEGER PRIMARY KEY,moth_uid INTEGER,system TEXT,time TEXT,duration REAL,user TEXT,classification TEXT)'
                    con_class.execute(sql_create)
                    con_class.commit()

                username = current_user.username
                sql_insert = 'INSERT INTO classification_records(moth_uid,system,time,duration,user,classification) VALUES(?,?,?,?,?,?)'
                data = [selected_moth[moth_columns.index('uid')],str(selected_moth[moth_columns.index('system')]),str(selected_moth[moth_columns.index('time')]),str(selected_moth[moth_columns.index('duration')]),username,selected_classification]
                con_class.execute(sql_insert, data)
                con_class.commit()

        return selected_classification

    if not os.path.exists(os.path.expanduser('~/patsc/static/')):
        os.makedirs(os.path.expanduser('~/patsc/static/'))

    with patsc.open_data_db() as con:
        moth_columns = [i[1] for i in con.execute('PRAGMA table_info(moth_records)')]
        if 'Human_classification' not in moth_columns:
            con.execute('ALTER TABLE moth_records ADD COLUMN Human_classification TEXT')

            if os.path.exists(patsc.db_classification_path):
                with patsc.open_classification_db() as con_class:
                    classification_columns = [i[1] for i in con_class.execute('PRAGMA table_info(classification_records)')]
                    sql_str = 'SELECT * FROM classification_records'
                    classification_data = con_class.execute(sql_str).fetchall()

                for entry in classification_data:
                    sql_where=' WHERE uid=' + str(entry[classification_columns.index('moth_uid')]) + ' AND time="' + str(entry[classification_columns.index('time')]) + '" AND duration=' + str(entry[classification_columns.index('duration')])
                    sql_str = 'UPDATE moth_records SET Human_classification="' + entry[classification_columns.index('classification')] + '"' + sql_where
                    con.execute(sql_str)
                con.commit()
                print('Re-added cliassification results to main db. Added ' + str(len(classification_data)) + ' classifications.')

    hour_labels = []
    for i in range(0,24):
        hour_labels.append(str((i+12)%24)+'h')

    def make_layout():
        fig_hm = go.Figure(data=go.Heatmap())
        fig_hist=go.Figure(data=go.Histogram())
        fig_hist24h=go.Figure(data=go.Histogram())
        fig_scatter = go.Figure(data=go.Scatter())
        fig_path = go.Figure(data=go.Scatter3d())
        system_options, system_style, group_options, group_value, group_style = init_system_dropdown()
        return html.Div(children=[
            dbc.Navbar([
                dbc.Col(html.H1(children='PATS-C')),
                dbc.Col(html.H1()),
                dbc.Col(
                    dbc.Nav(dbc.NavItem(dbc.NavLink("Sign out", href="/logout", external_link=True)), navbar=True),
                    width="auto",
                ),],
                color="#222",
                dark=True,
            ),
            html.Div([
                html.Div([
                    html.Div('Customer:'),
                    html.Div(dcc.Dropdown(
                        id='groups_dropdown',
                        options=group_options,
                        value=group_value,
                        clearable=True,
                        multi=True,
                        placeholder = 'Select customer'
                    ), className='dash-bootstrap'),
                ], style=group_style),
                html.Div([
                    html.Div('Systems:'),
                    html.Div(dcc.Dropdown(
                        id='systems_dropdown',
                        options=system_options,
                        multi=True,
                        placeholder = 'Select systems'
                    ), className='dash-bootstrap'),
                ], style=system_style),
            ],style={'display': 'block','textAlign':'left','width':'80%','margin':'auto'}),
            html.Div([
                html.Div('Date range: '),
                html.Div(dcc.DatePickerRange(
                        id='date_range_picker',
                        clearable=True,
                        minimum_nights=7,
                        with_portal=True,
                        display_format='DD MMM \'YY',
                        min_date_allowed=datetime.date(2020,9,1),
                        max_date_allowed=datetime.datetime.today(),
                        start_date=(datetime.date.today() - relativedelta(months=1)),
                        end_date=datetime.date.today(),
                        number_of_months_shown=3,
                        start_date_placeholder_text="Start period",
                        reopen_calendar_on_clear=True,
                        end_date_placeholder_text="End period",
                        persistence=True
                    ),style={'display': 'inline-block','width':'30%'},className='dash-bootstrap')],style={'display':'block','textAlign':'left','width':'80%','margin':'auto'}),
            html.Br(),
            html.Div([
                dcc.Loading(
                    children=html.Div([html.Br(),html.Br(),html.Br()],id='Loading_animation',style={'display': 'none'}),
                    type='default'
                ),
                dcc.Graph(id='staaf_kaart',style={'display': 'none','margin-left': 'auto','margin-right': 'auto','textAlign':'center', 'width': '80%'},figure=fig_hist)
            ]),
            html.Div([
                dcc.Graph(id='staaf24h_kaart',style={'display': 'none','margin-left': 'auto','margin-right': 'auto','textAlign':'center', 'width': '80%'},figure=fig_hist24h)
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
                    value='Size',
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

    app.layout = make_layout

    return app
