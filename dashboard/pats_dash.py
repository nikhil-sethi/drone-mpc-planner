#!/usr/bin/env python3
import socket, json,shutil,subprocess,sqlite3
import datetime,time, argparse,math,pickle, glob, os, re
import numpy as np
import pandas as pd

import dash,flask
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objects as go

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
    conn,cur = open_db()
    sql_str = '''SELECT DISTINCT system FROM mode_records ORDER BY system '''
    cur.execute(sql_str)
    systems = cur.fetchall()
    systems = natural_sort_systems(systems)
    systems = [d[0] for d in systems]
    return systems

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
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result != None:
                n = n+1
                break
            print(stdout_line.decode('utf-8'),end ='')
        popen.stdout.close()

def load_moth_data(selected_systems,selected_dayrange):
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
        return [],[],[],[],[]

    end_date = datetime.datetime.strptime(end_date, "%Y%m%d_%H%M%S")
    d = start_date-datetime.timedelta(hours=12)
    unique_dates = []
    while d <= end_date - datetime.timedelta(hours=12):
        unique_dates.append(d.strftime("%d-%m-%Y"))
        d += datetime.timedelta(days=1)

    heatmap_data = np.zeros((len(unique_dates),24))
    heatmap_data_lists = np.empty((len(unique_dates),24),dtype=object)
    hist_data = np.zeros((len(unique_dates)))
    sql_str = 'SELECT * FROM moth_records WHERE ((duration > 1 AND duration < 10 AND Dist_traveled > 0.15 AND Dist_traveled < 4) OR (Version="1.0" AND duration > 1 AND duration < 10)) AND '
    sql_str += systems_str
    if selected_dayrange > 0:
        sql_str += 'AND time > "' + start_date_str + '"'
    cur.execute(sql_str)
    moths = cur.fetchall()
    cnt = 0
    for moth in moths:
        d = datetime.datetime.strptime(moth[1], "%Y%m%d_%H%M%S")-datetime.timedelta(hours=12)
        day = (d.date() - start_date.date()).days
        hour = d.hour
        heatmap_data[day,hour] += 1
        if heatmap_data_lists[day,hour] == None:
            heatmap_data_lists[day,hour] = [moth]
        else:
            heatmap_data_lists[day,hour].append(moth)
        hist_data[day] +=1

    moth_columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]
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

def load_mode_data(unique_dates,moth_columns,heatmap_data,selected_systems,selected_dayrange):
    systems_str = system_sql_str(selected_systems)
    conn,cur = open_db()

    sql_str = 'SELECT op_mode,start_datetime,end_datetime FROM mode_records WHERE ' + systems_str
    start_date = datetime.datetime.combine(datetime.date.today(), datetime.datetime.min.time())+datetime.timedelta(hours=12) - datetime.timedelta(days=selected_dayrange)
    start_date_str = start_date.strftime("%Y%m%d_%H%M%S")
    sql_str += 'AND start_datetime > "' + start_date_str + '"'
    sql_str += ' ORDER BY "start_datetime"'
    cur.execute(sql_str)
    modes = cur.fetchall()

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
    return heatmap_data,modemap_data

def natural_sort_systems(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key[0][10:])]
    return sorted(l, key=alphanum_key)

def create_heatmap(unique_dates,heatmap_data,xlabels):
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
        title_text='Moth counts per hour',
        clickmode='event+select'
    )
    return fig

def create_hist(selected_systems,selected_dayrange,hist_data,unique_dates):
    hist = go.Bar(
        x = unique_dates,
        y = hist_data,
        marker=dict(
            color='green'
        )
    )

    fig = go.Figure(data=hist)

    fig.update_layout(
        title_text='Moth counts per day',
        xaxis_title_text='Days',
        yaxis_title_text='Counts',
        clickmode='event+select'
    )

    return fig


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dash app that shows Pats monitoring and system analysis')
    parser.add_argument('--db', help="Path to the sql database", default='~/pats.db')
    args = parser.parse_args()
    db_path = os.path.expanduser(args.db)
else:
    db_path = os.path.expanduser('~/pats.db')

if not os.path.exists('static'):
    os.makedirs('static')

#initials empty values for gui:
dateranges = ['Last week', 'Last month', 'Last year']
xlabels = []
for i in range(0,24):
    xlabels.append(str((i+12)%24)+'h')
systems = load_systems()
fig_hm=go.Figure(data=go.Heatmap())
fig_hist=go.Figure(data=go.Histogram())
fig_scatter = go.Figure(data=go.Scatter())

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
server = flask.Flask(__name__)
app = dash.Dash(__name__, external_stylesheets=external_stylesheets, server=server)
app.layout = html.Div(children=[
    html.H1(children='Pats Dash'),
    html.Div([
            html.Div('Select systems:'),
            dcc.Dropdown(
                id='systems_dropdown',
                options=[{"label": system, "value": system} for system in systems],
                value=systems[0],
                multi=True
            ),
        ], style={'width': '49%', 'display': 'inline-block'}),
    html.Div([
            html.Div('Select range:'),
            dcc.Dropdown(
                id='date_range_dropdown',
                options=[{"label": daterange, "value": daterange} for daterange in dateranges],
                value=dateranges[0],
                searchable=False,
                clearable=False
            ),
        ], style={'width': '49%', 'display': 'inline-block'}),
    html.Div([
            dcc.Graph(id='staaf_kaart',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig_hm)
        ]),
    html.Div([
            dcc.Graph(id='hete_kaart',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig_hist)
        ]),
    html.Div([
            dcc.Graph(id='verstrooide_kaart',style={"display": "block","margin-left": "auto","margin-right": "auto","width": "50%"},figure=fig_scatter)
        ]),
    html.Div([
        html.Video(id='insect_video',src='',controls=True)
        ]),
    html.Div(id='intermediate-value', style={'display': 'none'})
])


def selected_dates(daterange_value):
    if daterange_value == 'Last week':
        selected_dayrange = 7
    elif daterange_value == 'Last month':
        selected_dayrange = 31
    elif daterange_value == 'Last year':
        selected_dayrange = 365
    return selected_dayrange

@app.callback(
    dash.dependencies.Output('hete_kaart', 'figure'),
    dash.dependencies.Output('staaf_kaart', 'figure'),
    dash.dependencies.Input('date_range_dropdown', 'value'),
    dash.dependencies.Input('systems_dropdown', 'value')
)
def dropdown_click(daterange_value,selected_systems):

    selected_dayrange = selected_dates(daterange_value)
    if not len(selected_systems):
        return go.Figure(data=go.Scatter()),go.Figure(data=go.Histogram())


    unique_dates,heatmap_data,heatmap_data_lists,hist_data,moth_columns = load_moth_data(selected_systems,selected_dayrange)
    heatmap_data,modemap_data = load_mode_data(unique_dates,moth_columns,heatmap_data,selected_systems,selected_dayrange)
    fig_hm=create_heatmap(unique_dates,heatmap_data,xlabels)
    fig_hist=create_hist(selected_systems,selected_dayrange,hist_data,unique_dates)
    return fig_hm,fig_hist


@app.callback(
    dash.dependencies.Output('verstrooide_kaart', 'figure'),
    dash.dependencies.Input('date_range_dropdown', 'value'),
    dash.dependencies.Input('systems_dropdown', 'value'),
    dash.dependencies.Input('hete_kaart', 'clickData')
)
def heatmap_clickData(daterange_value,selected_systems,clickData):
    if clickData == None or not len(selected_systems):
        return go.Figure(data=go.Scatter())

    selected_dayrange = selected_dates(daterange_value)
    unique_dates,heatmap_data,heatmap_data_lists,hist_data,moth_columns = load_moth_data(selected_systems,selected_dayrange)
    heatmap_data,modemap_data = load_mode_data(unique_dates,moth_columns,heatmap_data,selected_systems,selected_dayrange)

    x = xlabels.index(clickData['points'][0]['x'])
    y = unique_dates.index(clickData['points'][0]['y'])
    moths = heatmap_data_lists[y,x]
    if moths == None:
        return go.Figure(data=go.Scatter())
    df_scatter = pd.DataFrame(moths,columns=moth_columns)

    distinct_cols = px.colors.qualitative.Alphabet
    system_ids = df_scatter['system'].str.replace('pats-proto','').astype(int)
    df_scatter['system_ids'] = system_ids

    scatter = go.Scattergl(
        x = df_scatter['duration'],
        y = df_scatter['Vel_mean'],
        mode='markers',
        marker=dict(
            color=df_scatter['system_ids'],
            colorscale=distinct_cols
        ),
        hovertemplate = "<b>System %{marker.color}</b><br><br>" +
        "Size: %{y}<br>" +
        "Duration: %{x}<br>"
    )
    fig = go.Figure(data=scatter)
    fig.update_layout(
        title_text='Selected moths',
        xaxis_title = 'Duration [s]',
        yaxis_title = 'Velocity [m/s]',
        clickmode='event+select'
    )
    # fig['layout']['xaxis']['title']='Duration'
    # fig['layout']['yaxis']['title']='Velocity [m/s]'
    return fig

@app.callback(
    dash.dependencies.Output('insect_video', 'src'),
    dash.dependencies.Input('date_range_dropdown', 'value'),
    dash.dependencies.Input('systems_dropdown', 'value'),
    dash.dependencies.Input('hete_kaart', 'clickData'),
    dash.dependencies.Input('verstrooide_kaart', 'clickData')
)
def scatter_clickData(daterange_value,selected_systems,clickData_hm,clickData_dot):
    if clickData_dot == None or clickData_hm ==None or not len(selected_systems) or not len(clickData_hm):
        return ''

    selected_dayrange = selected_dates(daterange_value)
    unique_dates,heatmap_data,heatmap_data_lists,hist_data,moth_columns = load_moth_data(selected_systems,selected_dayrange)
    heatmap_data,modemap_data = load_mode_data(unique_dates,moth_columns,heatmap_data,selected_systems,selected_dayrange)

    x = xlabels.index(clickData_hm['points'][0]['x'])
    y = unique_dates.index(clickData_hm['points'][0]['y'])
    moths = heatmap_data_lists[y,x]
    df_scatter = pd.DataFrame(moths,columns=moth_columns)

    point_id = clickData_dot['points'][0]['pointIndex']
    tmp = df_scatter.values[point_id]
    video_fn = tmp[moth_columns.index('Video_Filename')]

    if video_fn == None:
        return ''
    if video_fn == 'NA':
        return ''
    sys_name = tmp[moth_columns.index('system')].replace('-proto','')
    target_video_fn = '/static/' + tmp[moth_columns.index('Folder')] + '_' + sys_name + '_' + video_fn

    if not os.path.isfile(target_video_fn):
        rsync_src = sys_name + ':data/' +tmp[moth_columns.index('Folder')] + '/logging/render_' + video_fn
        cmd = ['rsync -a ' + rsync_src + ' ' + target_video_fn[1:]]
        execute(cmd)

    return target_video_fn


if __name__ == '__main__':
    app.run_server(debug=True)