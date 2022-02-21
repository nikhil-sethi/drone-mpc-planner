#!/usr/bin/env python3
import datetime
from dateutil.relativedelta import relativedelta
import dash
from dash import dcc
from dash import html
import plotly.io as pio
from dash.dependencies import Output, Input
import dash_bootstrap_components as dbc
import pats_c.lib.lib_patsc as patsc


def download_timelapse(sys_name, source, start_date, end_date):
    src_video_fn = '~/pats/timelapse/timelapse_' + source + '_' + start_date + '_' + end_date + '.mp4'
    target_video_mp4_fn = 'static/timelapse_' + sys_name + '_' + source + '_' + start_date + '_' + end_date + '.mp4'
    cmd = ['rsync --timeout=5 -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + sys_name + ':' + src_video_fn + ' ' + target_video_mp4_fn]
    patsc.execute(cmd)
    return target_video_mp4_fn


def init_dropdown():
    customer_dict, demo = patsc.load_customers()
    _, sys_options = patsc.init_system_and_customer_options(customer_dict, demo)
    return sys_options


def dash_application():
    print("Starting PATS-C timelapse!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/timelapse/', title='PATS-C timelapse')

    pio.templates.default = 'plotly_dark'

    @ app.callback(
        Output('timelapse_video', 'src'),
        Output('timelapse_video', 'style'),
        Output('Loading_animation', 'style'),
        Input('date_range_picker', 'start_date'),
        Input('date_range_picker', 'end_date'),
        Input('source_dropdown', 'value'),
        Input('system_dropdown', 'value'))
    def update_timelapse_ui(start_date, end_date, source, selected_system):  # pylint: disable=unused-variable
        loading_animation_style = {'display': 'block'}
        video_style = {'display': 'none'}
        target_video_fn = ''

        if start_date and end_date and source and selected_system:
            end_date = datetime.datetime.strptime(end_date, '%Y-%m-%d')
            start_date = datetime.datetime.strptime(start_date, '%Y-%m-%d')
            end_date = patsc.datetime_to_str(datetime.datetime.combine(end_date, datetime.datetime.min.time()))
            start_date = patsc.datetime_to_str(datetime.datetime.combine(start_date, datetime.datetime.min.time()))
        else:
            return target_video_fn, video_style, loading_animation_style

        sys = selected_system

        video_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
        cmd = 'ssh -o StrictHostKeyChecking=no -T ' + sys + ' python < ./pats_c/timelapse_generator.py - --start ' + start_date + ' --end ' + end_date + ' --prefix ' + source
        patsc.execute(cmd)
        target_video_fn = download_timelapse(sys, source, start_date, end_date)
        target_video_fn = '/' + target_video_fn

        return target_video_fn, video_style, loading_animation_style

    def make_layout():
        system_options = init_dropdown()
        return html.Div([
            dbc.Navbar
            ([
                dbc.Col(html.H1(children='PATS-C Timelapse')),
                dbc.Col(html.H1()),
                dbc.Col(dbc.Nav(dbc.NavItem(dbc.NavLink("PATS-C", href="/pats-c/", external_link=True)), navbar=True), width="auto"),
                dbc.Col(dbc.Nav(dbc.NavItem(dbc.NavLink("Live", href="/live/", external_link=True)), navbar=True), width="auto"),
                dbc.Col(dbc.Nav(dbc.NavItem(dbc.NavLink("Time lapse", href="/timelapse/", external_link=True)), navbar=True), width="auto"),
                dbc.Col(dbc.Nav(dbc.NavItem(dbc.NavLink("Sign out", href="/logout", external_link=True)), navbar=True), width="auto"),
            ], color="#222", dark=True,),
            html.Div
            ([
                html.Div
                ([
                    html.Div('System:'),
                    html.Div(dcc.Dropdown(
                        id='system_dropdown',
                        options=system_options,
                        multi=False,
                        placeholder='Select system'
                    ), className='dash-bootstrap'),
                ], style={'width': '30%', 'display': 'inline-block', 'float': 'left'}),
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
                        number_of_months_shown=1,
                        start_date_placeholder_text="Start period",
                        reopen_calendar_on_clear=True,
                        end_date_placeholder_text="End period",
                        persistence=True
                    ), className='dash-bootstrap')
                ], style={'width': '40%', 'display': 'inline-block', 'float': 'right'}),
                html.Div
                ([
                    html.Div('Source:'),
                    html.Div(dcc.Dropdown(
                        id='source_dropdown',
                        multi=False,
                        placeholder='Select camera source',
                        options=[{'label': 'RGB', 'value': 'rgb'}, {'label': 'IR Left', 'value': 'stereoL'}, {'label': 'IR Right', 'value': 'stereoR'}],
                        value='rgb',
                        clearable=False,
                    ), className='dash-bootstrap'),
                ], style={'width': '30%', 'display': 'inline-block', 'float': 'right'}),
            ], style={'display': 'block', 'width': '80%', 'margin': 'auto'}),
            dcc.Loading(
                children=html.Div(id='Loading_animation', style={'display': 'none'}),
                type='default'
            ),
            html.Br(),
            html.Br(),
            html.Br(),
            html.Br(),
            html.Video(id='timelapse_video', style={'display': 'none'}, controls=True, loop=True, autoPlay=True),
        ])

    app.layout = make_layout

    return app
