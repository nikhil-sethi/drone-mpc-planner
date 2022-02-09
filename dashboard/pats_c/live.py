#!/usr/bin/env python3
import numpy as np
import os
from PIL import Image

import dash
from dash import dcc
from dash import html
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from dash.dependencies import Output, Input, State
import dash_bootstrap_components as dbc
import pats_c.lib.lib_patsc as patsc


def init_dropdowns():
    customer_dict, demo = patsc.load_customers()
    customer_value = None
    customer_style = {'width': '30%', 'display': 'inline-block'}
    sys_style = {'width': '70%', 'display': 'inline-block'}
    customer_options, sys_options = patsc.init_system_and_customer_options(customer_dict, demo)

    if len(customer_dict.keys()) == 1 or demo:
        customer_value = [list(customer_dict)[0]]
        customer_style = {'width': '0%', 'display': 'none'}
        sys_style = {'width': '100%', 'display': 'inline-block'}

    return sys_options, sys_style, customer_options, customer_value, customer_style


def dash_application():
    print("Starting PATS-C live-view!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/live/', title='PATS-C live')

    pio.templates.default = 'plotly_dark'

    @ app.callback(
        Output('systems_dropdown', 'value'),
        Input('customers_dropdown', 'value'))
    def select_system_customer(selected_customer):  # pylint: disable=unused-variable
        customer_dict, demo = patsc.load_customers()
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
        Output('levend_plaatje', 'figure'),
        Output('levend_plaatje', 'style'),
        Output('levend_plaatje_container', 'style'),
        Output('Loading_live_animation', 'style'),
        Input('systems_dropdown', 'value'))
    def update_ui_system_selection(selected_systems):  # pylint: disable=unused-variable
        live_image_fig = go.Figure()
        live_image_style = {'display': 'none'}
        live_image_container_style = {'display': 'none'}
        loading_animation_style = {'display': 'block'}
        if not (len(selected_systems)):
            return live_image_fig, live_image_style, live_image_container_style, loading_animation_style

        images = []
        tot_height = 0
        for sys in selected_systems:
            rsync_src = sys + ':pats/status/monitor_tmp.jpg'
            rsync_target = 'static/' + sys + '_live_image.jpg'
            cmd_rsync = ['rsync --timeout=5 -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" ' + rsync_src + ' ' + rsync_target]
            patsc.execute(cmd_rsync)

            cmd_trigger = 'ssh -o StrictHostKeyChecking=no -T ' + sys + ' touch pats/flags/cc_update_request'
            patsc.execute(cmd_trigger)

            rsync_target = './' + rsync_target
            image_ok = False
            if os.path.exists(rsync_target):
                try:
                    image = Image.open(rsync_target)
                    image_rgb = Image.new("RGB", image.size)
                    image_rgb.paste(image)
                    image_ok = True
                except Exception:
                    pass
            if not image_ok:
                image_rgb = Image.new("RGB", size=(848, 480), color=(20, 20, 20))

            tot_height += image_rgb.height
            images.append(np.array(image_rgb))

        live_image_fig = px.imshow(np.array(images), facet_col=0, facet_col_wrap=1, labels={'facet_col': 'sigma'})
        for i, sys in enumerate(selected_systems):
            live_image_fig.layout.annotations[len(selected_systems) - i - 1]['text'] = sys

        live_image_fig.update_xaxes(showticklabels=False).update_yaxes(showticklabels=False)
        live_image_fig.update_layout(margin={"l": 0, "r": 0, "t": 50, "b": 50})
        live_image_fig.update(layout_coloraxis_showscale=False)
        live_image_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '100%', 'height': str(tot_height) + 'px'}
        live_image_container_style = {'display': 'block', 'margin-left': 'auto', 'margin-right': 'auto', 'width': '80%'}
        return live_image_fig, live_image_style, live_image_container_style, loading_animation_style

    def make_layout():
        system_options, system_style, customer_options, customer_value, customer_style = init_dropdowns()
        live_image_fig = go.Figure()

        return html.Div([
            dbc.Navbar
            ([
                dbc.Col(html.H1(children='PATS-C Live')),
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
            html.Br(),
            dcc.Loading(
                children=html.Div([html.Br(), html.Br(), html.Br()], id='Loading_live_animation', style={'display': 'none'}),
                type='default'
            ),
            html.Div
            ([
                dcc.Graph(id='levend_plaatje', responsive=True, style={'display': 'none'}, figure=live_image_fig)
            ], style={'display': 'none'}, id='levend_plaatje_container'),
            html.Br()
        ])

    app.layout = make_layout

    return app
