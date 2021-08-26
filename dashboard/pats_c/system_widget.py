import pandas as pd
import dash
import dash_table
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Output, Input
import dash_bootstrap_components as dbc
from flask_login import current_user
from pats_c.lib.lib_patsc import open_systems_db


def load_user_data():
    con = open_systems_db()
    sql_str = '''SELECT user_id,name FROM users ORDER BY name COLLATE NOCASE'''
    user_data = pd.read_sql_query(sql_str, con)
    user_data['customers'] = ''
    for id in user_data['user_id']:
        cur = con.cursor()
        customers_sql = '''SELECT customers.name FROM customers
                        JOIN user_customer_connection ON customers.customer_id = user_customer_connection.customer_id
                        WHERE user_customer_connection.user_id = :id
                        ORDER BY customers.name COLLATE NOCASE''', {'id': id}
        customers_data = cur.execute(*customers_sql).fetchall()
        customers_str = ''
        for d in customers_data:
            customers_str += d[0] + ', '
        user_data.loc[user_data.user_id == id, 'customers'] = customers_str[:-2]
    return user_data


def load_customer_data():
    con = open_systems_db()
    sql_str = '''SELECT * FROM customers ORDER BY name COLLATE NOCASE'''
    customer_data = pd.read_sql_query(sql_str, con)
    return customer_data


def load_system_data():
    con = open_systems_db()
    sql_str = '''SELECT system_id,systems.description,customers.name AS 'customer',location,customers.name,version,NUC,realsense,multimodule,LED,active,wifi,LG,maintenance
    FROM systems JOIN customers on customers.customer_id = systems.customer_id ORDER BY system_id'''
    system_data = pd.read_sql_query(sql_str, con)
    return system_data


def dash_application():
    print("Starting system-widget dash application!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/sys-adm/', title='sys-adm')
    tables = ['systems', 'users', 'customers']

    @app.callback(
        Output('table_holder', 'children'),
        Output('table_holder', 'style'),
        Input('table_dropdown', 'value'))
    def select_table(choosen_table):  # pylint: disable=unused-variable
        data = None
        if not isinstance(choosen_table, list):
            choosen_table = [choosen_table]
        if 'systems' in choosen_table:
            data = load_system_data()
        elif 'users' in choosen_table:
            data = load_user_data()
        elif 'customers' in choosen_table:
            data = load_customer_data()

        if isinstance(data, pd.DataFrame):
            columns = [{'name': column, 'id': column} for column in data.columns]
            data = data.to_dict('records')
            table = dash_table.DataTable(
                id='table',
                columns=columns,
                filter_action='native',
                sort_action='native',
                data=data,
                style_table={'overflowX': 'auto', 'overflowY': 'auto'},
                # fixed_rows={'headers': True}, fixed rows gives bugs atm, should be used if dash is updated
                css=[{'selector': 'table', 'rule': 'width: 100%;'}, {'selector': '.table, .dash-spreadsheet.dash-freeze-top, .dash-spreadsheet, .dash-virtualized, .dash-spreadsheet-inner', 'rule': 'max-height: calc(100vh - 250px);'}],
                style_cell={'backgroundColor': 'rgb(50, 50, 50)', 'color': 'white', 'padding': '5px', 'padding-right': '15px'},
                style_filter={'backgroundColor': 'white', 'color': 'white'},
                style_cell_conditional=[
                    {
                        'if': {'column_id': 'customers'},
                        'textAlign': 'left'
                    }
                ],
            )

            return table, {'width': '75%', 'display': 'block', 'margin': 'auto'}
        else:
            return dash_table.DataTable(), {'width': '75%', 'display': 'None', 'margin': 'auto'}

    def make_layout():
        if current_user:
            if current_user.is_authenticated:
                username = current_user.username
                if username.lower() == 'bram' or username.lower() == 'kevin' or username.lower() == 'sjoerd' or username.lower() == 'jorn':
                    return html.Div(children=[
                        dbc.Navbar([
                            dbc.Col(html.H1(children='PATS-C')),
                            dbc.Col(html.H1()),
                            dbc.Col(
                                dbc.Nav(dbc.NavItem(dbc.NavLink("Sign out", href="/logout", external_link=True)), navbar=True),
                                width="auto",
                            )], color="#222", dark=True
                        ),
                        html.Div([
                            html.Div([
                                dcc.Dropdown(
                                    id='table_dropdown',
                                    options=[{'label': table, 'value': table} for table in tables]
                                )
                            ], className='dash-bootstrap', style={'width': '50%', 'display': 'inline-block', 'float': 'left'}),
                            html.Div([
                                dbc.Button('edit', id='edit-button', className='ml-1 mr-1', color='dark', block=True, disabled=True)
                            ], style={'width': '25%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('save', id='save-button', className='ml-2 mr-1', color='dark', block=True, disabled=True)
                            ], style={'width': '25%', 'display': 'inline-block'})
                        ], style={'display': 'block', 'textAlign': 'center', 'width': '50%', 'margin': 'auto'}),
                        html.Div([
                            dash_table.DataTable(),
                        ], id='table_holder', className='dash-bootstrap', style={'width': '75%', 'display': 'block', 'margin': 'auto'})
                    ])
                else:
                    print('Illegal user!')
        return html.Div([])

    app.layout = make_layout

    return app
