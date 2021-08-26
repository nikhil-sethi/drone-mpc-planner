#!/usr/bin/env python3
import sqlite3
import os
import pandas as pd

import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go

db_path = os.path.expanduser('~/patsc/db/pats.db')
conn = sqlite3.connect(db_path)
cur = conn.cursor()

columns = [i[1] for i in cur.execute('PRAGMA table_info(moth_records)')]
moths = cur.execute('SELECT * FROM moth_records where time LIKE "2020102________"').fetchall()
df_all = pd.DataFrame.from_records(moths, columns=columns)

simple = True
if (simple):
    fig1 = px.scatter(df_all, x="duration", y="Size", color="system", hover_data=['Video_Filename'], title='Too Many Moths')
    fig1.show()
    fig2 = px.histogram(df_all, x="duration", color="system", title='Duurt lang')
    fig2.show()
else:
    # Showing two plots on one page is considerably harder and there are weird constraints.
    # For instance, we can only have one legend for all the plots in a subplot. Handling of colors is a disaster really.
    # For product we can use html divs though, so then this way does not really seem that useful
    # Keeping it for now because I spend the better part of a day on it.
    distinct_cols = px.colors.qualitative.Alphabet
    distinct_cols += distinct_cols  # hack because we have too many systems
    system_ids = df_all['system'].str.replace('pats-proto', '').astype(int)
    df_all['system_ids'] = system_ids

    scatter1 = go.Scattergl(
        x=df_all['duration'],
        y=df_all['Size'],
        mode='markers',
        marker=dict(
            color=df_all['system_ids'],
            colorscale=distinct_cols
        ),
        hovertemplate="<b>System %{marker.color}</b><br><br>" +
        "Size: %{y}<br>" +
        "Duration: %{x}<br>"
    )

    moths = cur.execute('SELECT * FROM moth_records where system="pats-proto13" AND time LIKE "2020102________"').fetchall()
    df1 = pd.DataFrame.from_records(moths, columns=columns)
    moths = cur.execute('SELECT * FROM moth_records where system="pats-proto17" AND time LIKE "2020102________"').fetchall()
    df2 = pd.DataFrame.from_records(moths, columns=columns)

    hist1 = go.Histogram(
        x=df1['duration'],
        name=df2['system'][0],
        opacity=0.75,
        marker=dict(
            color='green'
        )
    )
    hist2 = go.Histogram(
        x=df2['duration'],
        name=df2['system'][0],
        opacity=0.75,
        marker=dict(
            color='red'
        )
    )
    fig = make_subplots(rows=2, cols=1, subplot_titles=('Too Many Moth', 'Duurt lang'))
    fig.add_trace(scatter1, row=1, col=1)
    fig.add_trace(hist1, row=2, col=1)
    fig.append_trace(hist2, 2, 1)

    fig['layout']['xaxis']['title'] = 'Duration'
    fig['layout']['xaxis2']['title'] = 'Duration'
    fig['layout']['yaxis']['title'] = 'Szie'
    fig['layout']['yaxis2']['title'] = '# detections'

    fig.show()
