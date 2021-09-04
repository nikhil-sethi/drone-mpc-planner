#!/usr/bin/env python3
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc


def window_filter_monster(insect_df, monster_df):
    monster_times = monster_df['time'].values
    insect_times = insect_df['time'].values
    monster_times_mesh, insect_times_mesh = np.meshgrid(monster_times, insect_times, sparse=True)
    insects_with_monsters = np.sum((monster_times_mesh >= insect_times_mesh - pd.Timedelta(minutes=5)) * (monster_times_mesh <= insect_times_mesh + pd.Timedelta(minutes=5)), axis=1)
    insect_without_monsters = insect_df.iloc[insects_with_monsters == 0]
    return insect_without_monsters


sql_str = '''SELECT system,time,duration,Size,LIA_insect,Monster FROM moth_records
             WHERE LIA_version = "cnn5_noDrop_v1" AND (Dist_traveled > 0.15 AND Dist_traveled < 4
             AND duration > 1 AND duration < 10 OR Monster) ORDER BY Size'''
with patsc.open_data_db() as con:
    data = pd.read_sql(sql_str, con)

data['time'] = pd.to_datetime(data['time'], format='%Y%m%d_%H%M%S')
monster_df = data.loc[data['Monster'] == 1]
data = data.loc[data['Monster'] != 1]
data = window_filter_monster(data, monster_df)

systems_c = {'pats136': 'b', 'pats137': 'r'}
insect_c = {0: 'x', 1: 'o', 2: '*', 3: '>', 4: 's'}

plt.figure(1)
for system in systems_c.keys():
    for insect in insect_c.keys():
        plt.plot(data.loc[(data['system'] == system) & (data['LIA_insect'] == insect), 'duration'], data.loc[(data['system'] == system) & (data['LIA_insect'] == insect), 'Size'], systems_c[system] + insect_c[insect], alpha=0.5, label=system + ' insect ' + str(insect))

plt.legend()
plt.xlabel('duration (s)')
plt.ylabel('Size (m)')
plt.show()
