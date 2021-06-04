import os
import glob
import re
import datetime
import matplotlib.dates as matDate
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

temp_logs = os.path.expanduser('~/pats/temp_logs/koppert')

for file in glob.glob(temp_logs + "/*.log"):
    if os.path.exists(file):
        system = os.path.basename(file).split('_')[0]
        if system.lower().startswith('pats'):
            with open(file,'r') as f:
                targets = [line for line in f if 'CPU Temperature' in line]
                time_temp_holder = np.empty((2,len(targets)),dtype=object)
                for i,target in enumerate(targets):
                    try:
                        time_temp_holder[0,i] = datetime.datetime.strptime(target.split(' - ')[0],"%Y-%m-%d %H:%M:%S,%f")
                    except Exception as e:
                        print(e)
                        print(f'date not stripped {i}')
                        time_temp_holder[0,i] = datetime.datetime.min
                    try:
                        time_temp_holder[1,i] = float(re.findall(r'\d+\.*\d*',target.split(' - ')[3])[0])
                    except:
                        print(f'temp not stripped {i}')
                        time_temp_holder[1,i] = 0


                dates = matDate.date2num(time_temp_holder[0,:])
                plt.plot_date(dates, time_temp_holder[1,:],'.',label = system, alpha = 0.4)


        else:
            print(system + 'file name doesnt start with pats')
    else:
        print(file + 'does not exsist.')

plt.legend()
plt.grid()
plt.show()