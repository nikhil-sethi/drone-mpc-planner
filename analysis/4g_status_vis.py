import os
import glob
import re
import datetime
import matplotlib.dates as matDate
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

fourg_logs = os.path.expanduser('~/pats/4g_logs/hortipower')
start_date = datetime.datetime(2021,5,28,18,00)

# fourg_logs = os.path.expanduser('~/pats/4g_logs/Redstar')
# start_date = datetime.datetime(2021,6,9,18,00)

# fourg_logs = os.path.expanduser('~/pats/4g_logs/PCH')
# start_date = datetime.datetime(2021,5,1,21,00)

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

files = natural_sort(glob.glob(fourg_logs + "/*tunnel.log"))

fourg_problems = np.zeros((len(files),3))
systems = np.empty((len(files),1),dtype=object)
for i, file in enumerate(files):
    if os.path.exists(file):
        system = os.path.basename(file).split('_')[0]
        if system.lower().startswith('pats'):
            systems[i] = system
            with open(file,'r') as f:
                for line in f:
                    if 'ERROR' in line:
                        try:
                            if datetime.datetime.strptime(line.split(' - ')[0],"%Y-%m-%d %H:%M:%S,%f") > start_date:
                                fourg_problems[i,0] += 1
                        except Exception as e:
                            print(e)

                pats_file = os.path.join(os.path.dirname(file),system + '_wdt_pats.log')
                if os.path.exists(pats_file):
                    with open(pats_file,'r') as pf:
                        for line in pf:
                            if 'ERROR' in line or 'realsense' in line:
                                if 'realsense' in line:
                                    try:
                                        if datetime.datetime.strptime(line.split(' - ')[0],"%Y-%m-%d %H:%M:%S,%f") > start_date:
                                            fourg_problems[i,2] += 1
                                    except Exception as e:
                                        print(e)
                                else:
                                    try:
                                        if datetime.datetime.strptime(line.split(' - ')[0],"%Y-%m-%d %H:%M:%S,%f") > start_date:
                                            fourg_problems[i,1] += 1
                                    except Exception as e:
                                        print(e)


                else:
                    print(system + ' pats_wdt.log doesnt exsist')
        else:
            print(system + 'file name doesnt start with pats')
    else:
        print(file + 'does not exsist.')


fig, ax = plt.subplots()
ax.bar(systems[:,0,],fourg_problems[:,0],label='Tunnel failure')
ax.bar(systems[:,0,],fourg_problems[:,1],label='Pats failure',bottom=fourg_problems[:,0])
ax.bar(systems[:,0,],fourg_problems[:,2],label='Camera failure',bottom=fourg_problems[:,1]+fourg_problems[:,0])
ax.legend()

plt.show()