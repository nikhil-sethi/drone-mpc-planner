#!/usr/bin/env python3

import os
from pathlib import Path
import argparse
import processloglib as lib
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description='Cluster moth vs no moth.')
parser.add_argument('-s', '--source_folder',type=str,default="/home/houjebek/data_check/checked/")
args = parser.parse_args()

source_folder = os.path.expanduser(args.source_folder.strip(" "))

fps = 30
n0 = 0
n1 = 3 * fps #take only the first 3 seconds, because that makes the human classification a  lot faster

data,csv_data = lib.get_dataset(source_folder,n0,n1)

# Plot
colormap = np.array(['#0b559f', '#89bedc'])
categories = data[0][:,0].astype(int)
plt.subplot(2,1,1)
plt.scatter(data[0][:,1],data[0][:,2],c=colormap[categories])

plt.title('Corrcoef')
plt.xlabel('im x')
plt.ylabel('im y')

plt.subplot(2,1,2)
plt.scatter(data[0][:,2],data[0][:,3],c=colormap[categories])
plt.xlabel('avg v [m/s]')
plt.ylabel('travel [px]')

plt.show()


