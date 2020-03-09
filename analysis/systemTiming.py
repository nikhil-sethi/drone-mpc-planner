#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  9 14:34:27 2019

@author: ludwig

The script can take the path to a log-file as argument. If no argument is given, it will try to read the log.csv from the current logging folder.

Commands for the spyder ide to set the location of the plot:
	%matplotlib qt
	%matplotlib inline
"""

import sys
import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np
from io import StringIO
import os
import pwd

FPS = 60

def readnPlot(filepath, data_string=None):
	if(not data_string):
		data = pd.read_csv(filepath, sep=';')
	else:
		data = pd.read_csv(data_string, sep=';')
		
	n_measurements = np.size(data,0)
	time = np.arange(0, n_measurements/FPS, 1/FPS)	
	time = time[0:n_measurements]
	cmap = plt.get_cmap("tab10")
	
	
	if(np.size(data['posX_drone'])):
	
		fig, ([ax1, ax2]) = plt.subplots(2,1, sharex=True)
		fig.suptitle(filepath)
		ax1.set_title('Sample Time')
		ax1.plot(time, data['dt'], color=cmap(0), label='dt')
		ax1.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax1.set_xlabel('Time [s]')
		ax1.set_ylabel('dt [s]')
		ax1.set_ylim([0, 0.3])
		ax1.grid()
		
		ax2.set_title('Drone Inputs')
		ax2.plot(time, data['t_visdat'], label='visdata')
		ax2.plot(time, data['t_trkrs'], label='trkrs')
		ax2.plot(time, data['t_nav'], label='nav')
		ax2.plot(time, data['t_ctrl'], label='ctrl')
		ax2.plot(time, data['t_prdct'], label='prdct')
		ax2.plot(time, data['t_frame'], label='frame')
		ax2.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax2.set_xlabel('Time [s]')
		ax2.set_ylabel('Sample Time [us]')
		ax2.grid()
		
		return data, fig
	

def cleanWhitespaces(filepath):
	file = open(filepath, "r")
	datastring = file.read()
	file.close()
	datastring = datastring.replace(' ', '')
	datastring = StringIO(datastring)
	return datastring

def zerosToNans(data):
#	for i in range(len(data.keys())):
		data['posX_drone'][data['posX_drone']==0] = np.nan

def getUserName():
	return pwd.getpwuid(os.getuid())[0]

if __name__ == "__main__":
	if(len(sys.argv)>=2):
		filepath = str(sys.argv[1])
	else:
		username = getUserName()
		filepath = r'/home/'+username+'/code/pats/pc/build-vscode/logging/log.csv'
		
	try:
		data, fig = readnPlot(filepath)
	except KeyError:
		data_string_io = cleanWhitespaces(filepath)
		data, fig = readnPlot(filepath, data_string_io)
	except:
		print('Unknown error occured.')
	
	fig.subplots_adjust(top=0.9,
		bottom=0.066,
		left=0.049,
		right=0.885,
		hspace=0.178,
		wspace=0.395)
	
	plt.show()
	
	