#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  9 14:34:27 2019

@author: ludwig

@brief Plots some log data related to the control behavior of the drone.

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

FPS = 90

def readnPlot(filepath, data_string_io=None):
	if not data_string_io:
		data = pd.read_csv(filepath, sep=';')
	else:
		data = pd.read_csv(data_string_io, sep=';')

	n_measurements = np.size(data,0)
	time = np.arange(0, n_measurements/FPS, 1/FPS)	
	time = time[0:n_measurements]
	cmap = plt.get_cmap("tab10")


	if np.size(data['posX_drone']):

		fig, ([ax1, ax2], [ax3, ax4]) = plt.subplots(2,2, sharex=True)
		fig.suptitle(filepath)
		ax1.set_title('Drone Position')
		ax1.plot(time, data['posX_drone'], color=cmap(0), label='posX_drone')
		ax1.plot(time, data['posY_drone'], color=cmap(1), label='posY_drone')
		ax1.plot(time, data['posZ_drone'], color=cmap(2), label='posZ_drone')
		#ax1.plot(time, data['modelPosX'], '-.', color=cmap(0), label='modelX')
		#ax1.plot(time, data['modelPosY'], '-.', color=cmap(1), label='modelY')
		#ax1.plot(time, data['modelPosZ'], '-.', color=cmap(2), label='modelZ')
		ax1.plot(time, data['target_pos_x'], '.-', color=cmap(0), label='target_pos_x')
		ax1.plot(time, data['target_pos_y'], '.-', color=cmap(1), label='target_pos_y')
		ax1.plot(time, data['target_pos_z'], '.-', color=cmap(2), label='target_pos_z')
		ax1.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax1.set_xlabel('Time [s]')
		ax1.set_ylabel('Drone Position [m]')
		ax1.set_xlim([time[np.where(data['posZ_drone']!=0)[0][0]-100], time[-1]])
		ax1.grid()

		ax2.set_title('Drone Inputs')
		ax2.plot(time, data['autoRoll'], label='autoRoll')
		ax2.plot(time, data['autoThrottle'], label='autoThrottle')
		ax2.plot(time, data['autoPitch'], label='autoPitch')
		ax2.plot(time, data['autoYaw'], label='autoYaw')
		ax2.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax2.set_xlabel('Time [s]')
		ax2.set_ylabel('Drone Inputs [1]')
		ax2.grid()

		abs_err = np.zeros([len(data['posX_drone'])])
		for i in range(len(data['posX_drone'])):
			abs_err[i] = np.linalg.norm([data['posX_drone'][i]-data['target_pos_x'][i],
										 data['posY_drone'][i]-data['target_pos_y'][i],
										 data['posZ_drone'][i]-data['target_pos_z'][i]])

		ax3.set_title('Position Errors')
		ax3.plot(time, data['target_pos_x']-data['posX_drone'], label='posErrX')
		ax3.plot(time, data['target_pos_y']-data['posY_drone'], label='posErrY')
		ax3.plot(time, data['target_pos_z']-data['posZ_drone'], label='posErrZ')
		ax3.plot(time, abs_err, 'k', label='abs_error')
#		ax3.plot(time, data['yaw'], label='yawErr [?]')
#		ax3.plot(time, data['yaw_smooth'], label='yawErrSmooth [?]')
		ax3.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax3.set_xlabel('Time [s]')
		ax3.set_ylabel('Position Errors [m]')
		ax3.grid()

		# Showing Model error;
		#accepted_error = 0.6
		#abs_modelerr = np.zeros([len(data['posX_drone'])])
		#for i in range(len(data['posX_drone'])):
			#abs_modelerr[i] = np.linalg.norm([data['modelPosX'][i]-data['posX_drone'][i],
										 #data['modelPosY'][i]-data['posY_drone'][i],
										 #data['modelPosZ'][i]-data['posZ_drone'][i]])
#
		#ax4.set_title('Model error')
		#ax4.plot(time, data['modelPosX']-data['posX_drone'], color=cmap(0), label='posErrX')
		#ax4.plot(time, data['modelPosY']-data['posY_drone'], color=cmap(1), label='posErrY')
		#ax4.plot(time, data['modelPosZ']-data['posZ_drone'], color=cmap(2), label='posErrZ')
		#ax4.plot(time, abs_modelerr, 'k', label='abs_error')
		#ax4.plot([time[0], time[-1]], [accepted_error, accepted_error], 'r', label='accepted_error')
		#ax4.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		#ax4.set_xlabel('Time [s]')
		#ax4.set_ylabel('Model erorr [m]')
		#ax4.grid()

		# Showing Drone Speed:
		abs_speed = np.zeros([n_measurements, 1])
		for i in range(n_measurements):
			abs_speed[i,0] = np.linalg.norm([data['svelX_drone'][i], data['svelY_drone'][i], data['svelZ_drone'][i]])
		ax4.set_title('Drone Velocity')
		ax4.plot(time, data['svelX_drone'], label='velX_drone')
		ax4.plot(time, data['svelY_drone'], label='velY_drone')
		ax4.plot(time, data['svelZ_drone'], label='velZ_drone')
		ax4.plot(time, abs_speed, 'k', label='abs_drone_speed')
		ax4.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
		ax4.set_xlabel('Time [s]')
		ax4.set_ylabel('Drone Velocity [m/s]')
		ax4.grid()

		return data, fig


def cleanWhitespaces(filepath):
	file = open(filepath, "r")
	datastring = file.read()
	file.close()
	datastring = datastring.replace(' ', '')
	datastring = StringIO(datastring)
	return datastring


def getUserName():
	return pwd.getpwuid(os.getuid())[0]

if __name__ == "__main__":
	if len(sys.argv) >= 2:
		filepath = str(sys.argv[1])
	else:
		username = getUserName()
		filepath = r'/home/'+username+'/code/pats/pc/build-vscode/logging/log.csv'

	try:
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
