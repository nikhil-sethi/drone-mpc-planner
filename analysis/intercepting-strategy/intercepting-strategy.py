#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import norm
from scipy import signal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


idx_posx = 0
idx_posy = 1
idx_posz = 2
idx_velx = 3
idx_vely = 4
idx_velz = 5

idx_pos = [0,1,2]
idx_vel = [3,4,5]

def position_control(drone_state, target_pos):
	K = 1.
	T1 = np.array([0.2, 0.2, 0.2])
	T2 = np.array([0.2, 0.2, 0.2])

	yk2 = drone_state[idx_pos]-drone_state[idx_vel]*dt
	yk1 = drone_state[idx_pos]

	yk = target_pos*K*dt**2 + yk1*(2*T1*T2+(T1+T2)*dt) - yk2*T1*T2
	yk /= T1*T2 + (T1+T2)*dt + dt**2

	return yk

was_closechasing = False
def drone_model(drone_state, insect_state):
	target = insect_state[idx_pos]
	
	if(strategy=='pure_position_control'):
		pass
	
	elif(strategy=='current_approach'):
		hor_seperator = norm( np.array([insect_state[[0,2]]]) - np.array([drone_state[[0,2]]]))
		ver_seperator = np.abs(insect_state[idx_posy] - drone_state[idx_posy])

		global was_closechasing
		if(was_closechasing or (np.abs(hor_seperator)<0.6 and ver_seperator<0.8 and ver_seperator>0)):
			was_closechasing = True
			error = insect_state[idx_pos] - drone_state[idx_pos]
			target += error/norm(error)*3.#close chasing
		else:
			target[idx_posy] -= 0.1
			
	elif(strategy=='distance_dependant_shifting'):
		error_threshold = 0.8
		error = insect_state[idx_pos] - drone_state[idx_pos]
		if(norm(error)>error_threshold):
			target[idx_posy] -= 0.01
#			target += 4*insect_state[idx_vel]*dt
		else:
			offset = norm(error)/error_threshold*np.array([0, -.1, 0]) + (1-norm(error))/error_threshold*error/norm(error)*3
			target +=  offset
			
	elif(strategy=='sim_velocity_control'):
		target += 5*insect_state[idx_vel]*dt
		target += 2.*(target-drone_state[idx_pos])
		
			

	prev_dronepos = drone_state[idx_pos]
	drone_state[idx_pos] = position_control(drone_state, target)
	drone_state[idx_vel] = (drone_state[idx_pos] - prev_dronepos)/dt

	return drone_state

if __name__ == "__main__":
	t_sim = 2
	dt = 1./60
	strategy = 'sim_velocity_control'

	drone_state = np.array([0., -2, 0, 0, 0, 0])
	insect_state = np.array([2, 0, 0, -1.0, 0, 0])

	err = insect_state[idx_pos]- drone_state[idx_pos]
	drone_state[idx_vel] = err/np.linalg.norm(err) * 3.

	drone_trajectory = np.zeros([6, int(t_sim/dt)])
	insect_trajectory = np.zeros([6, int(t_sim/dt)])
	error_trajectory = np.zeros([int(t_sim/dt)])

	for tk in np.linspace(0., t_sim, int(t_sim/dt)):
		drone_trajectory [:, int(tk/dt)-1] = drone_state
		insect_trajectory[:, int(tk/dt)-1] = insect_state

		drone_state = drone_model(drone_state, insect_state)
		insect_state[idx_pos] += insect_state[idx_vel]*dt
#		print('Insect:', insect_state, ', drone:', drone_state)
		
		err = norm(insect_state[idx_pos] - drone_state[idx_pos])
		error_trajectory[int(tk/dt)-1] = err

	idx_minerr = np.where(error_trajectory==np.min(error_trajectory[1:]))[0][0]
	print(strategy,'Minimal error', error_trajectory[idx_minerr+1], 'at', idx_minerr*dt, 's')

	# VISUALIZE RESULTS:
	fig1 = plt.figure()
	ax1 = fig1.add_subplot(211, projection='3d')
	ax1.plot(insect_trajectory[idx_posx, :-2], insect_trajectory[idx_posy, :-2], insect_trajectory[idx_posz, :-2], label='insect')
	ax1.plot(drone_trajectory[idx_posx, :-2], drone_trajectory[idx_posy, :-2], drone_trajectory[idx_posz, :-2], label='drone')
	fig1.suptitle('Figure Title')
	ax1.set_title('Axes Title')
	ax1.set_xlabel('x [m]')
	ax1.set_ylabel('y [m]')
	ax1.set_zlabel('z [m]')
	ax1.legend()
	ax1.view_init(elev=105, azim=-90)

	ax2 = fig1.add_subplot(212)
	ax2.plot(np.linspace(0., t_sim, int(t_sim/dt)), error_trajectory, label='error')
	ax2.set_xlabel('time [s]')
	ax2.set_ylabel('velocities [m/s]')
	ax2.legend()
	ax2.grid()

	plt.show()
