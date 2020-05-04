#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


	if(print_type=='markdown'):
		print('| Step directions | T_first_close [sec] | T_last_far [sec] | mean overshoot [m] | max overshoot [m] |')
		print('|:-------------------:|---------------------|------------------|--------------------|-------------------|')
		print('|x|'+'{:.3f}'.format(t_first_close[x]/n_t_first_close[x])+'|'+'{:.3f}'.format(t_last_large[x]/n_max_overshoot[x])+'|'+'{:.3f}'.format(max_overshoot_mean[x]/n_max_overshoot[x])+'|'+'{:.3f}'.format(max_overshoot[x])+'|')
		print('|y|'+'{:.3f}'.format(t_first_close[y]/n_t_first_close[y])+'|'+'{:.3f}'.format(t_last_large[y]/n_max_overshoot[y])+'|'+'{:.3f}'.format(max_overshoot_mean[y]/n_max_overshoot[y])+'|'+'{:.3f}'.format(max_overshoot[y])+'|')
		print('|z|'+'{:.3f}'.format(t_first_close[z]/n_t_first_close[z])+'|'+'{:.3f}'.format(t_last_large[z]/n_max_overshoot[z])+'|'+'{:.3f}'.format(max_overshoot_mean[z]/n_max_overshoot[z])+'|'+'{:.3f}'.format(max_overshoot[z])+'|')
		
		print('\n|Non-step directions| Mean of max erros [m] | Mean of integrated errors [m] |')
		print('|:-----------------:|-----------------------|-------------------------------|')
		print('|x|'+'{:.3f}'.format(max_err[x]/n_max_err[x])+'|'+'{:.3f}'.format(int_err[x]/duration_int_err[x]/FPS)+'|')
		print('|y|'+'{:.3f}'.format(max_err[y]/n_max_err[y])+'|'+'{:.3f}'.format(int_err[y]/duration_int_err[y]/FPS)+'|')
		print('|z|'+'{:.3f}'.format(max_err[z]/n_max_err[z])+'|'+'{:.3f}'.format(int_err[z]/duration_int_err[z]/FPS)+'|')
	elif('tablesheet'):
		print('T_first_close_x [sec];T_first_close_y [sec];T_first_close_z [sec];T_last_far_x [sec];T_last_far_y [sec];T_last_far_z [sec];mean overshoot_x [m];mean overshoot_y [m];mean overshoot_z [m];max overshoot_x [m];max overshoot_y [m];max overshoot_z [m];;Max_erros_mean_x [m];Max_erros_mean_y [m];Max_erros_mean_z [m];Integrated_errors_mean_x [m;Integrated_errors_mean_y [m]];Integrated_errors_mean_z [m]')
		print('{:.3f}'.format(t_first_close[x]/n_t_first_close[x])+';'+'{:.3f}'.format(t_last_large[x]/n_max_overshoot[x])+';'+'{:.3f}'.format(max_overshoot_mean[x]/n_max_overshoot[x])+';'+'{:.3f}'.format(max_overshoot[x])+';'+'{:.3f}'.format(t_first_close[y]/n_t_first_close[y])+';'+'{:.3f}'.format(t_last_large[y]/n_max_overshoot[y])+';'+'{:.3f}'.format(max_overshoot_mean[y]/n_max_overshoot[y])+';'+'{:.3f}'.format(max_overshoot[y])+';'+'{:.3f}'.format(t_first_close[z]/n_t_first_close[z])+';'+'{:.3f}'.format(t_last_large[z]/n_max_overshoot[z])+';'+'{:.3f}'.format(max_overshoot_mean[z]/n_max_overshoot[z])+';'+'{:.3f}'.format(max_overshoot[z])+';;'+'{:.3f}'.format(max_err[x]/n_max_err[x])+';'+'{:.3f}'.format(int_err[x]/duration_int_err[x]/FPS)+';'+'{:.3f}'.format(max_err[y]/n_max_err[y])+';'+'{:.3f}'.format(int_err[y]/duration_int_err[y]/FPS)+';'+'{:.3f}'.format(max_err[z]/n_max_err[z])+';'+'{:.3f}'.format(int_err[z]/duration_int_err[z]/FPS))
		
	else:
		print('Characteristics in the direction of the step:')			
		print('Average time first close [sec]:', t_first_close/n_t_first_close)
		print('Average time last far [sec]:', t_last_large/n_max_overshoot)
		print('Average overshoot [m]:', max_overshoot_mean/n_max_overshoot)
		print('Max overshoot [m]:', max_overshoot)
		
		print('\nCharacteristics in the orthogonal direction of the step:')
		print('Average max. error [m]:', max_err/n_max_err)
		print('Average error [m/sample]:', int_err/duration_int_err/FPS)



if __name__ == "__main__":

	# VISUALIZE RESULTS:
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	fig.suptitle('Figure Title')
	ax.set_title('Axes Title')		
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_zlabel('z [m]')
	ax.view_init(elev=105, azim=-90)
	plt.show()

