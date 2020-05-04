#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

def calc_stat_varibales(eval_criteria_data):
	criteria_stats =  {}
	for key in eval_criteria_data:
		if(isinstance(eval_criteria_data[key],(list)) and len(eval_criteria_data[key])):
			criteria_stats[key] = [len(eval_criteria_data[key]), np.min(eval_criteria_data[key]), np.mean(eval_criteria_data[key]), np.var(eval_criteria_data[key]), np.max(eval_criteria_data[key])]
		elif(isinstance(eval_criteria_data[key],(bool))):
			criteria_stats[key] = eval_criteria_data[key]

	return criteria_stats

def plot_stats(stats, units, mode):
	first_list = True
	first_bool = True
	if(mode=='md'):
		for key in stats:
			if(isinstance(stats[key], (list))):
				if(first_list):
					first_list = False
					print('| Criteria [Unit] | N | Min | Mean | Var | Max |')
					print('|-----------------|---|-----|------|-----|-----|')
				print('|',key,'['+units[key]+'] |','{:.0f}'.format(stats[key][0]),' |','{:.3f}'.format(stats[key][1]),'|','{:.3f}'.format(stats[key][2]),'|','{:.3f}'.format(stats[key][3]),'|','{:.3f}'.format(stats[key][4]),'|')

		for key in stats:
			if(isinstance(stats[key], (bool))):
				if(first_bool and not first_list):
					print('')
				if(first_bool):
					first_bool = False
					print('| Criteria | Result |')
					print('|----------|--------|')
				print('|',key,'['+str(units[key])+'] |',stats[key],'|')
	print('')


if __name__ == "__main__":
	pass