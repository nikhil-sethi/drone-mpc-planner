#!/usr/bin/env python3
"""."""

import numpy as np
import pickle
from dronemodel import *
from insectmodel import *

def save_results_csv_one_stage_optimization(results, filename):
    """."""
    try:
        results['states']
    except:
        results = {'delta_t': results[0],
                    'states': results[1],
                    'inputs': results[2],
                    'insect_states': results[4]}

    dt = results['delta_t'][:, :]
    drone_pos_trajectory = np.array(results['states'][idx_posx:(idx_posz+1), :]).T
    drone_state_trajectory = np.array(results['states'][idx_posx:(idx_velz+1), :]).T
    drone_input_trajectory = np.array(results['inputs'][idx_accx:(idx_accz+1), :]).T
    insect_pos_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectposz+1), :]).T
    insect_state_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectvelz+1), :]).T
    n_samples = np.size(drone_pos_trajectory, 0)
    with open(filename, "w") as f:
        f.write("time;dt;drone_posx;drone_posy;drone_posz;drone_velx;drone_vely;drone_velz;drone_accx;drone_accy;drone_accz; insect_posx;insect_posy;insect_posz;insect_velx;insect_vely;insect_velz;\n")
        t = 0
        for i in range(n_samples-1):
            output_line = str(t) + ";" \
                + str(dt) + ";" \
                + str(drone_pos_trajectory[i, idx_posx]) + ";" \
                + str(drone_pos_trajectory[i, idx_posy]) + ";" \
                + str(drone_pos_trajectory[i, idx_posz]) + ";" \
                + str(drone_state_trajectory[i, idx_velx]) + ";" \
                + str(drone_state_trajectory[i, idx_vely]) + ";" \
                + str(drone_state_trajectory[i, idx_velz]) + ";" \
                + str(drone_input_trajectory[i, idx_accx]) + ";" \
                + str(drone_input_trajectory[i, idx_accy]) + ";" \
                + str(drone_input_trajectory[i, idx_accz]) + ";" \
                + str(insect_pos_trajectory[i, idx_posx]) + ";" \
                + str(insect_pos_trajectory[i, idx_posy]) + ";" \
                + str(insect_pos_trajectory[i, idx_posz]) + ";" \
                + str(insect_state_trajectory[i, idx_velx]) + ";" \
                + str(insect_state_trajectory[i, idx_vely]) + ";" \
                + str(insect_state_trajectory[i, idx_velz]) + ";\n"

            f.write(output_line)
            t += dt

        output_line = str(t) + ";" \
            + str(dt) + ";" \
            + str(drone_pos_trajectory[-1, idx_posx]) + ";" \
            + str(drone_pos_trajectory[-1, idx_posy]) + ";" \
            + str(drone_pos_trajectory[-1, idx_posz]) + ";" \
            + str(drone_state_trajectory[-1, idx_velx]) + ";" \
            + str(drone_state_trajectory[-1, idx_vely]) + ";" \
            + str(drone_state_trajectory[-1, idx_velz]) + ";" \
            + str(drone_input_trajectory[-1, idx_accx]) + ";" \
            + str(drone_input_trajectory[-1, idx_accy]) + ";" \
            + str(drone_input_trajectory[-1, idx_accz]) + ";" \
            + str(insect_pos_trajectory[-1, idx_posx]) + ";" \
            + str(insect_pos_trajectory[-1, idx_posy]) + ";" \
            + str(insect_pos_trajectory[-1, idx_posz]) + ";" \
            + str(insect_state_trajectory[-1, idx_velx]) + ";" \
            + str(insect_state_trajectory[-1, idx_vely]) + ";" \
            + str(insect_state_trajectory[-1, idx_velz]) + ";\n"
        f.write(output_line)


def save_results_csv_two_stage_optimization(results, filename):
    """."""
    try:
        results['states_intercepting']
    except:
        results = {'delta_t_intercepting': results[0],
                    'states_intercepting': results[1],
                    'inputs_intercepting': results[2],
                    'insect_states': results[4],
                    'delta_t_breaking': results[6],
                    'states_breaking': results[7],
                    'inputs_breaking': results[8]}

    dt_intercepting = results['delta_t_intercepting'][:, :]
    dt_breaking = results['delta_t_breaking'][:, :]
    drone_state_trajectory_intercepting = np.array(results['states_intercepting'][idx_posx:(idx_velz+1), :]).T
    drone_input_trajectory_intercepting = np.array(results['inputs_intercepting'][idx_accx:(idx_accz+1), :]).T
    insect_pos_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectposz+1), :]).T
    insect_state_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectvelz+1), :]).T
    n_samples_intercepting = np.size(drone_state_trajectory_intercepting, 0)
    dt_breaking = results['delta_t_breaking'][:, :]
    drone_state_trajectory_breaking = np.array(results['states_breaking'][idx_posx:(idx_velz+1), :]).T
    drone_input_trajectory_breaking = np.array(results['inputs_breaking'][idx_accx:(idx_accz+1), :]).T
    n_samples_breaking = np.size(drone_state_trajectory_breaking, 0)
    with open(filename, "w") as f:
        f.write("time;dt;drone_posx;drone_posy;drone_posz;drone_velx;drone_vely;drone_velz;drone_accx;drone_accy;drone_accz; insect_posx;insect_posy;insect_posz;insect_velx;insect_vely;insect_velz;\n")
        t = 0
        for i in range(n_samples_intercepting - 1):
            output_line = str(t) + ";" \
                + str(dt_intercepting) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_posx]) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_posy]) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_posz]) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_velx]) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_vely]) + ";" \
                + str(drone_state_trajectory_intercepting[i, idx_velz]) + ";" \
                + str(drone_input_trajectory_intercepting[i, idx_accx]) + ";" \
                + str(drone_input_trajectory_intercepting[i, idx_accy]) + ";" \
                + str(drone_input_trajectory_intercepting[i, idx_accz]) + ";" \
                + str(insect_state_trajectory[i, idx_posx]) + ";" \
                + str(insect_state_trajectory[i, idx_posy]) + ";" \
                + str(insect_state_trajectory[i, idx_posz]) + ";" \
                + str(insect_state_trajectory[i, idx_velx]) + ";" \
                + str(insect_state_trajectory[i, idx_vely]) + ";" \
                + str(insect_state_trajectory[i, idx_velz]) + ";\n"

            f.write(output_line)
            t += dt_intercepting

        for i in range(n_samples_breaking - 1):
            output_line = str(t) + ";" \
                + str(dt_breaking) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_posx]) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_posy]) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_posz]) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_velx]) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_vely]) + ";" \
                + str(drone_state_trajectory_breaking[i, idx_velz]) + ";" \
                + str(drone_input_trajectory_breaking[i, idx_accx]) + ";" \
                + str(drone_input_trajectory_breaking[i, idx_accy]) + ";" \
                + str(drone_input_trajectory_breaking[i, idx_accz]) + ";" \
                + str(0) + ";" \
                + str(0) + ";" \
                + str(0) + ";" \
                + str(0) + ";" \
                + str(0) + ";" \
                + str(0) + ";\n"

            f.write(output_line)
            t += dt_breaking

        output_line = str(t) + ";" \
            + str(dt_breaking) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_posx]) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_posy]) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_posz]) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_velx]) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_vely]) + ";" \
            + str(drone_state_trajectory_breaking[-1, idx_velz]) + ";" \
            + str(drone_input_trajectory_breaking[-1, idx_accx]) + ";" \
            + str(drone_input_trajectory_breaking[-1, idx_accy]) + ";" \
            + str(drone_input_trajectory_breaking[-1, idx_accz]) + ";" \
            + str(0) + ";" \
            + str(0) + ";" \
            + str(0) + ";" \
            + str(0) + ";" \
            + str(0) + ";" \
            + str(0) + ";\n"
        f.write(output_line)

def save_results_csv(results, two_stage_optimization=False, filename='result.csv'):
    """."""
    if two_stage_optimization:
        save_results_csv_two_stage_optimization(results, filename)
    else:
        save_results_csv_one_stage_optimization(results, filename)

def load_results():
    """Load results vector from file."""
    return pickle.load(open('results.p', 'rb'))

def save_results(results):
    """Save results vector in a binary file."""
    pickle.dump(results, open('results.p', 'wb'))
