#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
import casadi
from casadi import SX, DM

sys.path.append("../")
from dronemodel import *
from insectmodel import *
from helper import *
from optimizer_configurations import load_optimizer_settings

solvername, opts = load_optimizer_settings()

n_samples = 9
n_steps = n_samples - 1

# Create Optimization Variables
n_optvars = 0
delta_t = SX.sym('delta_t')
n_optvars, idx_delta_t = update_optvar_index(n_optvars, delta_t)

states = SX.sym('states', n_states, n_samples)
n_optvars, idx_states = update_optvar_index(n_optvars, states)

inputs = SX.sym('inputs', n_inputs, n_steps)
n_optvars, idx_inputs = update_optvar_index(n_optvars, inputs)

virtual_inputs = SX.sym('virtual_inputs', n_states, n_steps) #compensates for infeasible integration steps
n_optvars, idx_virtual_inputs = update_optvar_index(n_optvars, virtual_inputs)

insect_states = SX.sym('insect_state', n_insectstates, n_samples)
n_optvars, idx_insect_states = update_optvar_index(n_optvars, insect_states)

final_pos_error_slack = SX.sym('final_pos_error_slack', 3, 1)
n_optvars, idx_final_pos_error_slack  = update_optvar_index(n_optvars, final_pos_error_slack)

# Create mapping between structured and flattened variables
variables_list = [delta_t, states, inputs, virtual_inputs, insect_states, final_pos_error_slack]
variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in variables_list])
variables_name = ['delta_t', 'states', 'inputs', 'virtual_inputs', 'insect_states', 'final_pos_error_slack']
pack_variables_fn = casadi.Function('pack_variables_fn', variables_list, [variables_flat], variables_name, ['flat'])
unpack_variables_fn = casadi.Function('unpack_variables_fn', [variables_flat], variables_list, ['flat'], variables_name)

key_dronedynamics = 'dronedynamics'
key_insectdynamics = 'insectdynamics'
key_intercepting_error = 'intercepting_error'
key_final_velocity = 'final_velocity'

def export_casadi_defines(n_vars, n_ineqs, ineq_idxs):
    with open("casadiparameter.h", "w") as f:
        f.write("#pragma once\n")
        f.write("#define N_OPTVARS " + str(n_vars) +  "\n")
        f.write("#define N_CONSTRAINTS " + str(n_ineqs) + "\n")
        f.write("#define N_SAMPLES " + str(n_samples) + "\n")
        f.write("#define N_STEPS " + str(n_steps) + "\n")
        f.write("#define N_DRONE_STATES " + str(n_states) + "\n")
        f.write("#define N_DRONE_INPUTS " + str(n_inputs) + "\n")
        f.write("#define N_INSECT_STATES " + str(n_insectstates) + "\n")
        f.write("\n")
        f.write("enum optvar_idx{\n")
        f.write("    dt = " + str(idx_delta_t[0, 0]) + ",\n")

        f.write("    drone_posx0 = " + str(idx_states[0, 0]) + ",\n")
        f.write("    drone_posy0 = " + str(idx_states[1, 0]) + ",\n")
        f.write("    drone_posz0 = " + str(idx_states[2, 0]) + ",\n")
        f.write("    drone_velx0 = " + str(idx_states[3, 0]) + ",\n")
        f.write("    drone_vely0 = " + str(idx_states[4, 0]) + ",\n")
        f.write("    drone_velz0 = " + str(idx_states[5, 0]) + ",\n")
        f.write("    drone_posxF = " + str(idx_states[0, -1]) + ",\n")
        f.write("    drone_posyF = " + str(idx_states[1, -1]) + ",\n")
        f.write("    drone_poszF = " + str(idx_states[2, -1]) + ",\n")
        f.write("    drone_velzF = " + str(idx_states[-1, -1]) + ",\n")

        f.write("    drone_accx0 = " + str(idx_inputs[0, 0]) + ",\n")
        f.write("    drone_acczF = " + str(idx_inputs[-1, -1]) + ",\n")

        f.write("    drone_virt_accx0 = " + str(idx_virtual_inputs[0, 0]) + ",\n")
        f.write("    drone_virt_acczF = " + str(idx_virtual_inputs[-1, -1]) + ",\n")

        f.write("    insect_posx0 = " + str(idx_insect_states[0, 0]) + ",\n")
        f.write("    insect_posy0 = " + str(idx_insect_states[1, 0]) + ",\n")
        f.write("    insect_posz0 = " + str(idx_insect_states[2, 0]) + ",\n")
        f.write("    insect_velx0 = " + str(idx_insect_states[3, 0]) + ",\n")
        f.write("    insect_vely0 = " + str(idx_insect_states[4, 0]) + ",\n")
        f.write("    insect_velz0 = " + str(idx_insect_states[5, 0]) + ",\n")

        f.write("    insect_velzF = " + str(idx_insect_states[-1, -1]) + ",\n")

        f.write("    final_poserrx_slack = " + str(idx_final_pos_error_slack[0, 0]) + ",\n")
        f.write("    final_poserrz_slack = " + str(idx_final_pos_error_slack[2, 0]) + "\n")

        f.write("};\n")

        f.write("\n")
        f.write("enum inequality_constraint_idx{\n")
        f.write("    final_velocity = " + str(ineq_idxs[key_final_velocity][0]) + "\n")

        f.write("};\n")

def def_optimization():
    """Set optimization costfunction and inequalitiy constraints."""
    ineq_idxs = {}
    n_ineq = 0

    # Optimization objective
    # Its more important to have a valid trajectory then actually an interception!
    # But the desire to actually hit the insect must be large enough
    objective = delta_t[:, :]
    objective += 1e12*casadi.sum1(casadi.sum2(virtual_inputs[:, :]**2)) / virtual_inputs.size1() /virtual_inputs.size2()
    objective += 5e12*casadi.sum1(final_pos_error_slack[:,:]**2) / final_pos_error_slack.size1()

    ## Differential equation constraints
    ### There is no loop here, because it is vectorized.
    X0I = states[:, 0:(n_samples - 1)]
    X1I = states[:, 1:n_samples]
    K1I = drone_dynamics(X0I.T, inputs.T)
    K2I = drone_dynamics(X0I.T + delta_t * K1I, inputs.T)
    constr_dronedynamic = X0I.T + delta_t * (K1I + K2I) / 2.0 - X1I.T + virtual_inputs[:, :].T
    constr_dronedynamic = casadi.reshape(constr_dronedynamic, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_dronedynamics, constr_dronedynamic)

    X0_insect = insect_states[:, 0:(n_samples-1)]
    X1_insect = insect_states[:, 1:n_samples]
    K1_insect = insect_dynamics(X0_insect.T)
    K2_insect = insect_dynamics(X0_insect.T + delta_t * K1_insect)
    constr_insectdynamic = X0_insect.T + delta_t * (K1_insect + K2_insect) / 2.0 - X1_insect.T
    constr_insectdynamic = casadi.reshape(constr_insectdynamic, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_insectdynamics, constr_insectdynamic)

    # Constraint equations:
    ## Position error:
    constr_intercepting_poserr = states[idx_pos, -1] - insect_states[idx_pos, -1] + final_pos_error_slack
    constr_intercepting_poserr = casadi.reshape(constr_intercepting_poserr, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_intercepting_error, constr_intercepting_poserr)

    ## Final direction:
    constr_final_velocity = SX([1,1])
    constr_final_velocity[0] = states[idx_vely, -1]**2 - 100 * states[idx_velx, -1]**2
    constr_final_velocity[1] = states[idx_vely, -1]**2 - 100 * states[idx_velz, -1]**2
    constr_final_velocity = casadi.reshape(constr_final_velocity, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_final_velocity, constr_final_velocity)

    ### Run optimization
    constraints = casadi.vertcat(constr_dronedynamic, constr_insectdynamic, constr_intercepting_poserr, constr_final_velocity)
    solver = casadi.nlpsol('solver', solvername, {'x':variables_flat, 'f':objective, 'g':constraints}, opts)
    solver.generate_dependencies('intercept_optimization_problem.cpp', {'with_header': True, 'cpp': True})
    os.system("gcc -fPIC -shared -O3 intercept_optimization_problem.cpp -o intercept_optimization_problem.so")
    export_casadi_defines(n_optvars, n_ineq, ineq_idxs)

    return solver, n_ineq, ineq_idxs


def gen_initguess_linear(x0, u0, xinsect, uend_guess):
    pos_err = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    vel_err = np.linalg.norm(xinsect[idx_vel] - x0[idx_vel])
    t_guess = vel_err / DRONE_MAX_THRUST + sqrt(2 * pos_err / (DRONE_MAX_THRUST));


    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    drone_velEnd = x0[idx_vel] + t_guess * uend_guess;

    # Calc initial guess
    deltat_guess = t_guess/n_samples
    insect_guess = np.zeros([n_samples, n_insectstates])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, insect_pos0 + n_samples * insect_vel0 * deltat_guess, n_samples, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_samples, axis=1).T

    #
    xend = np.array([insect_guess[-1, 0], insect_guess[-1, 1], insect_guess[-1, 2], 0, np.linalg.norm(drone_velEnd), 0])
    xend_modified = xend - np.array([0, -.1, 0, 0, 0, 0])
    states_guess = np.linspace(x0, xend_modified, n_samples-1)
    states_guess = np.concatenate((states_guess, xend.reshape([1, 6])), axis=0)

    inputs_guess = np.linspace(u0, uend_guess, n_steps-1)
    inputs_guess = np.concatenate((inputs_guess, uend_guess.reshape([1, 3])), axis=0)

    virtualinputs_guess = np.zeros([n_samples-1, n_states])
    final_pos_error_slack = np.zeros([3,1])

    init_guess = np.concatenate((np.array([[deltat_guess]]),
                                 states_guess.reshape((n_states*n_samples, 1)),
                                 inputs_guess.reshape(((n_samples-1)*n_inputs, 1)),
                                 virtualinputs_guess.reshape((n_states*(n_samples-1), 1)),
                                 insect_guess.reshape((n_insectstates*n_samples, 1)),
                                 final_pos_error_slack
                                 ))
    return init_guess

def gen_initguess_spline(x0, u0, xinsect, uend_guess):
    pos_err = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    vel_err = np.linalg.norm(xinsect[idx_vel] - x0[idx_vel])
    t_guess = vel_err / DRONE_MAX_THRUST + sqrt(2 * pos_err / (DRONE_MAX_THRUST));

    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    drone_velEnd = x0[idx_vel] + t_guess * uend_guess;

    # Calc initial guess
    deltat_guess = t_guess/n_samples
    insect_guess = np.zeros([n_samples, n_insectstates])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, insect_pos0 + n_samples * insect_vel0 * deltat_guess, n_samples, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_samples, axis=1).T

    A, B, C, D = first_order_poly_between_two_points(x0[idx_pos], x0[idx_vel], insect_guess[-1, idx_pos], np.array([0, 1, 0]))

    states_guess = np.zeros([n_samples, n_states])
    inputs_guess = np.zeros([n_steps, n_inputs])
    ts = np.linspace(0, 1, n_samples)
    for k, t in enumerate(ts):
        states_guess[k, idx_pos] = A + B*t + C*t**2 + D*t**3
        if k == 0:
            states_guess[k, idx_vel] = x0[idx_vel]
        else:
            states_guess[k, idx_vel] = states_guess[k, idx_pos] - states_guess[k-1, idx_pos]
            inputs_guess[k-1, :] = states_guess[k, idx_vel]

    virtualinputs_guess = np.zeros([n_samples-1, n_states])
    final_pos_error_slack = np.zeros([3,1])

    init_guess = np.concatenate((np.array([[deltat_guess]]),
                                 states_guess.reshape((n_states*n_samples, 1)),
                                 inputs_guess.reshape(((n_samples-1)*n_inputs, 1)),
                                 virtualinputs_guess.reshape((n_states*(n_samples-1), 1)),
                                 insect_guess.reshape((n_insectstates*n_samples, 1)),
                                 final_pos_error_slack
                                 ))
    return init_guess

def results_to_initguess(results):
    """Copy results to next optimization initial guess."""
    init_guess = np.concatenate((results['delta_t'],
                                 results['states'].reshape((n_states*n_samples, 1)),
                                 results['inputs'].reshape((n_steps*n_inputs, 1)),
                                 results['virtual_inputs'].reshape((n_states*n_steps, 1)),
                                 results['insect_states'].reshape((n_insectstates*n_samples, 1)),
                                 results['final_pos_error_slack']
                                 ))
    return init_guess

def gen_initguess(x0, u0, xinsect, uend_guess):
    return gen_initguess_linear(x0, u0, xinsect, uend_guess)


def run_optimization(init_guess):
    solver, n_ineq, ineq_idxs = def_optimization()

    # Box constraints
    x0 = init_guess[idx_states[:, 0]]
    xinsect = init_guess[idx_insect_states[:, 0]]
    lower_bounds = unpack_variables_fn(flat=-float('inf')) # This creates a dict with inf and state, control and delta_t as keys
    upper_bounds = unpack_variables_fn(flat=float('inf'))

    lower_bounds['delta_t'][:, :] = 1.0e-6 # Time must run forwards
    lower_bounds['inputs'][idx_accx, :] = -DRONE_MAX_THRUST
    upper_bounds['inputs'][idx_accx, :] = DRONE_MAX_THRUST
    lower_bounds['inputs'][idx_accy, :] = -DRONE_MAX_THRUST
    upper_bounds['inputs'][idx_accy, :] = DRONE_MAX_THRUST
    lower_bounds['inputs'][idx_accz, :] = -DRONE_MAX_THRUST
    upper_bounds['inputs'][idx_accz, :] = DRONE_MAX_THRUST

    ### Initial state
    lower_bounds['states'][idx_posx, 0] = x0[idx_posx]
    upper_bounds['states'][idx_posx, 0] = x0[idx_posx]
    lower_bounds['states'][idx_posy, 0] = x0[idx_posy]
    upper_bounds['states'][idx_posy, 0] = x0[idx_posy]
    lower_bounds['states'][idx_posz, 0] = x0[idx_posz]
    upper_bounds['states'][idx_posz, 0] = x0[idx_posz]

    lower_bounds['states'][idx_velx, 0] = x0[idx_velx]
    upper_bounds['states'][idx_velx, 0] = x0[idx_velx]
    lower_bounds['states'][idx_vely, 0] = x0[idx_vely]
    upper_bounds['states'][idx_vely, 0] = x0[idx_vely]
    lower_bounds['states'][idx_velz, 0] = x0[idx_velz]
    upper_bounds['states'][idx_velz, 0] = x0[idx_velz]

    lower_bounds['states'][idx_vely, -1] = 0

    ## Trust regions:
    # pos_tr = 0.05
    # for k in range(1, n_steps):
    #     for dir_idx in range(3):
    #         lower_bounds['states'][dir_idx, k] = init_guess[idx_states[dir_idx, k]] - pos_tr
    #         upper_bounds['states'][dir_idx, k] = init_guess[idx_states[dir_idx, k]] + pos_tr


    ### Initial insect state:
    lower_bounds['insect_states'][idx_insectposx, 0] = xinsect[idx_insectposx]
    upper_bounds['insect_states'][idx_insectposx, 0] = xinsect[idx_insectposx]
    lower_bounds['insect_states'][idx_insectposy, 0] = xinsect[idx_insectposy]
    upper_bounds['insect_states'][idx_insectposy, 0] = xinsect[idx_insectposy]
    lower_bounds['insect_states'][idx_insectposz, 0] = xinsect[idx_insectposz]
    upper_bounds['insect_states'][idx_insectposz, 0] = xinsect[idx_insectposz]

    lower_bounds['insect_states'][idx_insectvelx, :] = xinsect[idx_insectvelx]
    upper_bounds['insect_states'][idx_insectvelx, :] = xinsect[idx_insectvelx]
    lower_bounds['insect_states'][idx_insectvely, :] = xinsect[idx_insectvely]
    upper_bounds['insect_states'][idx_insectvely, :] = xinsect[idx_insectvely]
    lower_bounds['insect_states'][idx_insectvelz, :] = xinsect[idx_insectvelz]
    upper_bounds['insect_states'][idx_insectvelz, :] = xinsect[idx_insectvelz]


    lbg_ = np.zeros([n_ineq])
    ubg_ = np.zeros([n_ineq])

    ubg_[ineq_idxs[key_final_velocity]] = float('inf')
    # print_optimizer_input(variables_flat, init_guess, lbg_, ubg_, pack_variables_fn(**lower_bounds)['flat'], pack_variables_fn(**upper_bounds)['flat'])

    t = time.time()
    result = solver(x0=init_guess, lbg =lbg_, ubg=ubg_,
                    lbx=pack_variables_fn(**lower_bounds)['flat'],
                    ubx=pack_variables_fn(**upper_bounds)['flat'])

    elapsed = time.time() - t
    print("Calculation time:", elapsed, ", in time:", elapsed<1./90)
    results = unpack_variables_fn(flat=result['x'])
    return results

def eval_results(results):
    """."""
    dt = results['delta_t'][:, :]
    drone_pos_trajectory = np.array(results['states'][idx_posx:(idx_posz+1), :]).T
    drone_state_trajectory = np.array(results['states'][idx_posx:(idx_velz+1), :]).T
    drone_input_trajectory = np.array(results['inputs'][idx_accx:(idx_accz+1), :]).T
    insect_pos_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectposz+1), :]).T
    insect_state_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectvelz+1), :]).T
    print("-----------------------------------------------")
    # print(results)
    print('Required intersection time:', results['delta_t'][:, :] * (n_samples - 1), 's')
    print('Interception point:', drone_pos_trajectory[-1, :], '(', insect_pos_trajectory[-1, :], ')')
    print('Interception error:', np.linalg.norm(drone_pos_trajectory[-1, :] - insect_pos_trajectory[-1, :]))
    print('Final_pos_error_slack:', results['final_pos_error_slack'])
    print('Trajectory slack:', np.sum(results['virtual_inputs']))

    print('Attack attitude:', angle_between_vectors(drone_input_trajectory[-1, :], insect_pos_trajectory[-2, :] - drone_pos_trajectory[-2, :]) * 180 / np.pi)
    print('Attack direction:', angle_between_vectors(drone_state_trajectory[-1, idx_vel], np.array([0, 1, 0])) * 180 / np.pi)
    print('Final drone velocity:', drone_state_trajectory[-1, idx_vel], )
    print('Final drone velocity condtion check:', drone_state_trajectory[-1, idx_vely] / np.linalg.norm(drone_state_trajectory[-1, idx_vel]))

if __name__ == "__main__":
    def_optimization()
