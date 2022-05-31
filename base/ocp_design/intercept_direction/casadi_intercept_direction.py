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

final_angle_error_slack = SX.sym('final_pos_error_slack')
n_optvars, idx_final_angle_error_slack  = update_optvar_index(n_optvars, final_angle_error_slack)

# Create mapping between structured and flattened variables
variables_list = [delta_t, states, inputs, virtual_inputs, insect_states, final_pos_error_slack, final_angle_error_slack]
variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in variables_list])
variables_name = ['delta_t', 'states', 'inputs', 'virtual_inputs', 'insect_states', 'final_pos_error_slack', 'final_angle_error_slack']
pack_variables_fn = casadi.Function('pack_variables_fn', variables_list, [variables_flat], variables_name, ['flat'])
unpack_variables_fn = casadi.Function('unpack_variables_fn', [variables_flat], variables_list, ['flat'], variables_name)

key_dronedynamics = 'dronedynamics'
key_insectdynamics = 'insectdynamics'
key_intercepting_error = 'intercepting_error'
key_intercepting_angle = 'intercepting_angle'

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
        f.write("    final_poserrz_slack = " + str(idx_final_pos_error_slack[2, 0]) + ",\n")

        f.write("    final_angleerr_slack = " + str(idx_final_angle_error_slack[0, 0]) + ",\n")
        f.write("};\n")

        f.write("\n")
        f.write("enum inequality_constraint_idx{\n")
        f.write("    final_angle_constr = " + str(ineq_idxs[key_intercepting_angle][0]) + "\n")

        f.write("};\n")

def def_optimization():
    ineq_idxs = {}
    n_ineq = 0

    # Optimization objective
    objective = delta_t[:, :]
    #velII_scale = 1e-3*np.linspace(1, n_timestepsII, n_timestepsII)**2/n_timestepsII**2
    #objective += casadi.sum1( casadi.sum2(statesII[:, idx_vel]**2) * velII_scale)
    # objective += 1e-4 * casadi.sum1(casadi.sum2(inputs[:,:]**2))
    objective += 1e12*casadi.sum1(casadi.sum2(virtual_inputs[:, :]**2)) / virtual_inputs.size1() /virtual_inputs.size2()
    objective += 5e12*casadi.sum1(final_pos_error_slack[:,:]**2) / final_pos_error_slack.size1()
    objective += 1e14*final_angle_error_slack[:,:]**2
    ## Also minimize input changes to get a smoother trajectory.
    # objective += 5e-3 * casadi.sum1(casadi.sum2((inputsI[0:-2, :] - inputsI[1:-1, :])**2))

    ## Differential equation constraints
    ### There is no loop here, because it is vectorized.
    X0I = states[:, 0:(n_samples - 1)]
    X1I = states[:, 1:n_samples]
    ### Heun's method (some other method, like RK4 could also be used here)
    K1I = drone_dynamics(X0I.T, inputs.T)
    K2I = drone_dynamics(X0I.T + delta_t * K1I, inputs.T)
    constr_dronedynamicI = X0I.T + delta_t * (K1I + K2I) / 2.0 - X1I.T + virtual_inputs[:, :].T
    constr_dronedynamicI = casadi.reshape(constr_dronedynamicI, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_dronedynamics, constr_dronedynamicI)

    X0_insect = insect_states[:, 0:(n_samples-1)]
    X1_insect = insect_states[:, 1:n_samples]
    K1_insect = insect_dynamics(X0_insect.T)
    K2_insect = insect_dynamics(X0_insect.T + delta_t * K1_insect)
    constr_insectdynamic = X0_insect.T + delta_t * (K1_insect + K2_insect) / 2.0 - X1_insect.T
    constr_insectdynamic = casadi.reshape(constr_insectdynamic, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_insectdynamics, constr_insectdynamic)

    # Constraint equations:
    ## Final state stage I:
    ### Position error:
    constr_intercepting_poserr = states[idx_pos, -1] - insect_states[idx_pos, -1] + final_pos_error_slack
    constr_intercepting_poserr = casadi.reshape(constr_intercepting_poserr, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_intercepting_error, constr_intercepting_poserr)

    ### Direction:
    constr_intercepting_anglerr = states[idx_vely, -1] \
                                   / casadi.norm_2(states[idx_vel, -1]) \
                                   + final_angle_error_slack

    constr_intercepting_anglerr = casadi.reshape(constr_intercepting_anglerr, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_intercepting_angle, constr_intercepting_anglerr)

    ### Run optimization
    constraints = casadi.vertcat(constr_dronedynamicI, constr_insectdynamic, constr_intercepting_poserr, constr_intercepting_anglerr)
    solver = casadi.nlpsol('solver', solvername, {'x':variables_flat, 'f':objective, 'g':constraints}, opts)
    solver.generate_dependencies('intercept_optimization_problem.cpp', {'with_header': True, 'cpp': True})
    os.system("gcc -fPIC -shared -O3 intercept_optimization_problem.cpp -o intercept_optimization_problem.so")
    export_casadi_defines(n_optvars, n_ineq, ineq_idxs)

    return solver, n_ineq, ineq_idxs


def gen_initguess(x0, u0, xinsect, uend_guess):
    return gen_initguess_linear(x0, u0, xinsect, uend_guess)

def gen_initguess_linear(x0, u0, xinsect, uend_guess):
    pos_err = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    vel_err = np.linalg.norm(xinsect[idx_vel] - x0[idx_vel])
    t_guess = vel_err / DRONE_MAX_THRUST + sqrt(2 * pos_err / (DRONE_MAX_THRUST));


    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    drone_velEnd = x0[idx_vel] + t_guess * uend_guess;

    # Calc initial guess
    deltatI_guess = t_guess/n_samples
    insect_guess = np.zeros([n_samples, n_insectstates])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, insect_pos0 + n_samples * insect_vel0 * deltatI_guess, n_samples, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_samples, axis=1).T

    #
    xend = np.array([insect_guess[-1, 0], insect_guess[-1, 1], insect_guess[-1, 2], 0, np.linalg.norm(drone_velEnd), 0])
    xend_modified = xend - np.array([0, -.1, 0, 0, 0, 0])
    statesI_guess = np.linspace(x0, xend_modified, n_samples-1)
    statesI_guess = np.concatenate((statesI_guess, xend.reshape([1, 6])), axis=0)
    state_noise = np.random.rand(np.shape(statesI_guess)[0]-1, np.shape(statesI_guess)[1]) * 0.05
    state_noise = np.concatenate((np.zeros([1,6]), state_noise))
    statesI_guess += state_noise

    inputsI_guess = np.linspace(u0, uend_guess, n_steps-1)
    inputsI_guess = np.concatenate((inputsI_guess, uend_guess.reshape([1, 3])), axis=0)
    input_noise = np.random.rand(np.shape(inputsI_guess)[0]-1, np.shape(inputsI_guess)[1]) * 1
    input_noise = np.concatenate((np.zeros([1,3]), input_noise))
    inputsI_guess += input_noise

    virtualinputsI_guess = np.zeros([n_samples-1, n_states])
    final_pos_error_slack = np.zeros([3,1])
    final_angle_error_slack = 0

    init_guess = np.concatenate((np.array([[deltatI_guess]]),
                                 statesI_guess.reshape((n_states*n_samples, 1)),
                                 inputsI_guess.reshape(((n_samples-1)*n_inputs, 1)),
                                 virtualinputsI_guess.reshape((n_states*(n_samples-1), 1)),
                                 insect_guess.reshape((n_insectstates*n_samples, 1)),
                                 final_pos_error_slack,
                                 np.array([[final_angle_error_slack]])
                                 ))
    return init_guess

def print_optimizer_input(init_guess, lbg, ubg, lbx, ubx):
    pretty_nparray_print("idx", np.arange(len(init_guess)))
    print("opt_vars:", variables_flat)
    pretty_nparray_print("init_guess", init_guess)
    print("lower_box_constraint:", lbx)
    print("upper_box_constraint:", ubx)

    pretty_nparray_print("lower_ineq_constraint", lbg)
    pretty_nparray_print("upper_ineq_constraint", ubg)

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

    lbg_[n_ineq-1:n_ineq] = np.cos(5./180*np.pi)
    ubg_[n_ineq-1:n_ineq] = 1
    print_optimizer_input(init_guess, lbg_, ubg_, pack_variables_fn(**lower_bounds)['flat'], pack_variables_fn(**upper_bounds)['flat'])

    t = time.time()
    result = solver(x0=init_guess, lbg =lbg_, ubg=ubg_,
                    lbx=pack_variables_fn(**lower_bounds)['flat'],
                    ubx=pack_variables_fn(**upper_bounds)['flat'])

    elapsed = time.time() - t
    print("Calculation time:", elapsed, ", in time:", elapsed<1./90)
    results = unpack_variables_fn(flat=result['x'])
    return results


if __name__ == "__main__":
    def_optimization()
