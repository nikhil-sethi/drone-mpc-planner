#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""."""

import os
import sys
import time
import numpy as np
import casadi
from casadi import SX, DM

sys.path.append("../")
sys.path.append("../casadi_parser")
from planes import n_planes, idx_normal, idx_support, n_plane_parameters
from dronemodel import *
from insectmodel import *
from helper import *
from optimizer_configurations import load_optimizer_settings
from casadi_parser import parse_casadi_to_QP

solvername, opts = load_optimizer_settings()
problem_name = 'intercept_in_planes'

n_samples_intercepting = 2
n_samples_breaking = 2

n_steps_intercepting = n_samples_intercepting - 1
n_steps_breaking = n_samples_breaking - 1

if n_steps_intercepting <= 0 and n_steps_breaking <= 0:
    raise("All stages need to do have at least one step!.")

# Create Optimization Variables:
n_optvars = 0
delta_t_intercepting = SX.sym('delta_t_intercepting')
n_optvars, idx_delta_t_intercepting = update_optvar_index(n_optvars, delta_t_intercepting)

states_intercepting = SX.sym('states_intercepting', n_states, n_samples_intercepting)
n_optvars, idx_states_intercepting = update_optvar_index(n_optvars, states_intercepting)

inputs_intercepting = SX.sym('inputs_intercepting', n_inputs, n_steps_intercepting)
n_optvars, idx_inputs_intercepting = update_optvar_index(n_optvars, inputs_intercepting)

virtual_inputs_intercepting = SX.sym('virtual_inputs_intercepting', n_states, n_steps_intercepting)
n_optvars, idx_virtual_inputs_intercepting = update_optvar_index(n_optvars, virtual_inputs_intercepting)

plane_slacks_intercepting = SX.sym('plane_slacks_intercepting', n_steps_intercepting * n_planes)
n_optvars, idx_plane_slacks_intercepting = update_optvar_index(n_optvars, plane_slacks_intercepting)

insect_states = SX.sym('insect_state', n_insectstates, n_samples_intercepting)
n_optvars, idx_insect_states = update_optvar_index(n_optvars, insect_states)

delta_t_breaking = SX.sym('delta_t_breaking')
n_optvars, idx_delta_t_breaking = update_optvar_index(n_optvars, delta_t_breaking)

states_breaking = SX.sym('states_breaking', n_states, n_samples_breaking)
n_optvars, idx_states_breaking = update_optvar_index(n_optvars, states_breaking)

inputs_breaking = SX.sym('inputs_breaking', n_inputs, n_steps_breaking)
n_optvars, idx_inputs_breaking = update_optvar_index(n_optvars, inputs_breaking)

virtual_inputs_breaking = SX.sym('virtual_inputs_breaking', n_states, n_steps_breaking)
n_optvars, idx_virtual_inputs_breaking = update_optvar_index(n_optvars, virtual_inputs_breaking)

plane_slacks_breaking = SX.sym('plane_slacks_breaking', n_steps_breaking * n_planes)
n_optvars, idx_plane_slacks_breaking = update_optvar_index(n_optvars, plane_slacks_breaking)

interception_slacks = SX.sym('interception', 3, 1)
n_optvars, idx_interception_slacks = update_optvar_index(n_optvars, interception_slacks)

state_transition_slacks = SX.sym('state_transition_slacks', n_states, 1)
n_optvars, idx_state_transition_slacks = update_optvar_index(n_optvars, state_transition_slacks)

# Create parameters:
print("n_planes:", n_planes)
planes = SX.sym('plane', n_plane_parameters, n_planes)

# Create mapping between structured and flattened variables
variables_list = [delta_t_intercepting,
                  states_intercepting,
                  inputs_intercepting,
                  virtual_inputs_intercepting,
                  plane_slacks_intercepting,
                  insect_states,
                  delta_t_breaking,
                  states_breaking,
                  inputs_breaking,
                  virtual_inputs_breaking,
                  plane_slacks_breaking,
                  interception_slacks,
                  state_transition_slacks
                  ]

variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in variables_list])
variables_name = ['delta_t_intercepting', 'states_intercepting', 'inputs_intercepting', 'virtual_inputs_intercepting', 'plane_slacks_intercepting',
                  'insect_states',
                  'delta_t_breaking', 'states_breaking', 'inputs_breaking', 'virtual_inputs_breaking', 'plane_slacks_breaking',
                  'interception_slacks', 'state_transition_slacks']
pack_variables_fn = casadi.Function('pack_variables_fn', variables_list, [variables_flat], variables_name, ['flat'])
unpack_variables_fn = casadi.Function('unpack_variables_fn', [variables_flat], variables_list, ['flat'], variables_name)

key_dronedynamics_intercepting = 'dronedynamics_intercepting'
key_insectdynamics = 'insectdynamics'
key_intercepting_error = 'intercepting_error'
key_dronedynamics_breaking = 'dronedynamics_breaking'
key_transition = 'transition'
key_planes = 'planes'

def export_casadi_defines(n_vars, n_ineqs, ineq_idxs):
    """Export index of optimization parameters for easier CPP integration."""
    with open("intercept_in_planes_index.h", "w") as f:
        f.write("#pragma once\n")
        f.write("#define N_OPTVARS " + str(n_vars) +  "\n")
        f.write("#define N_CONSTRAINTS " + str(n_ineqs) + "\n")
        f.write("#define N_SAMPLES_INTERCEPTING " + str(n_samples_intercepting) + "\n")
        f.write("#define N_STEPS_INTERCEPTING " + str(n_steps_intercepting) + "\n")
        f.write("#define N_SAMPLES_BREAKING " + str(n_samples_breaking) + "\n")
        f.write("#define N_STEPS_BREAKING " + str(n_steps_breaking) + "\n")
        f.write("#define N_PLANES " + str(n_planes) + "\n")
        f.write("#define N_PLANE_PARAMETERS " + str(n_plane_parameters) + "\n")
        f.write("#define N_DRONE_STATES " + str(n_states) + "\n")
        f.write("#define N_DRONE_INPUTS " + str(n_inputs) + "\n")
        f.write("#define N_INSECT_STATES " + str(n_insectstates) + "\n")
        f.write("\n")
        f.write("enum optvar_idx {\n")
        f.write("    dt_intercepting = " + str(idx_delta_t_intercepting[0, 0]) + ",\n")
        f.write("    drone_posx0_intercepting = " + str(idx_states_intercepting[0, 0]) + ",\n")
        f.write("    drone_posy0_intercepting = " + str(idx_states_intercepting[1, 0]) + ",\n")
        f.write("    drone_posz0_intercepting = " + str(idx_states_intercepting[2, 0]) + ",\n")
        f.write("    drone_velx0_intercepting = " + str(idx_states_intercepting[3, 0]) + ",\n")
        f.write("    drone_vely0_intercepting = " + str(idx_states_intercepting[4, 0]) + ",\n")
        f.write("    drone_velz0_intercepting = " + str(idx_states_intercepting[-1, 0]) + ",\n")
        f.write("    drone_posxF_intercepting = " + str(idx_states_intercepting[0, -1]) + ",\n")
        f.write("    drone_posyF_intercepting = " + str(idx_states_intercepting[1, -1]) + ",\n")
        f.write("    drone_poszF_intercepting = " + str(idx_states_intercepting[2, -1]) + ",\n")
        f.write("    drone_velzF_intercepting = " + str(idx_states_intercepting[-1, -1]) + ",\n")
        f.write("    drone_accx0_intercepting = " + str(idx_inputs_intercepting[0, 0]) + ",\n")
        f.write("    drone_accy0_intercepting = " + str(idx_inputs_intercepting[1, 0]) + ",\n")
        f.write("    drone_accz0_intercepting = " + str(idx_inputs_intercepting[2, 0]) + ",\n")
        f.write("    drone_acczF_intercepting = " + str(idx_inputs_intercepting[-1, -1]) + ",\n")
        f.write("    drone_virt_accx0_intercepting = " + str(idx_virtual_inputs_intercepting[0, 0]) + ",\n")
        f.write("    drone_virt_acczF_intercepting = " + str(idx_virtual_inputs_intercepting[-1, -1]) + ",\n")
        f.write("    drone_plane_slack0_intercepting = " + str(idx_plane_slacks_intercepting[0, 0]) + ",\n")
        f.write("    drone_plane_slackF_intercepting = " + str(idx_plane_slacks_intercepting[-1, -1]) + ",\n")
        f.write("    insect_posx0 = " + str(idx_insect_states[0, 0]) + ",\n")
        f.write("    insect_posy0 = " + str(idx_insect_states[1, 0]) + ",\n")
        f.write("    insect_posz0 = " + str(idx_insect_states[2, 0]) + ",\n")
        f.write("    insect_velx0 = " + str(idx_insect_states[3, 0]) + ",\n")
        f.write("    insect_vely0 = " + str(idx_insect_states[4, 0]) + ",\n")
        f.write("    insect_velz0 = " + str(idx_insect_states[5, 0]) + ",\n")
        f.write("    insect_velzF = " + str(idx_insect_states[-1, -1]) + ",\n")
        f.write("    dt_breaking = " + str(idx_delta_t_breaking[0, 0]) + ",\n")
        f.write("    drone_posx0_breaking = " + str(idx_states_breaking[idx_posx, 0]) + ",\n")
        f.write("    drone_velxF_breaking = " + str(idx_states_breaking[idx_velx, -1]) + ",\n")
        f.write("    drone_velzF_breaking = " + str(idx_states_breaking[idx_velz, -1]) + ",\n")
        f.write("    drone_accx0_breaking = " + str(idx_inputs_breaking[0, 0]) + ",\n")
        f.write("    drone_acczF_breaking = " + str(idx_inputs_breaking[-1, -1]) + ",\n")
        f.write("    drone_virt_accx0_breaking = " + str(idx_virtual_inputs_breaking[0, 0]) + ",\n")
        f.write("    drone_virt_acczF_breaking = " + str(idx_virtual_inputs_breaking[-1, -1]) + ",\n")
        f.write("    drone_plane_slack0_breaking = " + str(idx_plane_slacks_breaking[0, 0]) + ",\n")
        f.write("    drone_plane_slackF_breaking = " + str(idx_plane_slacks_breaking[-1, -1]) + ",\n")
        f.write("    interception_slack0 = " + str(idx_interception_slacks[0, 0]) + ",\n")
        f.write("    interception_slackF = " + str(idx_interception_slacks[2, 0]) + ",\n")
        f.write("    state_transition_slack0 = " + str(idx_state_transition_slacks[0, 0]) + ",\n")
        f.write("    state_transition_slackF = " + str(idx_state_transition_slacks[2, 0]) + ",\n")
        f.write("};\n")
        f.write("\n")
        f.write("enum inequality_constraint_idx {\n")
        f.write("    first_plane_constraint = " + str(ineq_idxs[key_planes][0]) + ",\n")
        f.write("    last_plane_constraint = " + str(ineq_idxs[key_planes][-1]) + "\n")
        f.write("};\n")

def def_optimization():
    """Set optimization costfunction and inequalitiy constraints."""
    ineq_idxs = {}
    n_ineq = 0
    # Optimization objective
    ## Its more important to have a valid trajectory then actually an interception!
    ## But the desire to actually hit the insect must be large enough
    objective = delta_t_intercepting[:, :]
    objective += 0.001 * delta_t_breaking[:, :]
    objective += 1e7*casadi.sum1(casadi.sum2(virtual_inputs_intercepting[:, :]**2)) / virtual_inputs_intercepting.size1() /virtual_inputs_intercepting.size2()
    objective += 1e7*casadi.sum1(casadi.sum2(virtual_inputs_breaking[:, :]**2)) / virtual_inputs_breaking.size1() /virtual_inputs_breaking.size2()
    objective += 1e2 * casadi.sum1(casadi.sum2(plane_slacks_intercepting[:,:]**2)) / plane_slacks_intercepting.size1() / plane_slacks_intercepting.size2()
    objective += 1e2 * casadi.sum1(casadi.sum2(plane_slacks_breaking[:,:]**2)) / plane_slacks_breaking.size1() / plane_slacks_breaking.size2()
    objective += 1e2 * casadi.sum1(casadi.sum2(interception_slacks[:, :]**2)) / interception_slacks.size1() / interception_slacks.size2()
    objective += 1e7 * casadi.sum1(casadi.sum2(state_transition_slacks[:, :]**2)) / state_transition_slacks.size1() / state_transition_slacks.size2()


    ## Differential equation constraints
    X0_intercepting = states_intercepting[:, 0:(n_samples_intercepting - 1)]
    X1_intercepting = states_intercepting[:, 1:n_samples_intercepting]
    K1_intercepting = drone_dynamics(X0_intercepting.T, inputs_intercepting.T)
    K2_intercepting = drone_dynamics(X0_intercepting.T + delta_t_intercepting * K1_intercepting, inputs_intercepting.T)
    constraint_dronedynamic_intercepting = X0_intercepting.T + delta_t_intercepting * (K1_intercepting + K2_intercepting) / 2.0 - X1_intercepting.T + virtual_inputs_intercepting[:, :].T
    constraint_dronedynamic_intercepting = casadi.reshape(constraint_dronedynamic_intercepting, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_dronedynamics_intercepting, constraint_dronedynamic_intercepting)

    X0_insect = insect_states[:, 0:(n_samples_intercepting-1)]
    X1_insect = insect_states[:, 1:n_samples_intercepting]
    K1_insect = insect_dynamics(X0_insect.T)
    K2_insect = insect_dynamics(X0_insect.T + delta_t_intercepting * K1_insect)
    constraint_insectdynamic = X0_insect.T + delta_t_intercepting * (K1_insect + K2_insect) / 2.0 - X1_insect.T
    constraint_insectdynamic = casadi.reshape(constraint_insectdynamic, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_insectdynamics, constraint_insectdynamic)

    constraint_intercepting_poserr = states_intercepting[idx_pos, -1] - insect_states[idx_pos, -1] + interception_slacks[:, :]
    constraint_intercepting_poserr = casadi.reshape(constraint_intercepting_poserr, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_intercepting_error, constraint_intercepting_poserr)

    constraint_transition_dronestate = states_intercepting[:, -1] - states_breaking[:, 0] + state_transition_slacks[:, :]
    constraint_transition_dronestate = casadi.reshape(constraint_transition_dronestate, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_transition, constraint_transition_dronestate)

    X0_breaking = states_breaking[:, 0:(n_samples_breaking - 1)]
    X1_breaking = states_breaking[:, 1:n_samples_breaking]
    K1_breaking = drone_dynamics(X0_breaking.T, inputs_breaking.T)
    K2_breaking = drone_dynamics(X0_breaking.T + delta_t_breaking * K1_breaking, inputs_breaking.T)
    constraint_dronedynamic_breaking = X0_breaking.T + delta_t_breaking * (K1_breaking + K2_breaking) / 2.0 - X1_breaking.T + virtual_inputs_breaking[:, :].T
    constraint_dronedynamic_breaking = casadi.reshape(constraint_dronedynamic_breaking, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_dronedynamics_breaking, constraint_dronedynamic_breaking)

    ## Plane constraints for stage I and II:
    constraint_planes_intercepting = SX(n_steps_intercepting * n_planes, 1)
    for k_intercepting in range(n_samples_intercepting - 1):
        for p in range(n_planes):
            constraint_planes_intercepting[p + k_intercepting * n_planes] = casadi.dot(planes[idx_normal, p], states_intercepting[idx_pos, k_intercepting + 1] - planes[idx_support, p])

    constraint_planes_breaking = SX(n_steps_breaking * n_planes, 1)
    for k_breaking in range(n_samples_breaking - 1):
        for p in range(n_planes):
            constraint_planes_breaking[p + k_breaking * n_planes] = casadi.dot(planes[idx_normal, p], states_breaking[idx_pos, k_breaking + 1] - planes[idx_support, p])

    constraint_planes = casadi.vertcat(constraint_planes_intercepting, constraint_planes_breaking)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_planes, constraint_planes)

    ### Run optimization
    constraints = casadi.vertcat(constraint_dronedynamic_intercepting,
                                 constraint_insectdynamic,
                                 constraint_intercepting_poserr,
                                 constraint_transition_dronestate,
                                 constraint_dronedynamic_breaking,
                                 constraint_planes)

    solver = casadi.nlpsol('solver', solvername, {'f':objective, 'x':variables_flat, 'p':planes.reshape((n_plane_parameters*n_planes, 1)), 'g':constraints}, opts)
    solver.generate_dependencies(problem_name+'_optimizer.cpp', {'with_header': False, 'cpp': False})
    os.system("gcc -fPIC -shared -O3 "+problem_name+"_optimizer.cpp -o "+problem_name+"_optimizer.so")

    parse_casadi_to_QP(problem_name, variables_flat, planes.reshape((n_plane_parameters*n_planes, 1)), objective, constraints)
    export_casadi_defines(n_optvars, n_ineq, ineq_idxs)

    return solver, n_ineq, ineq_idxs


def gen_initguess_linear(x0, xinsect):
    """."""
    pos_err = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    vel_err = np.linalg.norm(xinsect[idx_vel] - x0[idx_vel])
    t_intercepting_guess = vel_err / DRONE_MAX_THRUST + sqrt(2 * pos_err / (DRONE_MAX_THRUST));

    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    drone_posEnd = insect_pos0 + t_intercepting_guess * insect_vel0
    u = 2*(drone_posEnd - x0[idx_pos] - t_intercepting_guess * x0[idx_vel]) / t_intercepting_guess
    drone_velEnd = x0[idx_vel] + t_intercepting_guess * u

    # Calc initial guess
    deltat_intercepting_guess = t_intercepting_guess/n_samples_intercepting

    xinterception = np.array([drone_posEnd[0], drone_posEnd[1], drone_posEnd[2], drone_velEnd[0], drone_velEnd[1], drone_velEnd[2]])
    states_intercepting_guess = np.linspace(x0, xinterception, n_samples_intercepting)
    # states_intercepting_guess = add_noise(states_intercepting_guess, 0.05)
    inputs_intercepting_guess = np.linspace(u, u, n_steps_intercepting)
    # inputs_intercepting_guess = add_noise(inputs_intercepting_guess, 1)

    insect_guess = np.zeros([n_samples_intercepting, n_insectstates])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, drone_posEnd, n_samples_intercepting, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_samples_intercepting, axis=1).T

    virtual_inputs_intercepting_guess = np.zeros([n_steps_intercepting, n_states])
    plane_slacks_intercepting_guess = np.zeros([n_steps_intercepting * n_planes])

    t_breaking_guess = 2 * np.linalg.norm(drone_velEnd) / DRONE_MAX_THRUST
    deltat_breaking_guess = t_breaking_guess / n_samples_breaking

    u_stopping = - DRONE_MAX_THRUST * drone_velEnd / np.linalg.norm(drone_velEnd)
    xstoped = np.zeros([6])
    xstoped[idx_pos] = drone_posEnd + drone_velEnd * t_breaking_guess + u_stopping * 0.5 * t_breaking_guess**2
    states_breaking_guess = np.linspace(xinterception, xstoped, n_samples_breaking)
    states_breaking_guess = add_noise(states_breaking_guess, 0.05)
    inputs_breaking_guess = np.linspace(u_stopping, u_stopping, n_steps_breaking)
    # inputs_breaking_guess = add_noise(inputs_breaking_guess, 1)
    virtual_inputs_breaking_guess = np.zeros([n_steps_breaking, n_states])
    plane_slacks_breaking_guess = np.zeros([n_steps_breaking * n_planes])

    init_guess = np.concatenate((np.array([[deltat_intercepting_guess]]),
                                 states_intercepting_guess.reshape((n_states*n_samples_intercepting, 1)),
                                 inputs_intercepting_guess.reshape((n_inputs*n_steps_intercepting, 1)),
                                 virtual_inputs_intercepting_guess.reshape((n_states*n_steps_intercepting, 1)),
                                 plane_slacks_intercepting_guess.reshape((n_planes*n_steps_intercepting, 1)),
                                 insect_guess.reshape((n_insectstates*n_samples_intercepting, 1)),
                                 np.array([[deltat_breaking_guess]]),
                                 states_breaking_guess.reshape((n_states*n_samples_breaking, 1)),
                                 inputs_breaking_guess.reshape((n_inputs*n_steps_breaking, 1)),
                                 virtual_inputs_breaking_guess.reshape((n_states*n_steps_breaking, 1)),
                                 plane_slacks_breaking_guess.reshape((n_planes*n_steps_breaking, 1)),
                                 np.zeros([3, 1]),
                                 np.zeros([6, 1])
                                 ))
    pretty_nparray_print("init_guess", init_guess)
    return init_guess

def results_to_initguess(results):
    """Copy results to next optimization initial guess."""
    init_guess = np.concatenate((results['delta_t_intercepting'],
                                 results['states_intercepting'].reshape((n_states*n_samples_intercepting, 1)),
                                 results['inputs_intercepting'].reshape((n_inputs*n_steps_intercepting, 1)),
                                 results['virtual_inputs_intercepting'].reshape((n_states*n_steps_intercepting, 1)),
                                 results['insect_states'].reshape((n_insectstates*n_samples_intercepting, 1)),
                                 results['delta_t_breaking'],
                                 results['states_breaking'].reshape((n_states*n_samples_breaking, 1)),
                                 results['inputs_breaking'].reshape((n_inputs*n_steps_breaking, 1)),
                                 results['virtual_inputs_breaking'].reshape((n_states*n_steps_breaking, 1)),
                                 results['interception_slacks'].reshape((3,1)),
                                 results['state_transition_slacks'].reshape((n_states, 1))
                                 ))
    return init_guess

def gen_initguess(x0, xinsect):
    """."""
    return gen_initguess_linear(x0, xinsect)


def run_optimization(init_guess, plane_params):
    """."""
    solver, n_ineq, ineq_idxs = def_optimization()

    # Box constraints
    x0 = init_guess[idx_states_intercepting[:, 0]]
    xinsect = init_guess[idx_insect_states[:, 0]]
    lower_bounds = unpack_variables_fn(flat=-float('inf')) # This creates a dict with inf and state, control and delta_t as keys
    upper_bounds = unpack_variables_fn(flat=float('inf'))

    lower_bounds['delta_t_intercepting'][:, :] = 1.0e-6
    lower_bounds['delta_t_breaking'][:, :] = 1.0e-6
    lower_bounds['inputs_intercepting'] = -DRONE_MAX_THRUST
    upper_bounds['inputs_intercepting'] =  DRONE_MAX_THRUST
    lower_bounds['inputs_breaking'] = -DRONE_MAX_THRUST
    upper_bounds['inputs_breaking'] =  DRONE_MAX_THRUST

    ### Initial state
    lower_bounds['states_intercepting'][idx_posx, 0] = x0[idx_posx]
    upper_bounds['states_intercepting'][idx_posx, 0] = x0[idx_posx]
    lower_bounds['states_intercepting'][idx_posy, 0] = x0[idx_posy]
    upper_bounds['states_intercepting'][idx_posy, 0] = x0[idx_posy]
    lower_bounds['states_intercepting'][idx_posz, 0] = x0[idx_posz]
    upper_bounds['states_intercepting'][idx_posz, 0] = x0[idx_posz]
    lower_bounds['states_intercepting'][idx_velx, 0] = x0[idx_velx]
    upper_bounds['states_intercepting'][idx_velx, 0] = x0[idx_velx]
    lower_bounds['states_intercepting'][idx_vely, 0] = x0[idx_vely]
    upper_bounds['states_intercepting'][idx_vely, 0] = x0[idx_vely]
    lower_bounds['states_intercepting'][idx_velz, 0] = x0[idx_velz]
    upper_bounds['states_intercepting'][idx_velz, 0] = x0[idx_velz]

    ### Initial insect state:
    lower_bounds['insect_states'][idx_insectpos, 0] = xinsect[idx_insectpos]
    upper_bounds['insect_states'][idx_insectpos, 0] = xinsect[idx_insectpos]

    lower_bounds['insect_states'][idx_insectvelx, :] = xinsect[idx_insectvelx]
    upper_bounds['insect_states'][idx_insectvelx, :] = xinsect[idx_insectvelx]
    lower_bounds['insect_states'][idx_insectvely, :] = xinsect[idx_insectvely]
    upper_bounds['insect_states'][idx_insectvely, :] = xinsect[idx_insectvely]
    lower_bounds['insect_states'][idx_insectvelz, :] = xinsect[idx_insectvelz]
    upper_bounds['insect_states'][idx_insectvelz, :] = xinsect[idx_insectvelz]

    lower_bounds['states_breaking'][idx_vel, -1] = 0
    upper_bounds['states_breaking'][idx_vel, -1] = 0

    ## Trust regions:
    # pos_tr = 0.05
    # for k in range(1, n_steps):
    #     for dir_idx in range(3):
    #         lower_bounds['states'][dir_idx, k] = init_guess[idx_states[dir_idx, k]] - pos_tr
    #         upper_bounds['states'][dir_idx, k] = init_guess[idx_states[dir_idx, k]] + pos_tr

    lbg_ = np.zeros([n_ineq])
    ubg_ = np.zeros([n_ineq])
    ubg_[ineq_idxs[key_planes]] = float("inf")

    # print_optimizer_input(variables_flat, init_guess, lbg_, ubg_, pack_variables_fn(**lower_bounds)['flat'], pack_variables_fn(**upper_bounds)['flat'], plane_params.reshape((n_plane_parameters*n_planes, 1)))

    t = time.time()
    result = solver(x0=init_guess, p=plane_params.reshape((n_plane_parameters*n_planes, 1)), lbg =lbg_, ubg=ubg_,
                    lbx=pack_variables_fn(**lower_bounds)['flat'],
                    ubx=pack_variables_fn(**upper_bounds)['flat'])

    elapsed = time.time() - t
    print("Calculation time:", elapsed, ", in time:", elapsed<1./90)
    results = unpack_variables_fn(flat=result['x'])
    print("lam_g:", result['lam_g'])
    pretty_nparray_print('x_opt', np.array(result['x']))
    return results

def eval_results(results, plane_params, unittest = False):
    """."""
    dt_intercepting = results['delta_t_intercepting'][:, :]
    dt_breaking = results['delta_t_breaking'][:, :]
    drone_pos_trajectory = np.array(results['states_intercepting'][idx_posx:(idx_posz+1), :]).T
    drone_state_trajectory_intercepting = np.array(results['states_intercepting'][idx_posx:(idx_velz+1), :]).T
    drone_input_trajectory_intercepting = np.array(results['inputs_intercepting'][idx_accx:(idx_accz+1), :]).T
    drone_input_trajectory_breaking = np.array(results['inputs_breaking'][idx_accx:(idx_accz+1), :]).T
    insect_pos_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectposz+1), :]).T
    insect_state_trajectory = np.array(results['insect_states'][idx_insectposx:(idx_insectvelz+1), :]).T
    print("-----------------------------------------------")
    # print(results)
    interception_time = dt_intercepting * n_steps_intercepting
    print_constraint_check('Intersection time', interception_time < 2 and interception_time > 0.05, interception_time, 's')
    print('Breaking time:', dt_breaking * n_steps_breaking, 's')
    print('Interception point:', drone_pos_trajectory[-1, :], '(', insect_pos_trajectory[-1, :], ')')

    interception_error = np.linalg.norm(drone_pos_trajectory[-1, :] - insect_pos_trajectory[-1, :])
    print_constraint_check('Interception error', interception_error < 0.1, interception_error, 'm')

    trajectory_slack = np.sum(np.abs(np.array(results['virtual_inputs_intercepting'][:,:]))) + np.sum(np.abs(np.array(results['virtual_inputs_breaking'][:,:])))
    print_constraint_check('Trajectory slack', trajectory_slack<1, trajectory_slack)

    planes_slack = np.sum(np.abs(np.array(results['plane_slacks_intercepting']))) + np.sum(np.abs(np.array(results['plane_slacks_breaking'])))
    print_constraint_check('Plane slack', planes_slack<1, planes_slack)

    print('Attack attitude:', angle_between_vectors(drone_input_trajectory_intercepting[-1, :], insect_pos_trajectory[-2, :] - drone_pos_trajectory[-2, :]) * 180 / np.pi)
    print('Attack direction:', angle_between_vectors(drone_state_trajectory_intercepting[-1, idx_vel], np.array([0, 1, 0])) * 180 / np.pi)
    print('Final drone velocity:', drone_state_trajectory_intercepting[-1, idx_vel], )

    final_drone_speed = np.linalg.norm(results['states_breaking'][idx_vel, -1])
    print_constraint_check("Breaked drone speed", final_drone_speed < 0.1, final_drone_speed, 'm/s')
    print('Final drone velocity condtion check:', drone_state_trajectory_intercepting[-1, idx_vely] / np.linalg.norm(drone_state_trajectory_intercepting[-1, idx_vel]))

    final_drone_position = results['states_breaking'][idx_pos, -1]
    print('Final drone position:', final_drone_position)

    # print_constraint_check("Initial drone pos", np.linalg.norm(drone_pos_trajectory[0, :]) < 0.01, drone_pos_trajectory[0, :])

    print(drone_input_trajectory_intercepting)
    print_constraint_check("Applied acceleration", np.linalg.norm(drone_input_trajectory_intercepting[0, :]) < np.sqrt(3) * DRONE_MAX_THRUST, drone_input_trajectory_intercepting[0, :])
    print_constraint_check("Applied acceleration", np.linalg.norm(drone_input_trajectory_intercepting[0, :]) < np.sqrt(3) * DRONE_MAX_THRUST, np.linalg.norm(drone_input_trajectory_intercepting[0, :]))

    # Simulation test:
    # This is only a quick check (Forward Euler), the integration method of the optimizer is actually more precise
    drone_trajectory_sim = np.zeros([n_samples_intercepting + n_samples_breaking, n_states])

    drone_trajectory_sim[0,:] = drone_state_trajectory_intercepting[0, :]
    for k in range(n_steps_intercepting):
        drone_trajectory_sim[1 + k, idx_vel] = np.array(drone_trajectory_sim[k, idx_vel] + drone_input_trajectory_intercepting[k, :] * dt_intercepting).reshape([1, 3])
        drone_trajectory_sim[1 + k, idx_pos] = np.array(drone_trajectory_sim[k, idx_pos] + drone_trajectory_sim[k + 1, idx_vel] * dt_intercepting).reshape([1, 3])

    for k in range(n_steps_breaking):
        drone_trajectory_sim[n_steps_intercepting + 1 + k, idx_vel] = np.array(drone_trajectory_sim[k, idx_vel] + drone_input_trajectory_breaking[k, :] * dt_breaking).reshape([1, 3])
        drone_trajectory_sim[n_steps_intercepting + 1 + k, idx_pos] = np.array(drone_trajectory_sim[k, idx_pos] + drone_trajectory_sim[k + 1, idx_vel] * dt_breaking).reshape([1, 3])
    print("Simulated interception position: " + str(drone_trajectory_sim[n_samples_intercepting-1, :]))


if __name__ == "__main__":
    if(len(sys.argv)==1):
        print("idx:", [*range(variables_flat.size()[0])])
        print("opt_vars:", variables_flat)
        def_optimization()
    else:
        x0 = np.array([-1, -1, 0, -1, -1, -1])
        xinsect = np.array([0, 0, 0, 0, 0, 0])
        plane_params = np.array([[ 100, 0, 0, -1, 0, 0]
                                 # [-100, 0, 0, 1, 0, 0],
                                 # [0,  100, 0, 0, -1, 0],
                                 # [0, -100, 0, 0, 1, 0],
                                 # [0, 0,  100, 0, 0, -1],
                                 # [0, 0, -100, 0, 0, 1]
                                 ])

        init_guess = gen_initguess(x0, xinsect)
        opti_res = run_optimization(init_guess, plane_params)
        eval_results(opti_res, plane_params)
        opti_res = pack_variables_fn(**opti_res)['flat']
        pretty_nparray_print('xopt', np.array(opti_res))
