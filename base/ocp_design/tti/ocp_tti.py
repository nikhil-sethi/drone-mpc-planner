#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import casadi
from casadi import SX

sys.path.append("../")
sys.path.append("../casadi_parser")

from dronemodel import *
from insectmodel import *
from helper import *
from casadi_parser import  parse_casadi_to_QP
from pathlib import Path

from optimizer_configurations import load_optimizer_settings

solvername, opts = load_optimizer_settings()
problem_name = 'tti'
version = "1"

n_timestepsI = 2

# Create Optimization Variables
optvars = init_optvar()
delta_tI = SX.sym('delta_tI')
optvars = register_optvar(delta_tI, "delta_tI", optvars)

statesI = SX.sym('statesI', n_drone_states, n_timestepsI,)
optvars = register_optvar(statesI, "statesI", optvars)

inputsI = SX.sym('inputsI', n_drone_inputs, n_timestepsI-1)
optvars = register_optvar(inputsI, "inputsI", optvars)

virtual_inputsI = SX.sym('virtual_inputsI', n_drone_states, n_timestepsI-1) #compensates for infeasible integration steps
optvars = register_optvar(virtual_inputsI, "virtual_inputsI", optvars)

insect_states = SX.sym('insect_state', n_insect_states, n_timestepsI)
optvars = register_optvar(insect_states, "insect_states", optvars)

# Create mapping between structured and flattened variables
variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in optvars["variables_list"]])
pack_variables_fn = casadi.Function('pack_variables_fn', optvars["variables_list"], [variables_flat], optvars["variables_name"], ['flat'])
unpack_variables_fn = casadi.Function('unpack_variables_fn', [variables_flat], optvars["variables_list"], ['flat'], optvars["variables_name"])

def export_casadi_defines(optvars, constraints):
    """Export index of optimization parameters for easier CPP integration."""
    with open(Path("~/code/pats/base/src/optimization/" + problem_name + "_index.h").expanduser(), "w") as f:
        f.write("#pragma once\n")
        f.write('#define ' + problem_name.upper() + '_VERSION "' + str(version) + '"\n')
        f.write("#define N_OPTVARS " + str(optvars["n_optvars"]) + "\n")
        f.write("#define N_CONSTRAINTS " + str(constraints["n_constraints"]) + "\n")
        f.write("#define N_DRONE_STATES " + str(n_drone_states) + "\n")
        f.write("#define N_DRONE_INPUTS " + str(n_drone_inputs) + "\n")
        f.write("#define N_INSECT_STATES " + str(n_insect_states) + "\n")
        for key in drone_model_idxs:
            f.write("#define " + str(key) + " " + str(drone_model_idxs[key]) + "\n")
        for key in insect_model_idxs:
            f.write("#define " + str(key) + " " + str(insect_model_idxs[key]) + "\n")

        f.write("\n")
        f.write("enum optvar_idx {\n")
        for j, idxs_key in enumerate(optvars["idxs"]):
            name = optvars["variables_name"][j]
            idxs = optvars["idxs"][idxs_key]
            if len(idxs) == 1:
                f.write("    " + name + " = " + str(optvars["idxs"][idxs_key][0, 0]) + ",\n")
            else:
                f.write("    " + name + "_first = " + str(optvars["idxs"][idxs_key][0, 0]) + ",\n")
                f.write("    " + name + "_last = " + str(optvars["idxs"][idxs_key][-1, -1]) + ",\n")
        f.write("};\n")
        f.write("\n")
        f.write("enum inequality_constraint_idx {\n")
        for j, idxs_key in enumerate(constraints["idxs"]):
            name = idxs_key
            idxs = constraints["idxs"][idxs_key]
            if len(idxs) == 1:
                f.write("    " + name + " = " + str(constraints["idxs"][idxs_key][0]) + ",\n")
            else:
                f.write("    " + name + "_first = " + str(constraints["idxs"][idxs_key][0]) + ",\n")
                f.write("    " + name + "_last = " + str(constraints["idxs"][idxs_key][-1]) + ",\n")
        f.write("};\n")
        f.write("\n")
        f.write("enum trjactory_parameter {\n")
        f.write("    n_steps = " + str(n_timestepsI-1) + ",\n")
        f.write("    state_trajectory_first = " + str(optvars["idxs"]["statesI"][0,0]) + ",\n")
        f.write("    input_trajectory_first = " + str(optvars["idxs"]["inputsI"][0,0]) + ",\n")
        f.write("    virtual_input_trajectory_first = " + str(optvars["idxs"]["virtual_inputsI"][0,0]) + ",\n")
        f.write("};\n")

def def_optimization():
    ineq_idxs = {}
    n_ineq = 0

    # Optimization objective
    objective = delta_tI[:, :]
    objective += 1e13*casadi.sum1(casadi.sum2(virtual_inputsI[:, :]**2))/(n_timestepsI-1)

    ## Differential equation constraints
    constraints = init_constraints()
    ### There is no loop here, because it is vectorized.
    X0I = statesI[:, 0:(n_timestepsI - 1)]
    X1I = statesI[:, 1:n_timestepsI]
    ### Heun's method (some other method, like RK4 could also be used here)
    K1I = drone_dynamics(X0I.T, inputsI.T)
    K2I = drone_dynamics(X0I.T + delta_tI * K1I, inputsI.T)
    dronedynamicI = X0I.T + delta_tI * (K1I + K2I) / 2.0 - X1I.T + virtual_inputsI[:, :].T
    constraints = register_constraints(dronedynamicI, "dronedynamicI", constraints)


    X0_insect = insect_states[:, 0:(n_timestepsI-1)]
    X1_insect = insect_states[:, 1:n_timestepsI]
    K1_insect = insect_dynamics(X0_insect.T)
    K2_insect = insect_dynamics(X0_insect.T + delta_tI * K1_insect)
    insectdynamic = X0_insect.T + delta_tI * (K1_insect + K2_insect) / 2.0 - X1_insect.T
    constraints = register_constraints(insectdynamic, "insectdynamic", constraints)

    # Constraint equations:
    ## Final state stage I:
    intercepting = statesI[idx_pos, -1] - insect_states[idx_pos, -1]
    constraints = register_constraints(intercepting, "intercepting", constraints)

    ### Run optimization
    ocp_constraints = []
    for i in range(len(constraints["equations"])):
        ocp_constraints = casadi.vertcat(ocp_constraints, constraints["equations"][i])

    solver = casadi.nlpsol('solver', solvername, {'x': variables_flat, 'f': objective, 'g': ocp_constraints}, opts)
    solver.generate_dependencies(problem_name + '_optimizer.cpp', {'with_header': False, 'cpp': False})
    os.system("gcc -fPIC -shared -O3 " + problem_name + "_optimizer.cpp -o " + problem_name + "_optimizer.so")

    parse_casadi_to_QP(problem_name, variables_flat, [], objective, ocp_constraints)
    export_casadi_defines(optvars, constraints)

    return solver, constraints["n_constraints"], constraints["idxs"]


def gen_initguess(x0, xinsect):
    """."""
    t_guess = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    u0 = (xinsect[idx_pos] - x0[idx_pos]) / np.linalg.norm(xinsect[idx_pos] - x0[idx_pos]) * 20

    # Calc initial guess
    deltatI_guess = t_guess/n_timestepsI
    insect_guess = np.zeros([n_timestepsI, n_insect_states])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, insect_pos0 + n_timestepsI * insect_vel0 * deltatI_guess, n_timestepsI, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_timestepsI, axis=1).T
    xend = np.array([insect_guess[-1, 0], insect_guess[-1, 1], insect_guess[-1, 2], 0, 0, 0])
    statesI_guess = np.linspace(x0, xend, n_timestepsI, axis=1).T
    inputsI_guess = np.linspace(u0, u0, n_timestepsI-1, axis=1).T
    virtualinputsI_guess = np.zeros([n_timestepsI-1, n_drone_states])

    init_guess = np.concatenate((np.array([[deltatI_guess]]),
                                 statesI_guess.reshape((n_drone_states*n_timestepsI, 1)),
                                 inputsI_guess.reshape(((n_timestepsI-1)*n_drone_inputs, 1)),
                                 virtualinputsI_guess.reshape((n_drone_states*(n_timestepsI-1), 1)),
                                 insect_guess.reshape((n_insect_states*n_timestepsI, 1))))
    return init_guess

def run_optimization(init_guess):
    solver, n_ineq, ineq_idxs = def_optimization()

    # Box constraints
    x0 = init_guess[idx_statesI[:, 0]]
    xinsect = init_guess[idx_insect_states[:, 0]]
    lower_bounds = unpack_variables_fn(flat=-float('inf')) # This creates a dict with inf and state, control and delta_t as keys
    upper_bounds = unpack_variables_fn(flat=float('inf'))

    lower_bounds['delta_tI'][:, :] = 1.0e-6 # Time must run forwards
    lower_bounds['inputsI'][idx_accx, :] = -DRONE_MAX_THRUST
    upper_bounds['inputsI'][idx_accx, :] = DRONE_MAX_THRUST
    lower_bounds['inputsI'][idx_accy, :] = -DRONE_MAX_THRUST
    upper_bounds['inputsI'][idx_accy, :] = DRONE_MAX_THRUST
    lower_bounds['inputsI'][idx_accz, :] = -DRONE_MAX_THRUST
    upper_bounds['inputsI'][idx_accz, :] = DRONE_MAX_THRUST

    ### Initial state
    lower_bounds['statesI'][idx_posx, 0] = x0[idx_posx]
    upper_bounds['statesI'][idx_posx, 0] = x0[idx_posx]
    lower_bounds['statesI'][idx_posy, 0] = x0[idx_posy]
    upper_bounds['statesI'][idx_posy, 0] = x0[idx_posy]
    lower_bounds['statesI'][idx_posz, 0] = x0[idx_posz]
    upper_bounds['statesI'][idx_posz, 0] = x0[idx_posz]

    lower_bounds['statesI'][idx_velx, 0] = x0[idx_velx]
    upper_bounds['statesI'][idx_velx, 0] = x0[idx_velx]
    lower_bounds['statesI'][idx_vely, 0] = x0[idx_vely]
    upper_bounds['statesI'][idx_vely, 0] = x0[idx_vely]
    lower_bounds['statesI'][idx_velz, 0] = x0[idx_velz]
    upper_bounds['statesI'][idx_velz, 0] = x0[idx_velz]


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

    # print('x0: [', ', '.join([str(i) for i in init_guess.flatten()]), ']')
    # print("lbx: ", pack_variables_fn(**lower_bounds)['flat'])
    # print("ubx: ", pack_variables_fn(**upper_bounds)['flat'])
    result = solver(x0=init_guess,
                    lbg =lbg_,
                    ubg=ubg_,
                    lbx=pack_variables_fn(**lower_bounds)['flat'],
                    ubx=pack_variables_fn(**upper_bounds)['flat'])

    results = unpack_variables_fn(flat=result['x'])
    # print("xopt: ", result['x'])
    return results


if __name__ == "__main__":
    if(len(sys.argv)==1):
        print("idx:", [*range(variables_flat.size()[0])])
        print("opt_vars:", variables_flat)
        def_optimization()
    else:
        x0 = np.array([-1, -1, 0, -1, -1, -1])
        xinsect = np.array([0, 0, 0, 0, 0, 0])

        # Run optimization:
        init_guess = gen_initguess(x0, xinsect)

        pretty_nparray_print("init_guess", init_guess)
        opti_res = run_optimization(init_guess)
        opti_res = pack_variables_fn(**opti_res)['flat']
        pretty_nparray_print('xopt', np.array(opti_res))
