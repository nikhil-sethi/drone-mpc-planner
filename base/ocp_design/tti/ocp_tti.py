#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import casadi
from casadi import SX, DM

sys.path.append("../")
sys.path.append("../casadi_parser")

from dronemodel import *
from insectmodel import *
from helper import *
from casadi_parser import  parse_casadi_to_QP

from optimizer_configurations import load_optimizer_settings

solvername, opts = load_optimizer_settings()
name = 'tti'

n_timestepsI = 2

# Create Optimization Variables
n_optvars = 0
delta_tI = SX.sym('delta_tI')
n_optvars, idx_delta_tI = update_optvar_index(n_optvars, delta_tI)

statesI = SX.sym('statesI', n_states, n_timestepsI,)
n_optvars, idx_statesI = update_optvar_index(n_optvars, statesI)

inputsI = SX.sym('inputsI', n_inputs, n_timestepsI-1)
n_optvars, idx_inputsI = update_optvar_index(n_optvars, inputsI)

virtual_inputsI = SX.sym('virtual_inputsI', n_states, n_timestepsI-1) #compensates for infeasible integration steps
n_optvars, idx_virtual_inputsI = update_optvar_index(n_optvars, virtual_inputsI)

insect_states = SX.sym('insect_state', n_insectstates, n_timestepsI)
n_optvars, idx_insect_states = update_optvar_index(n_optvars, insect_states)

key_dronedynamicsI = 'dronedynamicsI'
key_insectdynamics = 'insectdynamics'
key_intercepting = 'intercepting'

# Create mapping between structured and flattened variables
variables_list = [delta_tI, statesI, inputsI, virtual_inputsI, insect_states]
variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in variables_list])
variables_name = ['delta_tI', 'statesI', 'inputsI', 'virtual_inputsI', 'insect_states']
pack_variables_fn = casadi.Function('pack_variables_fn', variables_list, [variables_flat], variables_name, ['flat'])
unpack_variables_fn = casadi.Function('unpack_variables_fn', [variables_flat], variables_list, ['flat'], variables_name)
# print("variables_flat: ", variables_flat)

def def_optimization():
    """."""
    ineq_idxs = {}
    n_ineq = 0

    # Optimization objective
    objective = delta_tI[:, :]
    objective += 1e13*casadi.sum1(casadi.sum2(virtual_inputsI[:, :]**2))/(n_timestepsI-1)

    ## Differential equation constraints
    ### There is no loop here, because it is vectorized.
    X0I = statesI[:, 0:(n_timestepsI - 1)]
    X1I = statesI[:, 1:n_timestepsI]
    ### Heun's method (some other method, like RK4 could also be used here)
    K1I = drone_dynamics(X0I.T, inputsI.T)
    K2I = drone_dynamics(X0I.T + delta_tI * K1I, inputsI.T)
    constr_dronedynamicI = X0I.T + delta_tI * (K1I + K2I) / 2.0 - X1I.T + virtual_inputsI[:, :].T
    constr_dronedynamicI = casadi.reshape(constr_dronedynamicI, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_dronedynamicsI, constr_dronedynamicI)

    X0_insect = insect_states[:, 0:(n_timestepsI-1)]
    X1_insect = insect_states[:, 1:n_timestepsI]
    K1_insect = insect_dynamics(X0_insect.T)
    K2_insect = insect_dynamics(X0_insect.T + delta_tI * K1_insect)
    constr_insectdynamic = X0_insect.T + delta_tI * (K1_insect + K2_insect) / 2.0 - X1_insect.T
    constr_insectdynamic = casadi.reshape(constr_insectdynamic, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_insectdynamics, constr_insectdynamic)

    # Constraint equations:
    ## Final state stage I:
    constr_intercepting = statesI[idx_pos, -1] - insect_states[idx_pos, -1]
    constr_intercepting = casadi.reshape(constr_intercepting, -1, 1)
    ineq_idxs, n_ineq = update_consraint_index(n_ineq, ineq_idxs, key_intercepting, constr_intercepting)


    ### Run optimization
    constraints = casadi.vertcat(constr_dronedynamicI, constr_insectdynamic, constr_intercepting)

    solver = casadi.nlpsol('solver', solvername, {'x':variables_flat, 'f':objective, 'g':constraints}, opts)
    solver.generate_dependencies(name + '.cpp', {'with_header': False, 'cpp': False})
    os.system("gcc -fPIC -shared -O3 " + name + ".cpp -o " + name + ".so")

    parse_casadi_to_QP(name, variables_flat, [], objective, constraints)

    return solver, n_ineq, ineq_idxs


def gen_initguess(x0, xinsect):
    """."""
    t_guess = np.linalg.norm(xinsect[idx_pos] - x0[idx_pos])
    insect_pos0 = xinsect[idx_pos]
    insect_vel0 = xinsect[idx_vel]
    u0 = (xinsect[idx_pos] - x0[idx_pos]) / np.linalg.norm(xinsect[idx_pos] - x0[idx_pos]) * 20

    # Calc initial guess
    deltatI_guess = t_guess/n_timestepsI
    insect_guess = np.zeros([n_timestepsI, n_insectstates])
    insect_guess[:, idx_pos] = np.linspace(insect_pos0, insect_pos0 + n_timestepsI * insect_vel0 * deltatI_guess, n_timestepsI, axis=1).T
    insect_guess[:, idx_vel] = np.linspace(insect_vel0, insect_vel0, n_timestepsI, axis=1).T
    xend = np.array([insect_guess[-1, 0], insect_guess[-1, 1], insect_guess[-1, 2], 0, 0, 0])
    statesI_guess = np.linspace(x0, xend, n_timestepsI, axis=1).T
    inputsI_guess = np.linspace(u0, u0, n_timestepsI-1, axis=1).T
    virtualinputsI_guess = np.zeros([n_timestepsI-1, n_states])

    init_guess = np.concatenate((np.array([[deltatI_guess]]),
                                 statesI_guess.reshape((n_states*n_timestepsI, 1)),
                                 inputsI_guess.reshape(((n_timestepsI-1)*n_inputs, 1)),
                                 virtualinputsI_guess.reshape((n_states*(n_timestepsI-1), 1)),
                                 insect_guess.reshape((n_insectstates*n_timestepsI, 1))))
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
