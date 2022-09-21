#!/usr/bin/env python3
"""."""
from colorama import Fore
import numpy as np
import casadi


def update_constraint_index(n_ineq, ineq_idxs, key, constraints):
    """Based on the new constraints, updates the total amount of constraints n_ineq and saves the indices of the constraints in ineq_idxs."""
    n_constraints = constraints.shape[0]
    ineq_idxs[key] = np.linspace(n_ineq, n_ineq + n_constraints - 1, n_constraints, dtype=int)
    n_ineq += n_constraints

    return ineq_idxs, n_ineq


def update_optvar_index(prev_n, optvars):
    """Based on the new opimization variables optvars, determines their indices optvars_idxs and returns the updated total amount of optimizatin variables (which is seen for further optimization variables as prev_variable index) prev_n."""
    optvars_idxs = np.linspace(0, optvars.size1() * optvars.size2() - 1, optvars.size1() * optvars.size2(), dtype=int) + prev_n
    prev_n += optvars.size1() * optvars.size2()

    return prev_n, optvars_idxs.reshape((optvars.size1(), optvars.size2()), order='F')


def init_optvar():
    return {"n_optvars": 0, "idxs": {}, "variables_list": [], "variables_name": []}


def register_optvar(new_optvar, name, optvars):
    optvars["n_optvars"], optvars["idxs"][name] = update_optvar_index(optvars["n_optvars"], new_optvar)
    optvars["variables_list"].append(new_optvar)
    optvars["variables_name"].append(name)

    return optvars

def init_constraints():
    return {"n_constraints": 0, "idxs": {}, "equations": []}

def register_constraints(new_constraints, name, constraints):
   new_constraints = casadi.reshape(new_constraints, -1, 1)
   constraints["idxs"], constraints["n_constraints"] = update_constraint_index(constraints["n_constraints"], constraints["idxs"], name, new_constraints)
   constraints["equations"].append(new_constraints)

   return constraints

def angle_between_vectors(vec1, vec2):
    """."""
    return np.arccos(np.dot(vec1, vec2) / np.linalg.norm(vec1) / np.linalg.norm(vec2))


def pretty_nparray_print(array_name, array):
    """Print arrays for comparison with c++ arrays."""
    print(array_name + ": [" + ", ".join([str(el) for el in array.flatten()]) + "]")


def print_optimizer_input(variables_flat, init_guess, lbg, ubg, lbx, ubx, planes=[]):
    """."""
    pretty_nparray_print("idx", np.arange(len(init_guess)))
    print("opt_vars:", variables_flat)
    pretty_nparray_print("init_guess", init_guess)
    print("lower_box_constraint:", lbx)
    print("upper_box_constraint:", ubx)
    pretty_nparray_print("planes:", planes)

    pretty_nparray_print("lower_ineq_constraint", lbg)
    pretty_nparray_print("upper_ineq_constraint", ubg)


def add_noise(matrix, noise_amplitute):
    """Assuming a initial guess for optimization problems. Add noise beginning in the second line."""
    matrix_shape = np.shape(matrix)
    if matrix_shape[0] > 2:
        noise = np.random.rand(matrix_shape[0] - 1, matrix_shape[1]) * noise_amplitute
        noise = np.concatenate((np.zeros([1, matrix_shape[1]]), noise))
        matrix += noise

    return matrix


def first_order_poly_between_two_points(p0, v0, p1, v1):
    """Determines the parameter for polynomial A + B*t + C*t^2 + D*t^3 which connects the points p0 and p1, while the slope of the polynomial of the funtion at p0 is v0 and v1 at p1."""
    A = p0
    B = v0
    C = 2 * p0 - 2 * p1 + v0 + v1
    D = -3 * p0 + 3 * p1 - 2 * v0 - v1
    return A, B, C, D


def print_constraint_check(text, condition, value, unit=""):
    """Based if condition is fullfilled print the check text in red or green."""
    if condition:
        print(Fore.GREEN + text + ": " + str(value) + unit + Fore.RESET)
    else:
        print(Fore.RED + text + ": " + str(value) + unit + Fore.RESET)
