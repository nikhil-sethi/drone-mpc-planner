#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import casadi
from casadi import SX, DM

sys.path.append("../")
from dronemodel import *
from insectmodel import *
from helper import *
from planes import *
from saveresults import *

from ocp_intercept_in_planes import gen_initguess, run_optimization, n_samples_intercepting, eval_results, results_to_initguess, n_samples_intercepting, unpack_variables_fn


def scenario1():
    # Define initial states:
    x0 = np.array([-0.0501962, -1.15732, -0.991027, 0, 0, 0])
    xinsect = np.array([0.1, -0.7, -1.9, -1.5, 0, 0])
    u0 = (xinsect[:3] - x0[:3]) / np.linalg.norm(xinsect[:3] - x0[:3]) * DRONE_MAX_THRUST * 0.95
    uend_guess = u0
    plane_params = np.array([[0.00134892, -0.149797, 0.0076874, 0.00899278, -0.998645, 0.0512494],
                    [-0.12533, -0.0567185, -0.0597957, -0.83553, -0.378124, -0.398638],
                    [0, -0.523709, -0.851897, 0, -0.523709, -0.851897],
                    [0, 0, -3.85, 0, 0, 1],
                    [0.126356, -0.0543524, -0.0598335, 0.842371, -0.362349, -0.39889],
                    [0, 0, -1.03267, 0, 0, -1],
                    [0, -0.95172, 0, 0, 1, 0],
                    [0, -0.95172, 0, 0, 1, 0]])

    # Run optimization:
    init_guess = gen_initguess(x0, xinsect)
    # save_results_csv(unpack_variables_fn(init_guess), True, "init_guess.csv")
    for i in range(1):
        results = run_optimization(init_guess, plane_params)
        # save_results_csv(results, True, "result"+str(i)+".csv")
        init_guess = results_to_initguess(results)

        eval_results(results, plane_params)

def scenario2():
    x0 = np.array([0, 0, 0, 0, 0, 0])
    xinsect = np.array([-0.746902, -0.659168, -1.38966, 1.18392, -1.32707, -1.43181])
    u0 = (xinsect[:3] - x0[:3]) / np.linalg.norm(xinsect[:3] - x0[:3]) * 20
    uend_guess = u0
    plane_params = plane_parameters()

    # Run optimization:
    init_guess = gen_initguess(x0, xinsect)
    # save_results_csv(unpack_variables_fn(init_guess), True, "init_guess.csv")
    for i in range(1):
        results = run_optimization(init_guess, plane_params)
        save_results_csv(results, True, "result"+str(i)+".csv")
        init_guess = results_to_initguess(results)

        # save_results(results)
        eval_results(results, plane_params)

def scenario3():

    x0 = np.array([-1, -1, 0, -1, -1, -1])
    xinsect = np.array([0, 0, 0, 0, 0, 0])
    plane_params = np.array([[ 100, 0, 0, -1, 0, 0]
                             # [-100, 0, 0, 1, 0, 0],
                             # [0,  100, 0, 0, -1, 0],
                             # [0, -100, 0, 0, 1, 0],
                             # [0, 0,  100, 0, 0, -1],
                             # [0, 0, -100, 0, 0, 1]
                             ])

    # Run optimization:
    init_guess = gen_initguess(x0, xinsect)
    # save_results_csv(unpack_variables_fn(init_guess), True, "init_guess.csv")
    for i in range(1):
        results = run_optimization(init_guess, plane_params)
        save_results_csv(results, True, "result"+str(i)+".csv")
        init_guess = results_to_initguess(results)
        print("xopt", results)

        # save_results(results)
        eval_results(results, plane_params)

scenario3()
