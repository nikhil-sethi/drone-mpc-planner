#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import casadi
from casadi import SX, DM

from dronemodel import *
from insectmodel import *
from helper import *
from planes import *
from saveresults import *
sys.path.append("./intercept_in_planes/")
sys.path.append("./casadi_parser/")

from ocp_intercept_in_planes import gen_initguess, run_optimization, n_samples_intercepting, eval_results, results_to_initguess, n_samples_intercepting, unpack_variables_fn

# Define initial states:
x0 = np.array([0, 0, 0, 0, 0, 0])
xinsect = np.array([-0.746902, -0.659168, -1.38966, 1.18392, -1.32707, -1.43181])
u0 = (xinsect[:3] - x0[:3]) / np.linalg.norm(xinsect[:3] - x0[:3]) * 20
uend_guess = u0
plane_params = plane_parameters()

# Run optimization:
init_guess = gen_initguess(x0, xinsect)
save_results_csv(unpack_variables_fn(init_guess), True, "init_guess.csv")
for i in range(1):
    results = run_optimization(init_guess, plane_params)
    save_results_csv(results, True, "result"+str(i)+".csv")
    init_guess = results_to_initguess(results)

save_results(results)
eval_results(results, plane_params)
