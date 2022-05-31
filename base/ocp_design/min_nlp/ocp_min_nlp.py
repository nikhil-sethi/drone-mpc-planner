#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import casadi
from casadi import SX, DM

sys.path.append("../")
sys.path.append("../casadi_parser")

from helper import *
from casadi_parser import  parse_casadi_to_QP

from optimizer_configurations import load_optimizer_settings


solvername, opts = load_optimizer_settings()
problem_name = 'min_nlp'

n_timestepsI = 2

# Create Optimization Variables
n_optvars = 0
X = SX.sym('X', 2, 1)

variables_list = [X]
variables_flat = casadi.vertcat(*[casadi.reshape(e, -1, 1) for e in variables_list])

objective = casadi.sum1(casadi.sum2(X[:, :]**2))
constraint1 = X[0,0] * X[1,0]
constraint2 = X[0,0]**2
constraints = casadi.vertcat(constraint1, constraint2)
constraints = casadi.reshape(constraints, -1, 1)

solver = casadi.nlpsol('solver', solvername, {'f':objective, 'x':variables_flat, 'g':constraints}, opts)
solver.generate_dependencies(problem_name+'_optimizer.cpp', {'with_header': False, 'cpp': False})
os.system("gcc -fPIC -shared -O3 "+problem_name+"_optimizer.cpp -o "+problem_name+"_optimizer.so")

parse_casadi_to_QP(problem_name, variables_flat, [], objective, constraints)
