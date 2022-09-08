#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Takes a optimization problem formulated by casadi and parses it into a QP."""
import casadi
from find_keywords import find_keywords
from save_problem_formulation import save_problem_as_text
from generalize_formulation import *
from clean_formulation import clean_formulation
from parse_matrix_formulation import parse_matrix_formulation, parse_constraint_formulation, parse_complex_matrix_formulation
from build_solver_dense import build_solver_dense
from build_solver_sparse import build_solver_sparse
from pretty_constraint_print import pretty_constraint_print


def lines_to_string(lines):
    text = ""
    for line in lines:
        text += line + "\n"

    return text

def parse_matrix_element(line):
    tmp = line.split(" -> ")

def parse_casadi_formulation(name):
    with open(name+'.txt', 'r') as f:
        formulation = f.read()

    formulation = generalize_formulation(formulation)

    formulation_lines = formulation.split('\n')
    n_xopts = find_nxopts(formulation_lines)

    with open(name+'-parsed.txt', 'w') as f:
        for fl in formulation_lines:
            f.write(fl+"\n")

    line_objective_jacobian, line_objective_hessian, line_constraints, line_constraints_jacobian, line_bound_constraints = find_keywords(formulation_lines)

    # print(line_objective_jacobian, line_objective_hessian, line_constraints, line_constraints_jacobian, line_bound_constraints)

    # print("linear_costs_lines: "+str(formulation_lines[line_objective_jacobian:line_objective_hessian-1]))
    linear_costs = parse_matrix_formulation(formulation_lines[line_objective_jacobian:line_objective_hessian-1])
    # print("linear_costs:", linear_costs)

    # print("quadratic_cost_lines: " +str(formulation_lines[line_objective_hessian:line_constraints-1]))
    quadratic_costs = parse_matrix_formulation(formulation_lines[line_objective_hessian:line_constraints-1])
    # print("quadratic_costs:", quadratic_costs)

    constraints = parse_matrix_formulation(formulation_lines[line_constraints:line_constraints_jacobian-1])
    # print("constraints:", constraints)

    # print("Constraint slopes formualtions: " + str(formulation_lines[line_constraints_jacobian:line_bound_constraints]))
    constraint_slopes = parse_matrix_formulation(formulation_lines[line_constraints_jacobian:line_bound_constraints-1])

    # print("bound_constraint_formulation: " + str(formulation_lines[line_bound_constraints]))
    bound_corrections = parse_matrix_formulation(formulation_lines[line_bound_constraints:-1])
    # print("bound_constraint: " + str(bound_corrections))

    build_solver_sparse(name, n_xopts, linear_costs, quadratic_costs, constraints, constraint_slopes, bound_corrections)
    # build_solver_dense(name, n_xopts, linear_costs, quadratic_costs, constraints, constraint_slopes, bound_corrections)

    formulation = lines_to_string(formulation_lines)


def parse_casadi_to_QP(name, xopt, parameters, objective, constraints):
    """constraints only holds inequalitiy constraints!."""
    save_problem_as_text(name, xopt, parameters, objective, constraints)
    parse_casadi_formulation(name)
    pretty_constraint_print(name)

    print("Casadi2Cpp for '" + name + "' done.")


if __name__ == '__main__':
    from casadi import *

    # Declare variables
    x = SX.sym("x",2)

    # Form the NLP
    f = x[0]**2 + x[1]**2 # objective
    g = x[0]+x[1]-10      # constraint

    parse_casadi_to_QP("min_nlp", x, [], f, g)
