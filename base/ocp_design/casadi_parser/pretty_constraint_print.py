#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""."""
from find_keywords import find_keywords
from parse_matrix_formulation import parse_matrix_formulation

def pretty_constraint_print(name):
    """."""
    with open(name+'.txt', 'r') as f:
        formulation = f.read()

    formulation_lines = formulation.split('\n')
    line_objective_jacobian, line_objective_hessian, line_constraints, line_constraints_jacobian, line_bound_constraints = find_keywords(formulation_lines)
    lines = formulation_lines[line_constraints].split(",")

    defs = {}
    clean_lines = []
    for line in lines:
        line = line.replace(" ", "")
        line = line.replace("[", "")
        line = line.replace("]", "")

        for key in defs:
            line = line.replace(key, defs[key])

        if line[0] == "@" and "=" in line:
            def_elements = line.split("=")
            defs[def_elements[0]] = def_elements[1]

        else:
            clean_lines.append(line)

    print("constraint_idx:", [*range(len(clean_lines))])
    print("constraint:", clean_lines)
