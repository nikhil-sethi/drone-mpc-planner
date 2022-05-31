#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def find_keywords(formulation_lines):
    line_objective_jacobian = 7
    line_objective_hessian = -1
    line_constraints = -1
    line_constraints_jacobian = -1
    line_bound_corrections = -1
    for line, line_content in enumerate(formulation_lines):
        if line_content == "Objectives hessian:":
            line_objective_hessian = line + 1
        elif line_content == "Constraints:":
            line_constraints = line + 1
        elif line_content == "Constraints jacobian:":
            line_constraints_jacobian = line + 1
        elif line_content == "CX0-dCX0:":
            line_bound_corrections = line + 1

    return line_objective_jacobian, line_objective_hessian, line_constraints, line_constraints_jacobian, line_bound_corrections
