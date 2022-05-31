#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pathlib import Path

def build_classname(name):
    name = name.replace("_", " ")
    name = name.title()
    name = name.replace(" ", "")
    return name+"QuadraticOptimizer"

def build_header(name, n_xopts, n_constraints):
    with open(Path('~/code/pats/base/ocp_design/casadi_parser/solvertemplate.h').expanduser(), 'r') as f:
        header = f.read();

    class_name = build_classname(name)
    header = header.replace("N_DIMS_H", str(n_xopts**2))
    header = header.replace("N_DIMS_A", str(n_xopts*n_constraints))
    header = header.replace("N_XOPTS", str(n_xopts))
    header = header.replace("N_CONSTRAINTS", str(n_constraints))
    header = header.replace("SolverTemplate", class_name)

    with open(name+"_quadratic_optimizer.h", "w") as f:
        f.write(header)


def build_H_entry(n_xopts, row, column, value):
    return "    H["+str(row*n_xopts + column) + "] = " + str(value) + ";\n"


def build_init_H(n_xopts, quadratic_costs):
    init_H = ""
    for c in quadratic_costs:
        init_H += build_H_entry(n_xopts, c[0], c[1], c[2])

    return init_H

def build_H(n_xopts, quadratic_costs):
    H = ""
    for c in quadratic_costs:
        H += build_H_entry(n_xopts, c[0], c[1], c[2])

def build_g_entry(row, value):
    return "    g["+str(row) + "] = " + str(value) + ";\n"

def build_init_g(linear_costs):
    init_g = ''

    for c in linear_costs:
        init_g += build_g_entry(c[1], c[2])

    return init_g

def build_definition(def_name, value):
    return "    double " + def_name.replace("@", "const") + " = " + str(value) + ";\n"

def build_linear_constraint_entry(n_xopts, row, column, value):
    return "    A["+str(row*n_xopts + column) + "] = " + value.replace("@", "const") + ";\n"


def build_A(n_xopts, constraint_slopes):
    A = ""
    for def_name in constraint_slopes[0]:
        A += build_definition(def_name, constraint_slopes[0][def_name])

    for lin_c in constraint_slopes[1]:
        A += build_linear_constraint_entry(n_xopts, lin_c[0], lin_c[1], lin_c[2])
    return A

def build_dconstraint_entry(n_xopts, row, column, value):
    return "    dconstraints("+str(row) + ", " + str(column) + ") = " + value.replace("@", "const") + ";\n"

def build_dconstraints(n_xopts, constraint_slopes):
    """Almost as build A but saves into a Matrix structure and not a array."""
    A = ""
    for def_name in constraint_slopes[0]:
        A += build_definition(def_name, constraint_slopes[0][def_name])

    for lin_c in constraint_slopes[1]:
        A += build_dconstraint_entry(n_xopts, lin_c[0], lin_c[1], lin_c[2])

    return A

def build_lbA_entry(c_idx, value):
    # return "    lbA["+str(c_idx) + "] = lbg[" + str(c_idx) + "] - " + value.replace("@", "const") + ";\n"
    return "    lbA["+str(c_idx) + "] = lbg[" + str(c_idx) + "];\n"

def build_ubA_entry(c_idx, value):
    # return "    ubA["+str(c_idx) + "] = ubg[" + str(c_idx) + "] - " + value.replace("@", "const") + ";\n"
    return "    ubA["+str(c_idx) + "] = ubg[" + str(c_idx) + "];\n"

def build_bA(bound_corrections):
    bA = ""
    # for def_name in bound_corrections[0]:
    #     bA += build_definition(def_name, bound_corrections[0][def_name])

    for c_idx, lbAc in enumerate(bound_corrections[1]):
        bA += build_lbA_entry(c_idx, lbAc)

    for c_idx, ubAc in enumerate(bound_corrections[1]):
        bA += build_ubA_entry(c_idx, ubAc)
    return bA

def build_c_entry(c_idx, value):
    return "    constraints[" + str(c_idx) + "] = " + value.replace("@", "const") + ";\n"

def build_c(constraints):
    c = ""

    for def_name in constraints[0]:
        c += build_definition(def_name, constraints[0][def_name])

    for c_idx, ck in enumerate(constraints[1]):
        c += build_c_entry(c_idx, ck)

    return c

def build_solver(name, n_xopts, linear_costs, quadratic_costs, constraints, constraint_slopes, bound_corrections):
    """."""
    n_constraints = len(constraints[1])
    build_header(name, n_xopts, n_constraints)

    with open(Path('~/code/pats/base/ocp_design/casadi_parser/solvertemplate.cpp').expanduser(), 'r') as f:
        source = f.read();

    source = source.replace("solvertemplate", name+"_quadratic_optimizer")
    class_name = build_classname(name)
    source = source.replace("SolverTemplate", class_name)
    source = source.replace("N_XOPTS + N_CONSTRAINTS", str(n_xopts + n_constraints))
    source = source.replace("N_XOPTS", str(n_xopts))
    source = source.replace("N_CONSTRAINTS", str(n_constraints))
    init_H = build_init_H(n_xopts, quadratic_costs)
    source = source.replace("    /* INIT H PLACEHOLDER*/", init_H)
    init_g = build_init_g(linear_costs)
    source = source.replace("    /* INIT g PLACEHOLDER*/", init_g)

    A = build_A(n_xopts, constraint_slopes)
    source = source.replace("    /* UPDATE A PLACEHOLDER*/", A)

    bA = build_bA(bound_corrections)
    source = source.replace("    /* UPDATE bA PLACEHOLDER*/", bA)

    c = build_c(constraints)
    source = source.replace("    /* CONSTRAINTS PLACEHOLDER*/", c)

    H = build_init_H(n_xopts, quadratic_costs)
    source = source.replace("    /* UPDATE H PLACEHOLDER*/", H)

    dconstr = build_dconstraints(n_xopts, constraint_slopes)
    source = source.replace("    /* CONSTRAINT_DERIVATIVES PLACEHOLDER*/", dconstr)

    with open(name+"_quadratic_optimizer.cpp", "w") as f:
        f.write(source)
