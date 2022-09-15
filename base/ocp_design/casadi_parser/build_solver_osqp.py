
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import numpy as np
from pathlib import Path

row = 0
column = 1
value = 2

def build_classname(name):
    name = name.replace("_", " ")
    name = name.title()
    name = name.replace(" ", "")
    return name+"QuadraticOptimizer"


def upper_triangular(matrix_elements):
    entries_lower_triangle = []
    for i in range(len(matrix_elements)):
        if matrix_elements[i][0] > matrix_elements[i][1]:
            entries_lower_triangle.append(i)

    for idx in list(reversed(entries_lower_triangle)):
        # print("del", idx, matrix_elements[idx])
        del matrix_elements[idx]

    return matrix_elements


def build_sparse_arrays(n_columns, matrix_elements):
    matrix_elements.sort(key=lambda c: c[1])
    Pp = [0]
    Pi = []
    Px = []
    c_prev = matrix_elements[0]
    for c in matrix_elements:
        if c[column] != c_prev[column]:
            for i in range(0, c[column] - c_prev[column]):
                Pp.append(len(Pi))
            c_prev = c

        Pi.append(c[row])
        Px.append(c[value])

    filled_columns = len(Pp)-1
    for col in range(filled_columns, n_columns):
        Pp.append(len(Pi))

    return Pp, Pi, Px


def build_P(init, n_columns, quadratic_costs):
    """Build P matrix in csc-format."""
    quadratic_costs = upper_triangular(quadratic_costs)

    Pp, Pi, Px = build_sparse_arrays(n_columns, quadratic_costs)

    init_P = ""
    if init:
        init_P += "    c_int P_p[" + str(len(Pp)) + "] = {"
        for pp in Pp:
            init_P += str(pp) + ", "
        init_P += "};\n"

        init_P += "    c_int P_i[" + str(len(Pi)) + "] = {"
        for pi in Pi:
            init_P += str(pi) + ", "
        init_P += "};\n"
        init_P += "    c_float P_x[" + str(len(Px)) + "];"

    else:
        for px_idx, px in enumerate(Px):
            init_P += "    P_x[" + str(px_idx) + "] = " + str(px) + ";\n"

    return init_P


def build_q_entry(row, value):
    return "    q["+str(row) + "] = " + str(value) + ";\n"

def build_init_q(linear_costs):
    init_q = ''

    for c in linear_costs:
        init_q += build_q_entry(c[column], c[value])

    return init_q

def build_definition(def_name, value):
    return "    double " + def_name.replace("@", "const") + " = " + str(value) + ";\n"

def build_linear_constraint_entry(n_xopts, row, column, value):
    return "    A["+str(row*n_xopts + column) + "] = " + value.replace("@", "const") + ";\n"


def add_box_entries(n_xopts, n_constraints, matrix_elements):
    for k in range(n_xopts):
        matrix_elements.append([n_constraints + k, k, 1])

    return matrix_elements


def build_A(init, n_xopts, n_constraints, constraint_slopes):
    """Build A matrix in csc-format holding (in)equality constraints + box constraints."""
    constraint_slopes = add_box_entries(n_xopts, n_constraints, constraint_slopes)

    Ap, Ai, Ax = build_sparse_arrays(n_xopts, constraint_slopes)

    init_A = ""
    if init:
        init_A += "    c_int A_p[" + str(len(Ap)) + "] = {"
        for ap in Ap:
            init_A += str(ap) + ", "
        init_A += "};\n"

        init_A += "    c_int A_i[" + str(len(Ai)) + "] = {"
        for ai in Ai:
            init_A += str(ai) + ", "
        init_A += "};\n"
        init_A += "    c_float A_x[" + str(len(Ax)) + "];"

    else:
        for ax_idx, ax in enumerate(Ax):
            init_A += "    A_x[" + str(ax_idx) + "] = " + str(ax) + ";\n"

    return init_A

def build_dconstraint_entry(n_xopts, row, column, value):
    return "    dconstraints("+str(row) + ", " + str(column) + ") = " + value.replace("@", "const") + ";\n"

def build_dconstraints(n_xopts, constraint_slopes):
    """Almost as build A but saves into a Matrix structure and not a array."""
    A = ""

    for lin_c in constraint_slopes:
        A += build_dconstraint_entry(n_xopts, lin_c[row], lin_c[column], lin_c[value])

    return A

def build_lbA_entry(c_idx, value):
    # return "    lbA["+str(c_idx) + "] = lbg[" + str(c_idx) + "] - " + value.replace("@", "const") + ";\n"
    return "    l["+str(c_idx) + "] = lbg[" + str(c_idx) + "];\n"

def build_ubA_entry(c_idx, value):
    # return "    ubA["+str(c_idx) + "] = ubg[" + str(c_idx) + "] - " + value.replace("@", "const") + ";\n"
    return "    u["+str(c_idx) + "] = ubg[" + str(c_idx) + "];\n"

def build_bA(bound_corrections):
    bA = ""
    # for def_name in bound_corrections[0]:
    #     bA += build_definition(def_name, bound_corrections[0][def_name])

    for c_idx, lbAc in enumerate(bound_corrections):
        bA += build_lbA_entry(lbAc[0], lbAc[2])

    for c_idx, ubAc in enumerate(bound_corrections):
        bA += build_ubA_entry(ubAc[0], ubAc[2])
    return bA

def build_c_entry(c_idx, value):
    return "    constraints[" + str(c_idx) + "] = " + value.replace("@", "const") + ";\n"

def build_c(constraints):
    c = ""

    for c_idx, ck in enumerate(constraints):
        c += build_c_entry(ck[0], ck[2])

    return c

def build_dense_eigen_matrix(name, matrix_elements):
    code = ""
    for entry in matrix_elements:
        code += "    " + str(name) + "(" + str(entry[0]) + ", " + str(entry[1]) + ") = " + str(entry[2]) + ";\n"

    return code

def build_vector(name, vector_elements):
    code = ""
    for entry in vector_elements:
        code += "    " + str(name) + "[" + str(entry[1]) + "] = " + str(entry[2]) + ";\n"

    return code

def build_header(name, n_xopts, n_constraints, P_nnz, A_nnz, quadratic_costs, constraint_slopes):
    with open(Path('~/code/pats/base/ocp_design/casadi_parser/solvertemplate_osqp.h').expanduser(), 'r') as f:
        header = f.read();

    class_name = build_classname(name)
    header = header.replace("N_DIMS_H", str(n_xopts**2))
    header = header.replace("N_DIMS_A", str(n_xopts*n_constraints))
    header = header.replace("N_CONSTRAINTS + N_XOPTS", str(n_xopts + n_constraints))
    header = header.replace("N_XOPTS", str(n_xopts))
    header = header.replace("N_CONSTRAINTS", str(n_constraints))
    header = header.replace("P_NNZ", str(P_nnz))
    header = header.replace("A_NNZ", str(A_nnz))
    header = header.replace("SolverTemplate", class_name)

    init_P = build_P(True, n_xopts, quadratic_costs)
    header = header.replace("    /* P_P AND P_I PLACEHOLDER*/", init_P)
    init_A = build_A(True, n_xopts, n_constraints, constraint_slopes)
    header = header.replace("    /* INIT A_P AND A_I PLACEHOLDER*/", init_A)

    file_name = name + "_quad_opti_osqp.h"
    print(file_name)
    with open(Path('~/code/pats/base/src/optimization/' + file_name).expanduser(), "w") as f:
        f.write(header)


def build_solver_osqp(name, n_xopts, linear_costs, quadratic_costs, constraints, constraint_slopes, bound_corrections):
    """."""
    n_constraints = len(constraints)

    Pp, Pi, Px = build_sparse_arrays(n_xopts, upper_triangular(copy.deepcopy(quadratic_costs)))
    assert(n_xopts+1 ==len(Pp))
    P_nnz = len(Px)
    Ap, Ai, Ax = build_sparse_arrays(n_xopts, add_box_entries(n_xopts, n_constraints, copy.deepcopy(constraint_slopes)))
    A_nnz = len(Ax)
    build_header(name, n_xopts, n_constraints, P_nnz, A_nnz, copy.deepcopy(quadratic_costs), copy.deepcopy(constraint_slopes))

    with open(Path('~/code/pats/base/ocp_design/casadi_parser/solvertemplate_osqp.cpp').expanduser(), 'r') as f:
        source = f.read()

    source = source.replace("solvertemplate", name+"_quad_opti_osqp")
    class_name = build_classname(name)
    source = source.replace("SolverTemplate", class_name)
    source = source.replace("N_XOPTS + N_CONSTRAINTS", str(n_xopts + n_constraints))
    source = source.replace("N_XOPTS", str(n_xopts))
    source = source.replace("P_NNZ", str(P_nnz))
    source = source.replace("A_NNZ", str(A_nnz))
    source = source.replace("N_CONSTRAINTS", str(n_constraints))
    init_q = build_init_q(linear_costs)
    source = source.replace("    /* INIT q PLACEHOLDER*/", init_q)
    source = source.replace("    /* INIT H DENSE PLACEHOLDER*/", build_dense_eigen_matrix("_H", quadratic_costs))
    source = source.replace("    /* INIT G PLACEHOLDER*/", build_vector("_g", linear_costs))

    A = build_A(False, n_xopts, n_constraints, copy.deepcopy(constraint_slopes))
    source = source.replace("    /* UPDATE A PLACEHOLDER*/", A)

    bA = build_bA(bound_corrections)
    source = source.replace("    /* UPDATE bA PLACEHOLDER*/", bA)

    c = build_c(constraints)
    source = source.replace("    /* CONSTRAINTS PLACEHOLDER*/", c)

    P = build_P(False, n_xopts, copy.deepcopy(quadratic_costs))
    source = source.replace("    /* UPDATE P PLACEHOLDER*/", P)

    dconstr = build_dconstraints(n_xopts, copy.deepcopy(constraint_slopes))
    source = source.replace("    /* CONSTRAINT_DERIVATIVES PLACEHOLDER*/", dconstr)

    file_name = name + "_quad_opti_osqp.cpp"
    print(file_name)
    with open(Path('~/code/pats/base/src/optimization/' + file_name).expanduser(), "w") as f:
        f.write(source)

if __name__ == "__main__":
    s = [[1, 0, 1], [1, 1, 2], [0, 0, 3], [0, 1, 4]]
    Pp, Pi, Px = build_sparse_arrays(3, s)
    print("Pp:", Pp)
    print("Pi:", Pi)
    print("Px:", Px)
