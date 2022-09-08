
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import casadi


def save_as_sparse_matrix(matrix):
    matrix_str = ""
    for row in range(matrix.size1()):
        for column in range(matrix.size2()):
            if str(matrix[row, column]) != "00":
                matrix_str += " (" + str(row) + ", " +str(column) + ") -> " + str(matrix[row, column]) + "\n"
    return matrix_str


def save_problem_as_text(name, xopt, parameters, objective, constraints):
    n_xopts = xopt.size1()
    n_constr = constraints.size1()
    lam_g= casadi.SX.sym('lamg', n_constr, 1) # Langrange multiplier constraint equations

    with open(name+'.txt', 'w') as f:
        f.write("Optimization variable:\n")
        f.write(str(xopt)+'\n')
        f.write("Parameters:\n")
        f.write(str(parameters)+'\n')
        f.write("Lagrange multiplier constraints:\n")
        f.write(str(lam_g)+'\n')
        f.write("Objectives jacobian:\n")
        jacobian_objective = casadi.jacobian(objective, xopt)
        f.write(save_as_sparse_matrix(jacobian_objective))

        jacobian_constraints = casadi.jacobian(constraints, xopt)
        hessian_objective = casadi.jacobian(jacobian_objective, xopt)
        for c in range(n_constr):
            hessian_objective += casadi.jacobian(lam_g[c] * jacobian_constraints[c,:], xopt)

        f.write("Objectives hessian:\n")
        f.write(save_as_sparse_matrix(hessian_objective))

        f.write("Constraints:\n")
        f.write(save_as_sparse_matrix(constraints))
        f.write("Constraints jacobian:\n")
        f.write(save_as_sparse_matrix(jacobian_constraints))
        f.write("CX0-dCX0:\n")
        f.write(save_as_sparse_matrix(constraints - jacobian_constraints@xopt))


if __name__ == '__main__':
    test = """[[@1, 00],\n[00, @1]]"""
