#pragma once
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <qpOASES.hpp>
#include "quadraticoptimizer.h"

USING_NAMESPACE_QPOASES

class SolverTemplate : public QuadraticOptimizer {
public:
    void init();
    Eigen::VectorXd constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    Eigen::VectorXd constraints(Eigen::VectorXd xopt, Eigen::VectorXd param);
    Eigen::MatrixXd constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    problem_solution solve(problem_parameters *prob_params, bool init, double cpu_time);
    problem_solution solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time);

    void nWSR(int nwsr) {if (nwsr < 1) _nWSR = 1; else _nWSR = nwsr;};
    void change_settings(float alpha, [[maybe_unused]] bool polish [[maybe_unused]]) {return;};
    double costs(Eigen::VectorXd X);

    std::string quadratic_solver_library() { return "qpOases";};

private:
    real_t H[N_DIMS_H];
    real_t g[N_XOPTS];
    real_t A[N_DIMS_A];
    real_t lb[N_XOPTS];
    real_t ub[N_XOPTS];
    real_t lbA[N_CONSTRAINTS];
    real_t ubA[N_CONSTRAINTS];

    SQProblem solver;
    int _nWSR = 40;
    double *_cpu_time;

    void update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void copy_xopt();
    void update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution);

    void update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_vectors_bA(problem_parameters *prob_params, problem_solution *prev_qpsolution);

    double sq(double x) {return x * x;};

    int qp_return_status(int solver_status) {
        if (solver_status == SUCCESSFUL_RETURN) // || solver_status == RET_MAX_NWSR_REACHED)
            return 0;
        else
            return 1;
    };

    void print_quadratic_problem(real_t *primal, real_t *dual) {
        std::cout << "***********PATS Optimization step:**************" << std::endl;
        print_array("H", N_DIMS_H, H);
        print_array("g", N_XOPTS, g);
        print_array("A", N_DIMS_A, A);
        print_array("lb", N_XOPTS, lb);
        print_array("ub", N_XOPTS, ub);
        print_array("lbA", N_CONSTRAINTS, lbA);
        print_array("ubA", N_CONSTRAINTS, ubA);
        print_array("prim", N_XOPTS, primal);
        print_array("(-1)*dual", N_XOPTS + N_CONSTRAINTS, dual); //dual is printed with inverted sign, the lagrange_multiplier for the constraints have the correct sign however.
        std::cout << "*******************************************" << std::endl;
    };
};
