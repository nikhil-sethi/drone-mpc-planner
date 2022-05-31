#pragma once
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <qpOASES.hpp>
#include "quadraticoptimizer.h"

USING_NAMESPACE_QPOASES

class TtiQuadraticOptimizer : public QuadraticOptimizer {
public:
    void init();
    Eigen::VectorXd constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    Eigen::VectorXd constraints(Eigen::VectorXd xopt, Eigen::VectorXd param);
    Eigen::MatrixXd constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    problem_solution solve(problem_parameters *prob_params, bool init, double cpu_time);
    problem_solution solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time);

    void nWSR(int nwsr) {if (nwsr < 1) _nWSR = 1; else _nWSR = nwsr;};
    double costs(Eigen::VectorXd X);

private:
    real_t H[1156];
    real_t g[34];
    real_t A[510];
    real_t lb[34];
    real_t ub[34];
    real_t lbA[15];
    real_t ubA[15];

    SQProblem *solver;
    int _nWSR = 40;
    double *_cpu_time;

    void update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void copy_xopt();
    void update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution);

    void update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_vectors_bA(problem_parameters *prob_params, problem_solution *prev_qpsolution);

    double sq(double x) {return x * x;};

    void print_quadratic_problem(real_t *primal, real_t *dual) {
        std::cout << "***********PATS Optimization step:**************" << std::endl;
        print_array("H", 1156, H);
        print_array("g", 34, g);
        print_array("A", 510, A);
        print_array("lb", 34, lb);
        print_array("ub", 34, ub);
        print_array("lbA", 15, lbA);
        print_array("ubA", 15, ubA);
        print_array("prim", 34, primal);
        print_array("(-1)*dual", 34 + 15, dual); //dual is printed with inverted sign, the lagrange_multiplier for the constraints have the correct sign however.
        std::cout << "*******************************************" << std::endl;
    }
};
