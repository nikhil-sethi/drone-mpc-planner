#pragma once
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <qpOASES.hpp>
#include "quadraticoptimizer.h"

USING_NAMESPACE_QPOASES

#include "qpoasesconfig.h"

class TtiQuadraticOptimizer : public QuadraticOptimizer {
public:

    void init();
    void qp_setup(QPSettings qpsettings);
    QPSettings qp_setup() {
        return QPSettings(solver.getOptions());
    };
    Eigen::VectorXd constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    Eigen::VectorXd constraints(Eigen::VectorXd xopt, Eigen::VectorXd param);
    Eigen::MatrixXd constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    problem_solution solve(problem_parameters *prob_params, bool init, double cpu_time);
    problem_solution solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time);

    std::vector<Eigen::VectorXd> trajectory(Eigen::VectorXd xopt, trajectory_type traj_type);

    double costs(Eigen::VectorXd X);

    std::string quadratic_solver_library() { return "qpOases";};

private:
    real_t H[1156];
    real_t g[34];
    real_t A[510];
    real_t lb[34];
    real_t ub[34];
    real_t lbA[15];
    real_t ubA[15];

    SQProblem solver;
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
    };
};
