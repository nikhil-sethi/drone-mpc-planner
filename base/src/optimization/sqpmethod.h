#pragma once
#include <vector>
#include <Eigen/Core>
#include "quadraticoptimizer.h"
#ifdef OCP_DEV
#include <casadi/casadi.hpp>
#endif
#ifdef OPTI_ROSVIS
#include "rosvisualizerinterface.h"
#endif

struct sqp_solver_configuration {
    int max_sqp_iterations;
    double tol_pr;
    double tol_du;
    double min_step_size;

    sqp_solver_configuration(): max_sqp_iterations(30), tol_pr(1e-2), tol_du(1e-2), min_step_size(1e-1) {};
    sqp_solver_configuration(int mi, double tp, double td, double mss) : max_sqp_iterations(mi), tol_pr(tp), tol_du(td), min_step_size(mss) {};
};


std::ostream &operator<<(std::ostream &os, sqp_solver_configuration &sqp_config);
class SQPSolver {
public:
    void init(QuadraticOptimizer *qpsolver);
    void setup(sqp_solver_configuration _config);
    Eigen::VectorXd solve_line_search(problem_parameters *prob_param);

    sqp_solver_configuration convergence_parameter() {
        return config;
    };

    void max_cpu_time(double mcput) {
        _max_cpu_time = mcput;
        if (_max_cpu_time <= 0)
            std::cout << "SQPMethod: Realtime ensurance disabled." << std::endl;
    };

#ifdef OCP_DEV
    void init_casadi(std::string problem_solver_path);
    Eigen::VectorXd solve_casadi(problem_parameters *prob_param);
#endif
#ifdef OPTI_ROSVIS
    void ros_interface(RosVisualizerInterface *interface) {_ros_interface = interface;};
#endif

private:

    bool use_casadi = false;
    sqp_solver_configuration config;
    QuadraticOptimizer *_qpsolver;
    double _max_cpu_time;
    double cpu_time_remaining;

    int min_iterations = 1;



    // Backtracking casadi:
    double sigma = 0;

    Eigen::VectorXd update_optization_guess(Eigen::VectorXd Xopt_prev, Eigen::VectorXd Xopt_new);

    double l1_sum_viol(Eigen::VectorXd z, problem_parameters *prob_params);
    void backtracking_casadi(problem_solution *prev_qpsolution, problem_solution *qpsolution, problem_parameters *prob_params);

    Eigen::VectorXd gradient_lagrangian(problem_solution *prob_sol, problem_parameters *prob_param);
    Eigen::VectorXd return_xopt(Eigen::VectorXd Xopt, Eigen::VectorXd X0);

#ifdef OCP_DEV
    casadi::Function casadi_solver;
    std::map<std::string, casadi::DM> arg;
    std::map<std::string, casadi::DM> res;
#endif
#ifdef OPTI_ROSVIS
    RosVisualizerInterface *_ros_interface;
#endif
};
