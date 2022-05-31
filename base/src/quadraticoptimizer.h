#pragma once
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <limits>


struct problem_parameters {
    Eigen::VectorXd X0;
    Eigen::VectorXd lbg;
    Eigen::VectorXd ubg;
    Eigen::VectorXd lbx;
    Eigen::VectorXd ubx;
    Eigen::VectorXd param;

    problem_parameters() {
        X0 = {};
        lbg = {};
        ubg = {};
        lbx = {};
        ubx = {};
        param = {};
    }

    problem_parameters(Eigen::VectorXd _X0, Eigen::VectorXd _lbg, Eigen::VectorXd _ubg, Eigen::VectorXd _lbx, Eigen::VectorXd _ubx, Eigen::VectorXd _param): X0(_X0), lbg(_lbg), ubg(_ubg), lbx(_lbx), ubx(_ubx), param(_param) {};

    problem_parameters(int n_optvars, int n_constraints, int n_params) {
        X0 = Eigen::VectorXd(n_optvars).setZero();
        lbg = Eigen::VectorXd(n_constraints).setZero();
        ubg = Eigen::VectorXd(n_constraints).setZero();
        ubx  = Eigen::VectorXd(n_optvars).setZero();
        lbx  = Eigen::VectorXd(n_optvars).setZero();
        param  = Eigen::VectorXd(n_params).setZero();
    }
};

struct problem_solution {
    Eigen::VectorXd Xopt;
    Eigen::VectorXd constraints_val;
    Eigen::VectorXd lagrange_multiplier;
    Eigen::VectorXd Z;
    double costs;
    int status;

    problem_solution() {
        Xopt = {};
        constraints_val = {};
        lagrange_multiplier = {};
        costs = std::numeric_limits<double>::infinity();
        status = -1;
        Z = {};
    };
    problem_solution(Eigen::VectorXd xopt, Eigen::VectorXd consts, Eigen::VectorXd _lagrange, double objval, int s): Xopt(xopt), constraints_val(consts), lagrange_multiplier(_lagrange), costs(objval), status(s) {
        Z = Eigen::VectorXd(xopt.size() + consts.size());
        Z.segment(0, xopt.size()) = xopt;
        Z.segment(xopt.size(), consts.size()) = consts;
    };

    problem_solution(problem_parameters *prob_param) {
        int n_optvars = prob_param->X0.size();
        int n_constraints = prob_param->lbg.size();
        Xopt = prob_param->X0;
        constraints_val = Eigen::VectorXd(n_constraints).setZero();
        lagrange_multiplier = Eigen::VectorXd(n_constraints).setZero();
        Z = Eigen::VectorXd(n_optvars + n_constraints).setZero();
        Z.segment(0, n_optvars) = prob_param->X0;
        status = -1;
    };
};

class QuadraticOptimizer {
public:
    virtual void init() = 0;
    virtual Eigen::VectorXd constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution) = 0;
    virtual Eigen::MatrixXd constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution) = 0;
    virtual problem_solution solve(problem_parameters *prob_params, bool init, double cpu_time) = 0;
    virtual problem_solution solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time) = 0;

    double costs(Eigen::VectorXd X) {
        auto ret = 0.5 * X.transpose() * _H * X + X.transpose() * _g;
        return ret[0];
    };

    Eigen::VectorXd dJ(Eigen::VectorXd X) {
        return _H * X + _g;
    }

    void print_eigenvector(std::string name, Eigen::VectorXd vec) {
        if (vec.size() > 0) {
            std::cout << name << ": [";

            for (uint k = 0; k < vec.size() - 1; k++)
                std::cout << vec[k] << ", ";
            std::cout << vec[vec.size() - 1] << "]" << std::endl;
        }
    }

    void print_array(std::string name, long long int n, double *vec) {
        std::cout << name << ": [";
        for (long long int k = 0; k < n - 1; k++)
            std::cout << vec[k] << ", ";
        std::cout << vec[n - 1] << "]" << std::endl;
    }

    void print_quadratic_problem_design(problem_parameters prob_params, problem_solution prob_sol) {
        std::cout << "***********Problem optimization step (PATS):**************" << std::endl;
        print_eigenvector("idx", Eigen::VectorXd::LinSpaced(prob_params.X0.size(), 0, prob_params.X0.size() - 1));
        print_eigenvector("X0", prob_params.X0);
        print_eigenvector("lbg", prob_params.lbg);
        print_eigenvector("ubg", prob_params.ubg);
        print_eigenvector("lbx", prob_params.lbx);
        print_eigenvector("ubx", prob_params.ubx);
        print_eigenvector("param", prob_params.param);
        print_eigenvector("Xopt", prob_sol.Xopt);
        std::cout << "solver_status: " << prob_sol.status << std::endl;
        std::cout << "*******************************************" << std::endl;

    }

    Eigen::MatrixXd hessian() {
        return _H;
    }

protected:
    Eigen::MatrixXd _H_orig; // H from Taylor development of the cost function
    Eigen::MatrixXd _H; // H used to solve quadratic subproblem
    Eigen::VectorXd _g;
};
