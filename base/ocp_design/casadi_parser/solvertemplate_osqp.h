#pragma once
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <osqp.h>
#include "quadraticoptimizer.h"

#include "osqpconfig.h"

class SolverTemplate : public QuadraticOptimizer {
public:
    void init();
    void qp_setup(QPSettings qpsettings);
    QPSettings qp_setup() {
        return QPSettings(settings);
    };

    Eigen::VectorXd constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    Eigen::VectorXd constraints(Eigen::VectorXd xopt, Eigen::VectorXd param);
    Eigen::MatrixXd constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    problem_solution solve(problem_parameters *prob_params, bool init, double cpu_time);
    problem_solution solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time);

    double costs(Eigen::VectorXd X);

    std::string quadratic_solver_library() { return "osqp";};

    ~SolverTemplate() {
        osqp_cleanup(work);
        if (settings)
            c_free(settings);
        if (data) {
            c_free(data);
        }
    }


private:
    OSQPWorkspace *work;
    OSQPSettings *settings;
    OSQPData *data;
    bool setup = false;


    /* P_P AND P_I PLACEHOLDER*/
    c_float q[N_XOPTS];

    /* INIT A_P AND A_I PLACEHOLDER*/

    c_float l[N_CONSTRAINTS + N_XOPTS];
    c_float u[N_CONSTRAINTS + N_XOPTS];
    c_float x0[N_XOPTS];
    c_float lam0[N_CONSTRAINTS + N_XOPTS];

    void update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void copy_xopt();
    void update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution);

    void update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_vectors_bA(problem_parameters *prob_params, problem_solution *prev_qpsolution);
    void update_init_guess(problem_solution *prev_qpsolution);


    double sq(double x) {return x * x;};

    // void print_quadratic_problem(real_t *primal, real_t *dual) {
    //     std::cout << "***********PATS Optimization step:**************" << std::endl;
    //     print_array("H", N_DIMS_H, H);
    //     print_array("g", N_XOPTS, g);
    //     print_array("A", N_DIMS_A, A);
    //     print_array("lb", N_XOPTS, lb);
    //     print_array("ub", N_XOPTS, ub);
    //     print_array("lbA", N_CONSTRAINTS, lbA);
    //     print_array("ubA", N_CONSTRAINTS, ubA);
    //     print_array("prim", N_XOPTS, primal);
    //     print_array("(-1)*dual", N_XOPTS + N_CONSTRAINTS, dual); //dual is printed with inverted sign, the lagrange_multiplier for the constraints have the correct sign however.
    //     std::cout << "*******************************************" << std::endl;
    // }


    float norm_1(c_int *Pp, c_float *Px, c_int nx) {
        std::vector<float> max_cands = {};
        for (int p = 0; p < nx; p++) {
            float max_cand = 0;
            for (int k = Pp[p]; k < Pp[p + 1]; k++) {
                max_cand += std::abs(static_cast<float>(Px[k]));
            }
            max_cands.push_back(max_cand);
        }
        return *std::max_element(std::begin(max_cands), std::end(max_cands));
    }

    float norm_inf(c_int *Pp, c_int *Pi, c_float *Px, c_int nx) {
        Eigen::VectorXd max_cands = Eigen::VectorXd(nx).setZero();
        for (int p = 0; p < nx; p++) {
            for (int k = Pp[p]; k < Pp[p + 1]; k++) {
                max_cands[Pi[k]] += std::abs(Px[k]);
            }
        }
        return max_cands.maxCoeff();
    }

    void add_diag(c_int *Pp, c_int *Pi, c_float *Px, c_int nx, float alpha) {
        int j = 0;
        for (int p = 0; p < nx; p++) {
            for (int k = Pp[p]; k < Pp[p + 1]; k++) {
                if (j == Pi[k])
                    Px[k] += static_cast<c_float>(alpha);
            }
            j++;
        }
    }

    int qp_return_status(int solver_status) {
        if (solver_status == OSQP_SOLVED  || solver_status == OSQP_SOLVED_INACCURATE)
            return 0;
        else
            return 1;

    }

};
