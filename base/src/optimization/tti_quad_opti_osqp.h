#pragma once
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <osqp.h>
#include "quadraticoptimizer.h"

#include "osqpconfig.h"

class TtiQuadraticOptimizer : public QuadraticOptimizer {
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

    std::vector<Eigen::VectorXd> trajectory(Eigen::VectorXd xopt, trajectory_type traj_type);

    double costs(Eigen::VectorXd X);

    std::string quadratic_solver_library() { return "osqp";};

    ~TtiQuadraticOptimizer() {
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


    c_int P_p[35] = {0, 1, 1, 1, 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16, };
    c_int P_i[16] = {0, 0, 0, 0, 0, 0, 0, 16, 17, 18, 19, 20, 21, 0, 0, 0, };
    c_float P_x[16];
    c_float q[34];

    c_int A_p[35] = {0, 10, 12, 14, 16, 19, 22, 25, 28, 31, 34, 36, 38, 40, 43, 46, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 70, 73, 76, 79, 82, 85, 87, 89, 91, };
    c_int A_i[91] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 15, 0, 16, 1, 17, 2, 18, 0, 3, 19, 1, 4, 20, 2, 5, 21, 0, 12, 22, 1, 13, 23, 2, 14, 24, 3, 25, 4, 26, 5, 27, 0, 3, 28, 1, 4, 29, 2, 5, 30, 0, 31, 1, 32, 2, 33, 3, 34, 4, 35, 5, 36, 6, 37, 7, 38, 8, 39, 6, 9, 40, 7, 10, 41, 8, 11, 42, 6, 12, 43, 7, 13, 44, 8, 14, 45, 9, 46, 10, 47, 11, 48, };
    c_float A_x[91];

    c_float l[49];
    c_float u[49];
    c_float x0[34];
    c_float lam0[49];

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
    //     print_array("H", 1156, H);
    //     print_array("g", 34, g);
    //     print_array("A", 510, A);
    //     print_array("lb", 34, lb);
    //     print_array("ub", 34, ub);
    //     print_array("lbA", 15, lbA);
    //     print_array("ubA", 15, ubA);
    //     print_array("prim", 34, primal);
    //     print_array("(-1)*dual", 34 + 15, dual); //dual is printed with inverted sign, the lagrange_multiplier for the constraints have the correct sign however.
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
