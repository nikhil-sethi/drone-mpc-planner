#pragma once
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <osqp.h>
#include "quadraticoptimizer.h"

#include "osqpconfig.h"

class InterceptInPlanesQuadraticOptimizer : public QuadraticOptimizer {
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

    ~InterceptInPlanesQuadraticOptimizer() {
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


    c_int P_p[66] = {0, 1, 2, 2, 2, 2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 26, 26, 26, 27, 28, 29, 29, 29, 29, 29, 29, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, };
    c_int P_i[38] = {0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 0, 0, 0, 56, 57, 58, 59, 60, 61, 62, 63, 64, };
    c_float P_x[38];
    c_float q[65];

    c_int A_p[66] = {0, 10, 17, 19, 21, 23, 26, 29, 32, 46, 60, 74, 77, 80, 83, 86, 89, 92, 96, 100, 104, 116, 128, 140, 142, 144, 146, 149, 152, 155, 158, 161, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190, 192, 194, 197, 200, 203, 206, 209, 212, 214, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 236, };
    c_int A_i[236] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 47, 21, 22, 23, 24, 25, 26, 48, 0, 49, 1, 50, 2, 51, 0, 3, 52, 1, 4, 53, 2, 5, 54, 0, 12, 15, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 55, 1, 13, 16, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 56, 2, 14, 17, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 57, 3, 18, 58, 4, 19, 59, 5, 20, 60, 15, 21, 61, 16, 22, 62, 17, 23, 63, 18, 21, 24, 64, 19, 22, 25, 65, 20, 23, 26, 66, 21, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 67, 22, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 68, 23, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 69, 24, 70, 25, 71, 26, 72, 0, 3, 73, 1, 4, 74, 2, 5, 75, 21, 24, 76, 22, 25, 77, 23, 26, 78, 0, 79, 1, 80, 2, 81, 3, 82, 4, 83, 5, 84, 21, 85, 22, 86, 23, 87, 24, 88, 25, 89, 26, 90, 6, 91, 7, 92, 8, 93, 6, 9, 94, 7, 10, 95, 8, 11, 96, 6, 12, 97, 7, 13, 98, 8, 14, 99, 9, 100, 10, 101, 11, 102, 12, 103, 13, 104, 14, 105, 15, 106, 16, 107, 17, 108, 18, 109, 19, 110, 20, 111, };
    c_float A_x[236];

    c_float l[112];
    c_float u[112];
    c_float x0[65];
    c_float lam0[112];

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
    //     print_array("H", 4225, H);
    //     print_array("g", 65, g);
    //     print_array("A", 3055, A);
    //     print_array("lb", 65, lb);
    //     print_array("ub", 65, ub);
    //     print_array("lbA", 47, lbA);
    //     print_array("ubA", 47, ubA);
    //     print_array("prim", 65, primal);
    //     print_array("(-1)*dual", 65 + 47, dual); //dual is printed with inverted sign, the lagrange_multiplier for the constraints have the correct sign however.
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
