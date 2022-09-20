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


    c_int P_p[66] = {0, 1, 1, 1, 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 13, 13, 14, 15, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 18, 19, 20, 20, 20, 20, 20, 20, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, };
    c_int P_i[38] = {0, 0, 0, 0, 0, 0, 0, 16, 17, 18, 19, 20, 21, 0, 0, 0, 34, 34, 34, 34, 34, 34, 34, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, };
    c_float P_x[38];
    c_float q[65];

    c_int A_p[66] = {0, 10, 12, 14, 16, 19, 22, 25, 39, 53, 67, 70, 73, 76, 79, 82, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 106, 109, 112, 115, 118, 121, 123, 125, 127, 134, 137, 140, 143, 147, 151, 155, 167, 179, 191, 193, 195, 197, 200, 203, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 236, };
    c_int A_i[236] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 47, 0, 48, 1, 49, 2, 50, 0, 3, 51, 1, 4, 52, 2, 5, 53, 0, 12, 15, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 54, 1, 13, 16, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 55, 2, 14, 17, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 56, 3, 18, 57, 4, 19, 58, 5, 20, 59, 0, 3, 60, 1, 4, 61, 2, 5, 62, 0, 63, 1, 64, 2, 65, 3, 66, 4, 67, 5, 68, 6, 69, 7, 70, 8, 71, 6, 9, 72, 7, 10, 73, 8, 11, 74, 6, 12, 75, 7, 13, 76, 8, 14, 77, 9, 78, 10, 79, 11, 80, 21, 22, 23, 24, 25, 26, 81, 15, 21, 82, 16, 22, 83, 17, 23, 84, 18, 21, 24, 85, 19, 22, 25, 86, 20, 23, 26, 87, 21, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 88, 22, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 89, 23, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 90, 24, 91, 25, 92, 26, 93, 21, 24, 94, 22, 25, 95, 23, 26, 96, 21, 97, 22, 98, 23, 99, 24, 100, 25, 101, 26, 102, 12, 103, 13, 104, 14, 105, 15, 106, 16, 107, 17, 108, 18, 109, 19, 110, 20, 111, };
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
