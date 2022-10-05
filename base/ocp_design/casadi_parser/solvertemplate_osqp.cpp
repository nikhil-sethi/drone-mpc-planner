#include "solvertemplate.h"
#include "problem_index.h"

void SolverTemplate::init() {
    settings = reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));

    data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

    Eigen::VectorXd xopt = Eigen::VectorXd(N_XOPTS).setZero();
    Eigen::VectorXd lamg = Eigen::VectorXd(N_CONSTRAINTS).setZero();

    if (settings) {
        osqp_set_default_settings(settings);
        // settings->rho = 0.1; // default: 0.1
        // settings->sigma = 1e-6; //default: 1e-6
        // settings->max_iter = 4000; // default: 4000
        // settings->eps_abs = 1e-2; // default: 1e-3
        // settings->eps_rel = 1e-9; // default: 1e-3
        // settings->eps_prim_inf = 1e-1; // default: 1-4
        // settings->eps_dual_inf = 1e-1; // default: 1-4
        // settings->alpha = 1.9; // default: 1.6
        // settings->linsys_solver = QDLDL_SOLVER; // default: QDLDL_SOLVER
        // // settings->delta = 1e-3; // default: 1e-6
        // settings->polish = true; // default: false
        // settings->polish_refine_iter = 3; // default: 3
        // settings->verbose = 0; // default:
        // settings->scaled_termination = true; // default: false
        // settings->check_termination = 25; // default: 25
        settings->warm_start = 0; // default: 1
        // settings->scaling = 1; //default 10
        // settings->adaptive_rho = true; // default: true
        // settings->adaptive_rho_interval = 1; // default: 0
        // settings->adaptive_rho_tolerance = 1; // default: 5
        // settings->adaptive_rho_fraction = 1; // default: 0.4
        // settings->time_limit = 0.0; // default: 0.1
    }

    _H = Eigen::MatrixXd(N_XOPTS, N_XOPTS);
    _H.setZero();
    /* INIT H DENSE PLACEHOLDER*/

    _g = Eigen::VectorXd(N_XOPTS);
    _g.setZero();
    /* INIT G PLACEHOLDER*/
}


void SolverTemplate::qp_setup(QPSettings qpsettings) {
    qpsettings.apply(settings);
}

Eigen::VectorXd SolverTemplate::constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    return constraints(xopt, param);
}

Eigen::VectorXd SolverTemplate::constraints(Eigen::VectorXd xopt, Eigen::VectorXd param [[maybe_unused]]) {
    Eigen::VectorXd constraints = Eigen::VectorXd(N_CONSTRAINTS).setZero();
    /* CONSTRAINTS PLACEHOLDER*/
    return constraints;
}

Eigen::MatrixXd SolverTemplate::constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::MatrixXd dconstraints = Eigen::MatrixXd(N_CONSTRAINTS, N_XOPTS).setZero();
    /* CONSTRAINT_DERIVATIVES PLACEHOLDER*/
    return dconstraints;
}

void SolverTemplate::update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lamg = prev_qpsolution->lagrange_multiplier;

    /* UPDATE P PLACEHOLDER*/

    float regularisation_value = sqrt(norm_1(P_p, P_x, N_XOPTS) * norm_inf(P_p, P_i, P_x, N_XOPTS));
    add_diag(P_p, P_i, P_x, N_XOPTS, regularisation_value);
}

void SolverTemplate::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    /* INIT q PLACEHOLDER*/
}

void SolverTemplate::update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    /* UPDATE A PLACEHOLDER*/
}

void SolverTemplate::update_vectors_bA(problem_parameters *prob_params[[maybe_unused]], problem_solution *prev_qpsolution) {
    // Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    // Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lbg = prob_params->lbg;
    Eigen::VectorXd ubg = prob_params->ubg;

    /* UPDATE bA PLACEHOLDER*/

    for (uint i = 0; i < N_CONSTRAINTS; i++) {
        u[i] -= prev_qpsolution->Z[N_XOPTS + i];
        l[i] -= prev_qpsolution->Z[N_XOPTS + i];
    }
}

void SolverTemplate::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        l[N_CONSTRAINTS + lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        u[N_CONSTRAINTS + ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

void SolverTemplate::update_init_guess(problem_solution *prev_qpsolution) {

    for (uint i = 0; i < N_XOPTS; i++) {
        x0[i] = prev_qpsolution->Xopt[i];
    }
    for (uint i = 0; i < N_CONSTRAINTS; i++) {
        lam0[i] = prev_qpsolution->lagrange_multiplier[i];
    }
}

problem_solution SolverTemplate::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution SolverTemplate::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init [[maybe_unused]], double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);
    update_init_guess(prev_qpsolution);

    if (!setup) {
        if (data) {
            data->n = N_XOPTS;
            data->m = N_XOPTS + N_CONSTRAINTS;
            data->P = csc_matrix(data->n, data->n, P_NNZ, P_x, P_i, P_p);
            data->q = q;
            data->A = csc_matrix(data->m, data->n, A_NNZ, A_x, A_i, A_p);
            data->l = l;
            data->u = u;
        }
        c_int exitflag = osqp_setup(&work, data, settings);
        if (!exitflag)
            setup = true;

    } else {
        osqp_update_P(work, P_x, OSQP_NULL, P_NNZ);
        osqp_update_lin_cost(work, q);
        osqp_update_A(work, A_x, OSQP_NULL, A_NNZ);
        osqp_update_bounds(work, l, u);
        osqp_update_time_limit(work, static_cast<float>(cpu_time));
    }

    osqp_warm_start(work, x0, lam0);
    osqp_solve(work);

    c_float *primal = work->solution->x;
    Eigen::VectorXd xopt = prev_qpsolution->Xopt + Eigen::Map<Eigen::VectorXd>(primal, work->data->n, 1);;

    c_float *dual = work->solution->y;
    Eigen::VectorXd lagrange_multiplier = -Eigen::Map<Eigen::VectorXd>(dual, work->data->m, 1);

    c_float obj_val = work->info->obj_val;
    c_int status_val = work->info->status_val;

    // print_array("Px", 38, P_x);
    // print_array("Pi", 38, P_i);
    // print_array("Pp", 65 + 1, P_p);
    // print_array("q", 65, q);
    // print_array("Ax", 236, A_x);
    // print_array("Ai", 236, A_i);
    // print_array("Ap", 65 + 1, A_p);
    // print_array("l", 112, l);
    // print_array("u", 112, u);
    // print_array("x0", 65, x0);
    // print_eigenvector("xopt", xopt);
    // print_eigenvector("lam", lagrange_multiplier);
    // std::cout << "Objective value: " << obj_val << std::endl;

    problem_solution ret = problem_solution(xopt, constraints(xopt, prob_params->param), lagrange_multiplier, obj_val, qp_return_status(status_val));
    // print_quadratic_problem_design(*prob_params, ret);
    // print_quadratic_problem(primal, _dual);
    // print_array("H", N_XOPTS * N_XOPTS, H);

    return ret;
}
