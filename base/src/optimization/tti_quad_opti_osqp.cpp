#include "tti_quad_opti_osqp.h"
#include "tti_index.h"

void TtiQuadraticOptimizer::init() {
    settings = reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));

    data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

    Eigen::VectorXd xopt = Eigen::VectorXd(34).setZero();
    Eigen::VectorXd lamg = Eigen::VectorXd(15).setZero();

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

    _H = Eigen::MatrixXd(34, 34);
    _H.setZero();
    _H(0, 0) = (((lamg[0]*((0.5)*(xopt[13]+xopt[13])))+(lamg[1]*((0.5)*(xopt[14]+xopt[14]))))+(lamg[2]*((0.5)*(xopt[15]+xopt[15]))));
    _H(0, 4) = lamg[0];
    _H(0, 5) = lamg[1];
    _H(0, 6) = lamg[2];
    _H(0, 13) = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    _H(0, 14) = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    _H(0, 15) = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    _H(0, 25) = lamg[6];
    _H(0, 26) = lamg[7];
    _H(0, 27) = lamg[8];
    _H(4, 0) = lamg[0];
    _H(5, 0) = lamg[1];
    _H(6, 0) = lamg[2];
    _H(13, 0) = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    _H(14, 0) = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    _H(15, 0) = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    _H(16, 16) = 2e+13;
    _H(17, 17) = 2e+13;
    _H(18, 18) = 2e+13;
    _H(19, 19) = 2e+13;
    _H(20, 20) = 2e+13;
    _H(21, 21) = 2e+13;
    _H(25, 0) = lamg[6];
    _H(26, 0) = lamg[7];
    _H(27, 0) = lamg[8];


    _g = Eigen::VectorXd(34);
    _g.setZero();
    _g[0] = 1;
    _g[16] = (1e+13*(xopt[16]+xopt[16]));
    _g[17] = (1e+13*(xopt[17]+xopt[17]));
    _g[18] = (1e+13*(xopt[18]+xopt[18]));
    _g[19] = (1e+13*(xopt[19]+xopt[19]));
    _g[20] = (1e+13*(xopt[20]+xopt[20]));
    _g[21] = (1e+13*(xopt[21]+xopt[21]));

}


void TtiQuadraticOptimizer::qp_setup(QPSettings qpsettings) {
    qpsettings.apply(settings);
}

Eigen::VectorXd TtiQuadraticOptimizer::constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    return constraints(xopt, param);
}

Eigen::VectorXd TtiQuadraticOptimizer::constraints(Eigen::VectorXd xopt, Eigen::VectorXd param [[maybe_unused]]) {
    Eigen::VectorXd constraints = Eigen::VectorXd(15).setZero();
    constraints[0] = (((xopt[1]+((xopt[0]*(xopt[4]+(xopt[4]+(xopt[0]*xopt[13]))))/2))-xopt[7])+xopt[16]);
    constraints[1] = (((xopt[2]+((xopt[0]*(xopt[5]+(xopt[5]+(xopt[0]*xopt[14]))))/2))-xopt[8])+xopt[17]);
    constraints[2] = (((xopt[3]+((xopt[0]*(xopt[6]+(xopt[6]+(xopt[0]*xopt[15]))))/2))-xopt[9])+xopt[18]);
    constraints[3] = (((xopt[4]+((xopt[0]*(xopt[13]+xopt[13]))/2))-xopt[10])+xopt[19]);
    constraints[4] = (((xopt[5]+((xopt[0]*(xopt[14]+xopt[14]))/2))-xopt[11])+xopt[20]);
    constraints[5] = (((xopt[6]+((xopt[0]*(xopt[15]+xopt[15]))/2))-xopt[12])+xopt[21]);
    constraints[6] = ((xopt[22]+((xopt[0]*(xopt[25]+xopt[25]))/2))-xopt[28]);
    constraints[7] = ((xopt[23]+((xopt[0]*(xopt[26]+xopt[26]))/2))-xopt[29]);
    constraints[8] = ((xopt[24]+((xopt[0]*(xopt[27]+xopt[27]))/2))-xopt[30]);
    constraints[9] = (xopt[25]-xopt[31]);
    constraints[10] = (xopt[26]-xopt[32]);
    constraints[11] = (xopt[27]-xopt[33]);
    constraints[12] = (xopt[7]-xopt[28]);
    constraints[13] = (xopt[8]-xopt[29]);
    constraints[14] = (xopt[9]-xopt[30]);

    return constraints;
}

Eigen::MatrixXd TtiQuadraticOptimizer::constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::MatrixXd dconstraints = Eigen::MatrixXd(15, 34).setZero();
    dconstraints(0, 0) = (0.5*((xopt[4]+(xopt[4]+(xopt[0]*xopt[13])))+(xopt[0]*xopt[13])));
    dconstraints(0, 1) = 1;
    dconstraints(0, 4) = xopt[0];
    dconstraints(0, 7) = -1;
    dconstraints(0, 13) = (0.5*sq(xopt[0]));
    dconstraints(0, 16) = 1;
    dconstraints(1, 0) = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[14])))+(xopt[0]*xopt[14])));
    dconstraints(1, 2) = 1;
    dconstraints(1, 5) = xopt[0];
    dconstraints(1, 8) = -1;
    dconstraints(1, 14) = (0.5*sq(xopt[0]));
    dconstraints(1, 17) = 1;
    dconstraints(2, 0) = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[15])))+(xopt[0]*xopt[15])));
    dconstraints(2, 3) = 1;
    dconstraints(2, 6) = xopt[0];
    dconstraints(2, 9) = -1;
    dconstraints(2, 15) = (0.5*sq(xopt[0]));
    dconstraints(2, 18) = 1;
    dconstraints(3, 0) = (0.5*(xopt[13]+xopt[13]));
    dconstraints(3, 4) = 1;
    dconstraints(3, 10) = -1;
    dconstraints(3, 13) = xopt[0];
    dconstraints(3, 19) = 1;
    dconstraints(4, 0) = (0.5*(xopt[14]+xopt[14]));
    dconstraints(4, 5) = 1;
    dconstraints(4, 11) = -1;
    dconstraints(4, 14) = xopt[0];
    dconstraints(4, 20) = 1;
    dconstraints(5, 0) = (0.5*(xopt[15]+xopt[15]));
    dconstraints(5, 6) = 1;
    dconstraints(5, 12) = -1;
    dconstraints(5, 15) = xopt[0];
    dconstraints(5, 21) = 1;
    dconstraints(6, 0) = (0.5*(xopt[25]+xopt[25]));
    dconstraints(6, 22) = 1;
    dconstraints(6, 25) = xopt[0];
    dconstraints(6, 28) = -1;
    dconstraints(7, 0) = (0.5*(xopt[26]+xopt[26]));
    dconstraints(7, 23) = 1;
    dconstraints(7, 26) = xopt[0];
    dconstraints(7, 29) = -1;
    dconstraints(8, 0) = (0.5*(xopt[27]+xopt[27]));
    dconstraints(8, 24) = 1;
    dconstraints(8, 27) = xopt[0];
    dconstraints(8, 30) = -1;
    dconstraints(9, 25) = 1;
    dconstraints(9, 31) = -1;
    dconstraints(10, 26) = 1;
    dconstraints(10, 32) = -1;
    dconstraints(11, 27) = 1;
    dconstraints(11, 33) = -1;
    dconstraints(12, 7) = 1;
    dconstraints(12, 28) = -1;
    dconstraints(13, 8) = 1;
    dconstraints(13, 29) = -1;
    dconstraints(14, 9) = 1;
    dconstraints(14, 30) = -1;

    return dconstraints;
}

void TtiQuadraticOptimizer::update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lamg = prev_qpsolution->lagrange_multiplier;

    P_x[0] = (((lamg[0]*((0.5)*(xopt[13]+xopt[13])))+(lamg[1]*((0.5)*(xopt[14]+xopt[14]))))+(lamg[2]*((0.5)*(xopt[15]+xopt[15]))));
    P_x[1] = lamg[0];
    P_x[2] = lamg[1];
    P_x[3] = lamg[2];
    P_x[4] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    P_x[5] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    P_x[6] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    P_x[7] = 2e+13;
    P_x[8] = 2e+13;
    P_x[9] = 2e+13;
    P_x[10] = 2e+13;
    P_x[11] = 2e+13;
    P_x[12] = 2e+13;
    P_x[13] = lamg[6];
    P_x[14] = lamg[7];
    P_x[15] = lamg[8];


    float regularisation_value = sqrt(norm_1(P_p, P_x, 34) * norm_inf(P_p, P_i, P_x, 34));
    add_diag(P_p, P_i, P_x, 34, regularisation_value);
}

void TtiQuadraticOptimizer::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    q[0] = 1;
    q[16] = (1e+13*(xopt[16]+xopt[16]));
    q[17] = (1e+13*(xopt[17]+xopt[17]));
    q[18] = (1e+13*(xopt[18]+xopt[18]));
    q[19] = (1e+13*(xopt[19]+xopt[19]));
    q[20] = (1e+13*(xopt[20]+xopt[20]));
    q[21] = (1e+13*(xopt[21]+xopt[21]));

}

void TtiQuadraticOptimizer::update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    A_x[0] = (0.5*((xopt[4]+(xopt[4]+(xopt[0]*xopt[13])))+(xopt[0]*xopt[13])));
    A_x[1] = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[14])))+(xopt[0]*xopt[14])));
    A_x[2] = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[15])))+(xopt[0]*xopt[15])));
    A_x[3] = (0.5*(xopt[13]+xopt[13]));
    A_x[4] = (0.5*(xopt[14]+xopt[14]));
    A_x[5] = (0.5*(xopt[15]+xopt[15]));
    A_x[6] = (0.5*(xopt[25]+xopt[25]));
    A_x[7] = (0.5*(xopt[26]+xopt[26]));
    A_x[8] = (0.5*(xopt[27]+xopt[27]));
    A_x[9] = 1;
    A_x[10] = 1;
    A_x[11] = 1;
    A_x[12] = 1;
    A_x[13] = 1;
    A_x[14] = 1;
    A_x[15] = 1;
    A_x[16] = xopt[0];
    A_x[17] = 1;
    A_x[18] = 1;
    A_x[19] = xopt[0];
    A_x[20] = 1;
    A_x[21] = 1;
    A_x[22] = xopt[0];
    A_x[23] = 1;
    A_x[24] = 1;
    A_x[25] = -1;
    A_x[26] = 1;
    A_x[27] = 1;
    A_x[28] = -1;
    A_x[29] = 1;
    A_x[30] = 1;
    A_x[31] = -1;
    A_x[32] = 1;
    A_x[33] = 1;
    A_x[34] = -1;
    A_x[35] = 1;
    A_x[36] = -1;
    A_x[37] = 1;
    A_x[38] = -1;
    A_x[39] = 1;
    A_x[40] = (0.5*sq(xopt[0]));
    A_x[41] = xopt[0];
    A_x[42] = 1;
    A_x[43] = (0.5*sq(xopt[0]));
    A_x[44] = xopt[0];
    A_x[45] = 1;
    A_x[46] = (0.5*sq(xopt[0]));
    A_x[47] = xopt[0];
    A_x[48] = 1;
    A_x[49] = 1;
    A_x[50] = 1;
    A_x[51] = 1;
    A_x[52] = 1;
    A_x[53] = 1;
    A_x[54] = 1;
    A_x[55] = 1;
    A_x[56] = 1;
    A_x[57] = 1;
    A_x[58] = 1;
    A_x[59] = 1;
    A_x[60] = 1;
    A_x[61] = 1;
    A_x[62] = 1;
    A_x[63] = 1;
    A_x[64] = 1;
    A_x[65] = 1;
    A_x[66] = 1;
    A_x[67] = xopt[0];
    A_x[68] = 1;
    A_x[69] = 1;
    A_x[70] = xopt[0];
    A_x[71] = 1;
    A_x[72] = 1;
    A_x[73] = xopt[0];
    A_x[74] = 1;
    A_x[75] = 1;
    A_x[76] = -1;
    A_x[77] = -1;
    A_x[78] = 1;
    A_x[79] = -1;
    A_x[80] = -1;
    A_x[81] = 1;
    A_x[82] = -1;
    A_x[83] = -1;
    A_x[84] = 1;
    A_x[85] = -1;
    A_x[86] = 1;
    A_x[87] = -1;
    A_x[88] = 1;
    A_x[89] = -1;
    A_x[90] = 1;

}

void TtiQuadraticOptimizer::update_vectors_bA(problem_parameters *prob_params[[maybe_unused]], problem_solution *prev_qpsolution) {
    // Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    // Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lbg = prob_params->lbg;
    Eigen::VectorXd ubg = prob_params->ubg;

    l[0] = lbg[0];
    l[1] = lbg[1];
    l[2] = lbg[2];
    l[3] = lbg[3];
    l[4] = lbg[4];
    l[5] = lbg[5];
    l[6] = lbg[6];
    l[7] = lbg[7];
    l[8] = lbg[8];
    l[9] = lbg[9];
    l[10] = lbg[10];
    l[11] = lbg[11];
    l[12] = lbg[12];
    l[13] = lbg[13];
    l[14] = lbg[14];
    u[0] = ubg[0];
    u[1] = ubg[1];
    u[2] = ubg[2];
    u[3] = ubg[3];
    u[4] = ubg[4];
    u[5] = ubg[5];
    u[6] = ubg[6];
    u[7] = ubg[7];
    u[8] = ubg[8];
    u[9] = ubg[9];
    u[10] = ubg[10];
    u[11] = ubg[11];
    u[12] = ubg[12];
    u[13] = ubg[13];
    u[14] = ubg[14];


    for (uint i = 0; i < 15; i++) {
        u[i] -= prev_qpsolution->Z[34 + i];
        l[i] -= prev_qpsolution->Z[34 + i];
    }
}

void TtiQuadraticOptimizer::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        l[15 + lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        u[15 + ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

void TtiQuadraticOptimizer::update_init_guess(problem_solution *prev_qpsolution) {

    for (uint i = 0; i < 34; i++) {
        x0[i] = prev_qpsolution->Xopt[i];
    }
    for (uint i = 0; i < 15; i++) {
        lam0[i] = prev_qpsolution->lagrange_multiplier[i];
    }
}

problem_solution TtiQuadraticOptimizer::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution TtiQuadraticOptimizer::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init [[maybe_unused]], double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);
    update_init_guess(prev_qpsolution);

    if (!setup) {
        if (data) {
            data->n = 34;
            data->m = 49;
            data->P = csc_matrix(data->n, data->n, 16, P_x, P_i, P_p);
            data->q = q;
            data->A = csc_matrix(data->m, data->n, 91, A_x, A_i, A_p);
            data->l = l;
            data->u = u;
        }
        c_int exitflag = osqp_setup(&work, data, settings);
        if (!exitflag)
            setup = true;

    } else {
        osqp_update_P(work, P_x, OSQP_NULL, 16);
        osqp_update_lin_cost(work, q);
        osqp_update_A(work, A_x, OSQP_NULL, 91);
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
    // print_array("H", 34 * 34, H);

    return ret;
}


std::vector<Eigen::VectorXd> TtiQuadraticOptimizer::trajectory(Eigen::VectorXd xopt, trajectory_type traj_type) {

    std::vector<Eigen::VectorXd> traj;

    if (traj_type == state_trajectory_t) {
        for (uint k = 0; k <= n_steps; k++) {
            Eigen::VectorXd sample = Eigen::VectorXd(N_DRONE_STATES);
            for (uint i = 0; i < N_DRONE_STATES; i++) {
                sample[i] = xopt[state_trajectory_first + k * N_DRONE_STATES + i];
            }
            traj.push_back(sample);
        }
    } else if (traj_type == input_trajectory_t) {
        for (uint k = 0; k < n_steps; k++) {
            Eigen::VectorXd sample = Eigen::VectorXd(N_DRONE_INPUTS);
            for (uint i = 0; i < N_DRONE_INPUTS; i++) {
                sample[i] = xopt[input_trajectory_first + k * N_DRONE_INPUTS + i];
            }
            traj.push_back(sample);
        }
    } else if (traj_type == virtual_input_trajectory_t) {
        for (uint k = 0; k <= n_steps; k++) {
            Eigen::VectorXd sample = Eigen::VectorXd(N_DRONE_INPUTS);
            for (uint i = 0; i < N_DRONE_INPUTS; i++) {
                sample[i] = xopt[virtual_input_trajectory_first + k * N_DRONE_STATES + i];
            }
            traj.push_back(sample);
        }
    }

    return traj;
}
