#include "tti_quad_opti_qpoases.h"
#include <intercept_in_planes_index.h>

void TtiQuadraticOptimizer::init() {
    solver = SQProblem(34, 15);

    Options options;
    options.setToDefault();
    options.printLevel = PL_NONE;

    // options.enableRegularisation = BT_TRUE;
    Eigen::VectorXd xopt = Eigen::VectorXd(34).setZero();

    solver.setOptions(options);
    memset(H, 0, sizeof(H));
    Eigen::VectorXd lamg = Eigen::VectorXd(15).setZero();
    H[0] = (((lamg[0]*((0.5)*(xopt[13]+xopt[13])))+(lamg[1]*((0.5)*(xopt[14]+xopt[14]))))+(lamg[2]*((0.5)*(xopt[15]+xopt[15]))));
    H[4] = lamg[0];
    H[5] = lamg[1];
    H[6] = lamg[2];
    H[13] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[14] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[15] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[25] = lamg[6];
    H[26] = lamg[7];
    H[27] = lamg[8];
    H[136] = lamg[0];
    H[170] = lamg[1];
    H[204] = lamg[2];
    H[442] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[476] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[510] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[560] = 2e+13;
    H[595] = 2e+13;
    H[630] = 2e+13;
    H[665] = 2e+13;
    H[700] = 2e+13;
    H[735] = 2e+13;
    H[850] = lamg[6];
    H[884] = lamg[7];
    H[918] = lamg[8];


    memset(g, 0, sizeof(g));
    g[0] = 1;
    g[16] = (1e+13*(xopt[16]+xopt[16]));
    g[17] = (1e+13*(xopt[17]+xopt[17]));
    g[18] = (1e+13*(xopt[18]+xopt[18]));
    g[19] = (1e+13*(xopt[19]+xopt[19]));
    g[20] = (1e+13*(xopt[20]+xopt[20]));
    g[21] = (1e+13*(xopt[21]+xopt[21]));


    _H = Eigen::Map<Eigen::Matrix<double, 34, 34, Eigen::RowMajor>>(H);
    _g = Eigen::Map<Eigen::VectorXd>(g, 34);
}

void TtiQuadraticOptimizer::qp_setup(QPSettings qpsettings [[maybe_unused]]) {

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

    H[0] = (((lamg[0]*((0.5)*(xopt[13]+xopt[13])))+(lamg[1]*((0.5)*(xopt[14]+xopt[14]))))+(lamg[2]*((0.5)*(xopt[15]+xopt[15]))));
    H[4] = lamg[0];
    H[5] = lamg[1];
    H[6] = lamg[2];
    H[13] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[14] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[15] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[25] = lamg[6];
    H[26] = lamg[7];
    H[27] = lamg[8];
    H[136] = lamg[0];
    H[170] = lamg[1];
    H[204] = lamg[2];
    H[442] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[476] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[510] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[560] = 2e+13;
    H[595] = 2e+13;
    H[630] = 2e+13;
    H[665] = 2e+13;
    H[700] = 2e+13;
    H[735] = 2e+13;
    H[850] = lamg[6];
    H[884] = lamg[7];
    H[918] = lamg[8];

}

void TtiQuadraticOptimizer::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    g[0] = 1;
    g[16] = (1e+13*(xopt[16]+xopt[16]));
    g[17] = (1e+13*(xopt[17]+xopt[17]));
    g[18] = (1e+13*(xopt[18]+xopt[18]));
    g[19] = (1e+13*(xopt[19]+xopt[19]));
    g[20] = (1e+13*(xopt[20]+xopt[20]));
    g[21] = (1e+13*(xopt[21]+xopt[21]));

}

void TtiQuadraticOptimizer::update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    A[0] = (0.5*((xopt[4]+(xopt[4]+(xopt[0]*xopt[13])))+(xopt[0]*xopt[13])));
    A[1] = 1;
    A[4] = xopt[0];
    A[7] = -1;
    A[13] = (0.5*sq(xopt[0]));
    A[16] = 1;
    A[34] = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[14])))+(xopt[0]*xopt[14])));
    A[36] = 1;
    A[39] = xopt[0];
    A[42] = -1;
    A[48] = (0.5*sq(xopt[0]));
    A[51] = 1;
    A[68] = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[15])))+(xopt[0]*xopt[15])));
    A[71] = 1;
    A[74] = xopt[0];
    A[77] = -1;
    A[83] = (0.5*sq(xopt[0]));
    A[86] = 1;
    A[102] = (0.5*(xopt[13]+xopt[13]));
    A[106] = 1;
    A[112] = -1;
    A[115] = xopt[0];
    A[121] = 1;
    A[136] = (0.5*(xopt[14]+xopt[14]));
    A[141] = 1;
    A[147] = -1;
    A[150] = xopt[0];
    A[156] = 1;
    A[170] = (0.5*(xopt[15]+xopt[15]));
    A[176] = 1;
    A[182] = -1;
    A[185] = xopt[0];
    A[191] = 1;
    A[204] = (0.5*(xopt[25]+xopt[25]));
    A[226] = 1;
    A[229] = xopt[0];
    A[232] = -1;
    A[238] = (0.5*(xopt[26]+xopt[26]));
    A[261] = 1;
    A[264] = xopt[0];
    A[267] = -1;
    A[272] = (0.5*(xopt[27]+xopt[27]));
    A[296] = 1;
    A[299] = xopt[0];
    A[302] = -1;
    A[331] = 1;
    A[337] = -1;
    A[366] = 1;
    A[372] = -1;
    A[401] = 1;
    A[407] = -1;
    A[415] = 1;
    A[436] = -1;
    A[450] = 1;
    A[471] = -1;
    A[485] = 1;
    A[506] = -1;

}

void TtiQuadraticOptimizer::update_vectors_bA(problem_parameters *prob_params[[maybe_unused]], problem_solution *prev_qpsolution) {
    // Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    // Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lbg = prob_params->lbg;
    Eigen::VectorXd ubg = prob_params->ubg;

    lbA[0] = lbg[0];
    lbA[1] = lbg[1];
    lbA[2] = lbg[2];
    lbA[3] = lbg[3];
    lbA[4] = lbg[4];
    lbA[5] = lbg[5];
    lbA[6] = lbg[6];
    lbA[7] = lbg[7];
    lbA[8] = lbg[8];
    lbA[9] = lbg[9];
    lbA[10] = lbg[10];
    lbA[11] = lbg[11];
    lbA[12] = lbg[12];
    lbA[13] = lbg[13];
    lbA[14] = lbg[14];
    ubA[0] = ubg[0];
    ubA[1] = ubg[1];
    ubA[2] = ubg[2];
    ubA[3] = ubg[3];
    ubA[4] = ubg[4];
    ubA[5] = ubg[5];
    ubA[6] = ubg[6];
    ubA[7] = ubg[7];
    ubA[8] = ubg[8];
    ubA[9] = ubg[9];
    ubA[10] = ubg[10];
    ubA[11] = ubg[11];
    ubA[12] = ubg[12];
    ubA[13] = ubg[13];
    ubA[14] = ubg[14];


    for (uint i = 0; i < 15; i++) {
        ubA[i] -= prev_qpsolution->Z[34 + i];
        lbA[i] -= prev_qpsolution->Z[34 + i];
    }
}

void TtiQuadraticOptimizer::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        lb[lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        ub[ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

problem_solution TtiQuadraticOptimizer::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution TtiQuadraticOptimizer::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);

    int solver_status;
    int nwsr = 5 * (49);
    if (cpu_time > 0)
        _cpu_time = &cpu_time;
    else
        _cpu_time = nullptr;

    if (!solver.isInitialised() || init)
        solver_status = solver.init(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);
    else
        solver_status = solver.hotstart(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);

    real_t primal[34];
    solver.getPrimalSolution(primal);
    Eigen::VectorXd xopt = prev_qpsolution->Xopt + Eigen::Map<Eigen::VectorXd>(primal, 34);
    real_t _dual[49];
    solver.getDualSolution(_dual);
    Eigen::VectorXd lagrange_multiplier = -Eigen::Map<Eigen::VectorXd>(&(_dual[34]), 15);

    problem_solution ret = problem_solution(xopt, constraints(xopt, prob_params->param), lagrange_multiplier, solver.getObjVal(), qp_return_status(solver_status));
    // print_quadratic_problem_design(*prob_params, ret);
    // print_quadratic_problem(primal, _dual);
    // print_array("H", 34 * 34, H);

    return ret;
}
