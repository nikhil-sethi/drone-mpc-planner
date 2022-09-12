#include "solvertemplate.h"

void SolverTemplate::init() {
    solver = SQProblem(N_XOPTS, N_CONSTRAINTS);

    Options options;
    options.setToMPC();
    options.printLevel = PL_NONE;
    options.enableRegularisation = BT_TRUE;
    Eigen::VectorXd xopt = Eigen::VectorXd(N_XOPTS).setZero();

    solver.setOptions(options);
    memset(H, 0, sizeof(H));
    Eigen::VectorXd lamg = Eigen::VectorXd(N_CONSTRAINTS).setZero();
    /* INIT H PLACEHOLDER*/

    memset(g, 0, sizeof(g));
    /* INIT g PLACEHOLDER*/

    _H = Eigen::Map<Eigen::Matrix<double, N_XOPTS, N_XOPTS, Eigen::RowMajor>>(H);
    _g = Eigen::Map<Eigen::VectorXd>(g, N_XOPTS);
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

    /* UPDATE H PLACEHOLDER*/
}

void SolverTemplate::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    /* INIT g PLACEHOLDER*/
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
        ubA[i] -= prev_qpsolution->Z[N_XOPTS + i];
        lbA[i] -= prev_qpsolution->Z[N_XOPTS + i];
    }
}

void SolverTemplate::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        lb[lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        ub[ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

problem_solution SolverTemplate::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution SolverTemplate::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);

    int solver_status;
    int nwsr = _nWSR;
    if (cpu_time > 0)
        _cpu_time = &cpu_time;
    else
        _cpu_time = nullptr;

    if (!solver.isInitialised() || init)
        solver_status = solver.init(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);
    else
        solver_status = solver.hotstart(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);

    real_t primal[N_XOPTS];
    solver.getPrimalSolution(primal);
    Eigen::VectorXd xopt = prev_qpsolution->Xopt + Eigen::Map<Eigen::VectorXd>(primal, N_XOPTS);
    real_t _dual[N_XOPTS + N_CONSTRAINTS];
    solver.getDualSolution(_dual);
    Eigen::VectorXd lagrange_multiplier = -Eigen::Map<Eigen::VectorXd>(&(_dual[N_XOPTS]), N_CONSTRAINTS);

    problem_solution ret = problem_solution(xopt, constraints(xopt, prob_params->param), lagrange_multiplier, solver.getObjVal(), qp_return_status(solver_status));
    // print_quadratic_problem_design(*prob_params, ret);
    // print_quadratic_problem(primal, _dual);
    // print_array("H", N_XOPTS * N_XOPTS, H);

    return ret;
}
