#include "intercept_in_planes_quad_opti_osqp.h"
#include "intercept_in_planes_index.h"

void InterceptInPlanesQuadraticOptimizer::init() {
    settings = reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));

    data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

    Eigen::VectorXd xopt = Eigen::VectorXd(65).setZero();
    Eigen::VectorXd lamg = Eigen::VectorXd(47).setZero();

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

    _H = Eigen::MatrixXd(65, 65);
    _H.setZero();
    _H(0, 0) = (((lamg[0]*((0.5)*(xopt[26]+xopt[26])))+(lamg[1]*((0.5)*(xopt[27]+xopt[27]))))+(lamg[2]*((0.5)*(xopt[28]+xopt[28]))));
    _H(0, 5) = lamg[0];
    _H(0, 6) = lamg[1];
    _H(0, 7) = lamg[2];
    _H(0, 26) = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    _H(0, 27) = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    _H(0, 28) = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    _H(0, 47) = lamg[6];
    _H(0, 48) = lamg[7];
    _H(0, 49) = lamg[8];
    _H(1, 1) = (((lamg[21]*((0.5)*(xopt[29]+xopt[29])))+(lamg[22]*((0.5)*(xopt[30]+xopt[30]))))+(lamg[23]*((0.5)*(xopt[31]+xopt[31]))));
    _H(1, 17) = lamg[21];
    _H(1, 18) = lamg[22];
    _H(1, 19) = lamg[23];
    _H(1, 29) = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    _H(1, 30) = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    _H(1, 31) = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    _H(5, 0) = lamg[0];
    _H(6, 0) = lamg[1];
    _H(7, 0) = lamg[2];
    _H(17, 1) = lamg[21];
    _H(18, 1) = lamg[22];
    _H(19, 1) = lamg[23];
    _H(26, 0) = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    _H(27, 0) = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    _H(28, 0) = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    _H(29, 1) = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    _H(30, 1) = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    _H(31, 1) = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    _H(32, 32) = 3.33333e+06;
    _H(33, 33) = 3.33333e+06;
    _H(34, 34) = 3.33333e+06;
    _H(35, 35) = 3.33333e+06;
    _H(36, 36) = 3.33333e+06;
    _H(37, 37) = 3.33333e+06;
    _H(38, 38) = 3.33333e+06;
    _H(39, 39) = 3.33333e+06;
    _H(40, 40) = 3.33333e+06;
    _H(41, 41) = 3.33333e+06;
    _H(42, 42) = 3.33333e+06;
    _H(43, 43) = 3.33333e+06;
    _H(47, 0) = lamg[6];
    _H(48, 0) = lamg[7];
    _H(49, 0) = lamg[8];
    _H(56, 56) = 66.6667;
    _H(57, 57) = 66.6667;
    _H(58, 58) = 66.6667;
    _H(59, 59) = 3.33333e+06;
    _H(60, 60) = 3.33333e+06;
    _H(61, 61) = 3.33333e+06;
    _H(62, 62) = 3.33333e+06;
    _H(63, 63) = 3.33333e+06;
    _H(64, 64) = 3.33333e+06;


    _g = Eigen::VectorXd(65);
    _g.setZero();
    _g[0] = 1;
    _g[1] = 0.001;
    _g[32] = (1.66667e+06*(xopt[32]+xopt[32]));
    _g[33] = (1.66667e+06*(xopt[33]+xopt[33]));
    _g[34] = (1.66667e+06*(xopt[34]+xopt[34]));
    _g[35] = (1.66667e+06*(xopt[35]+xopt[35]));
    _g[36] = (1.66667e+06*(xopt[36]+xopt[36]));
    _g[37] = (1.66667e+06*(xopt[37]+xopt[37]));
    _g[38] = (1.66667e+06*(xopt[38]+xopt[38]));
    _g[39] = (1.66667e+06*(xopt[39]+xopt[39]));
    _g[40] = (1.66667e+06*(xopt[40]+xopt[40]));
    _g[41] = (1.66667e+06*(xopt[41]+xopt[41]));
    _g[42] = (1.66667e+06*(xopt[42]+xopt[42]));
    _g[43] = (1.66667e+06*(xopt[43]+xopt[43]));
    _g[56] = (33.3333*(xopt[56]+xopt[56]));
    _g[57] = (33.3333*(xopt[57]+xopt[57]));
    _g[58] = (33.3333*(xopt[58]+xopt[58]));
    _g[59] = (1.66667e+06*(xopt[59]+xopt[59]));
    _g[60] = (1.66667e+06*(xopt[60]+xopt[60]));
    _g[61] = (1.66667e+06*(xopt[61]+xopt[61]));
    _g[62] = (1.66667e+06*(xopt[62]+xopt[62]));
    _g[63] = (1.66667e+06*(xopt[63]+xopt[63]));
    _g[64] = (1.66667e+06*(xopt[64]+xopt[64]));

}


void InterceptInPlanesQuadraticOptimizer::qp_setup(QPSettings qpsettings) {
    qpsettings.apply(settings);
}

Eigen::VectorXd InterceptInPlanesQuadraticOptimizer::constraints(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    return constraints(xopt, param);
}

Eigen::VectorXd InterceptInPlanesQuadraticOptimizer::constraints(Eigen::VectorXd xopt, Eigen::VectorXd param [[maybe_unused]]) {
    Eigen::VectorXd constraints = Eigen::VectorXd(47).setZero();
    constraints[0] = (((xopt[2]+((xopt[0]*(xopt[5]+(xopt[5]+(xopt[0]*xopt[26]))))/2))-xopt[8])+xopt[32]);
    constraints[1] = (((xopt[3]+((xopt[0]*(xopt[6]+(xopt[6]+(xopt[0]*xopt[27]))))/2))-xopt[9])+xopt[33]);
    constraints[2] = (((xopt[4]+((xopt[0]*(xopt[7]+(xopt[7]+(xopt[0]*xopt[28]))))/2))-xopt[10])+xopt[34]);
    constraints[3] = (((xopt[5]+((xopt[0]*(xopt[26]+xopt[26]))/2))-xopt[11])+xopt[35]);
    constraints[4] = (((xopt[6]+((xopt[0]*(xopt[27]+xopt[27]))/2))-xopt[12])+xopt[36]);
    constraints[5] = (((xopt[7]+((xopt[0]*(xopt[28]+xopt[28]))/2))-xopt[13])+xopt[37]);
    constraints[6] = ((xopt[44]+((xopt[0]*(xopt[47]+xopt[47]))/2))-xopt[50]);
    constraints[7] = ((xopt[45]+((xopt[0]*(xopt[48]+xopt[48]))/2))-xopt[51]);
    constraints[8] = ((xopt[46]+((xopt[0]*(xopt[49]+xopt[49]))/2))-xopt[52]);
    constraints[9] = (xopt[47]-xopt[53]);
    constraints[10] = (xopt[48]-xopt[54]);
    constraints[11] = (xopt[49]-xopt[55]);
    constraints[12] = ((xopt[8]-xopt[50])+xopt[56]);
    constraints[13] = ((xopt[9]-xopt[51])+xopt[57]);
    constraints[14] = ((xopt[10]-xopt[52])+xopt[58]);
    constraints[15] = ((xopt[8]-xopt[14])+xopt[59]);
    constraints[16] = ((xopt[9]-xopt[15])+xopt[60]);
    constraints[17] = ((xopt[10]-xopt[16])+xopt[61]);
    constraints[18] = ((xopt[11]-xopt[17])+xopt[62]);
    constraints[19] = ((xopt[12]-xopt[18])+xopt[63]);
    constraints[20] = ((xopt[13]-xopt[19])+xopt[64]);
    constraints[21] = (((xopt[14]+((xopt[1]*(xopt[17]+(xopt[17]+(xopt[1]*xopt[29]))))/2))-xopt[20])+xopt[38]);
    constraints[22] = (((xopt[15]+((xopt[1]*(xopt[18]+(xopt[18]+(xopt[1]*xopt[30]))))/2))-xopt[21])+xopt[39]);
    constraints[23] = (((xopt[16]+((xopt[1]*(xopt[19]+(xopt[19]+(xopt[1]*xopt[31]))))/2))-xopt[22])+xopt[40]);
    constraints[24] = (((xopt[17]+((xopt[1]*(xopt[29]+xopt[29]))/2))-xopt[23])+xopt[41]);
    constraints[25] = (((xopt[18]+((xopt[1]*(xopt[30]+xopt[30]))/2))-xopt[24])+xopt[42]);
    constraints[26] = (((xopt[19]+((xopt[1]*(xopt[31]+xopt[31]))/2))-xopt[25])+xopt[43]);
    constraints[27] = (((param[3]*(xopt[8]-param[0]))+(param[4]*(xopt[9]-param[1])))+(param[5]*(xopt[10]-param[2])));
    constraints[28] = (((param[9]*(xopt[8]-param[6]))+(param[10]*(xopt[9]-param[7])))+(param[11]*(xopt[10]-param[8])));
    constraints[29] = (((param[15]*(xopt[8]-param[12]))+(param[16]*(xopt[9]-param[13])))+(param[17]*(xopt[10]-param[14])));
    constraints[30] = (((param[21]*(xopt[8]-param[18]))+(param[22]*(xopt[9]-param[19])))+(param[23]*(xopt[10]-param[20])));
    constraints[31] = (((param[27]*(xopt[8]-param[24]))+(param[28]*(xopt[9]-param[25])))+(param[29]*(xopt[10]-param[26])));
    constraints[32] = (((param[33]*(xopt[8]-param[30]))+(param[34]*(xopt[9]-param[31])))+(param[35]*(xopt[10]-param[32])));
    constraints[33] = (((param[39]*(xopt[8]-param[36]))+(param[40]*(xopt[9]-param[37])))+(param[41]*(xopt[10]-param[38])));
    constraints[34] = (((param[45]*(xopt[8]-param[42]))+(param[46]*(xopt[9]-param[43])))+(param[47]*(xopt[10]-param[44])));
    constraints[35] = (((param[51]*(xopt[8]-param[48]))+(param[52]*(xopt[9]-param[49])))+(param[53]*(xopt[10]-param[50])));
    constraints[36] = (((param[57]*(xopt[8]-param[54]))+(param[58]*(xopt[9]-param[55])))+(param[59]*(xopt[10]-param[56])));
    constraints[37] = (((param[3]*(xopt[20]-param[0]))+(param[4]*(xopt[21]-param[1])))+(param[5]*(xopt[22]-param[2])));
    constraints[38] = (((param[9]*(xopt[20]-param[6]))+(param[10]*(xopt[21]-param[7])))+(param[11]*(xopt[22]-param[8])));
    constraints[39] = (((param[15]*(xopt[20]-param[12]))+(param[16]*(xopt[21]-param[13])))+(param[17]*(xopt[22]-param[14])));
    constraints[40] = (((param[21]*(xopt[20]-param[18]))+(param[22]*(xopt[21]-param[19])))+(param[23]*(xopt[22]-param[20])));
    constraints[41] = (((param[27]*(xopt[20]-param[24]))+(param[28]*(xopt[21]-param[25])))+(param[29]*(xopt[22]-param[26])));
    constraints[42] = (((param[33]*(xopt[20]-param[30]))+(param[34]*(xopt[21]-param[31])))+(param[35]*(xopt[22]-param[32])));
    constraints[43] = (((param[39]*(xopt[20]-param[36]))+(param[40]*(xopt[21]-param[37])))+(param[41]*(xopt[22]-param[38])));
    constraints[44] = (((param[45]*(xopt[20]-param[42]))+(param[46]*(xopt[21]-param[43])))+(param[47]*(xopt[22]-param[44])));
    constraints[45] = (((param[51]*(xopt[20]-param[48]))+(param[52]*(xopt[21]-param[49])))+(param[53]*(xopt[22]-param[50])));
    constraints[46] = (((param[57]*(xopt[20]-param[54]))+(param[58]*(xopt[21]-param[55])))+(param[59]*(xopt[22]-param[56])));

    return constraints;
}

Eigen::MatrixXd InterceptInPlanesQuadraticOptimizer::constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::MatrixXd dconstraints = Eigen::MatrixXd(47, 65).setZero();
    dconstraints(0, 0) = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[26])))+(xopt[0]*xopt[26])));
    dconstraints(0, 2) = 1;
    dconstraints(0, 5) = xopt[0];
    dconstraints(0, 8) = -1;
    dconstraints(0, 26) = (0.5*sq(xopt[0]));
    dconstraints(0, 32) = 1;
    dconstraints(1, 0) = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[27])))+(xopt[0]*xopt[27])));
    dconstraints(1, 3) = 1;
    dconstraints(1, 6) = xopt[0];
    dconstraints(1, 9) = -1;
    dconstraints(1, 27) = (0.5*sq(xopt[0]));
    dconstraints(1, 33) = 1;
    dconstraints(2, 0) = (0.5*((xopt[7]+(xopt[7]+(xopt[0]*xopt[28])))+(xopt[0]*xopt[28])));
    dconstraints(2, 4) = 1;
    dconstraints(2, 7) = xopt[0];
    dconstraints(2, 10) = -1;
    dconstraints(2, 28) = (0.5*sq(xopt[0]));
    dconstraints(2, 34) = 1;
    dconstraints(3, 0) = (0.5*(xopt[26]+xopt[26]));
    dconstraints(3, 5) = 1;
    dconstraints(3, 11) = -1;
    dconstraints(3, 26) = xopt[0];
    dconstraints(3, 35) = 1;
    dconstraints(4, 0) = (0.5*(xopt[27]+xopt[27]));
    dconstraints(4, 6) = 1;
    dconstraints(4, 12) = -1;
    dconstraints(4, 27) = xopt[0];
    dconstraints(4, 36) = 1;
    dconstraints(5, 0) = (0.5*(xopt[28]+xopt[28]));
    dconstraints(5, 7) = 1;
    dconstraints(5, 13) = -1;
    dconstraints(5, 28) = xopt[0];
    dconstraints(5, 37) = 1;
    dconstraints(6, 0) = (0.5*(xopt[47]+xopt[47]));
    dconstraints(6, 44) = 1;
    dconstraints(6, 47) = xopt[0];
    dconstraints(6, 50) = -1;
    dconstraints(7, 0) = (0.5*(xopt[48]+xopt[48]));
    dconstraints(7, 45) = 1;
    dconstraints(7, 48) = xopt[0];
    dconstraints(7, 51) = -1;
    dconstraints(8, 0) = (0.5*(xopt[49]+xopt[49]));
    dconstraints(8, 46) = 1;
    dconstraints(8, 49) = xopt[0];
    dconstraints(8, 52) = -1;
    dconstraints(9, 47) = 1;
    dconstraints(9, 53) = -1;
    dconstraints(10, 48) = 1;
    dconstraints(10, 54) = -1;
    dconstraints(11, 49) = 1;
    dconstraints(11, 55) = -1;
    dconstraints(12, 8) = 1;
    dconstraints(12, 50) = -1;
    dconstraints(12, 56) = 1;
    dconstraints(13, 9) = 1;
    dconstraints(13, 51) = -1;
    dconstraints(13, 57) = 1;
    dconstraints(14, 10) = 1;
    dconstraints(14, 52) = -1;
    dconstraints(14, 58) = 1;
    dconstraints(15, 8) = 1;
    dconstraints(15, 14) = -1;
    dconstraints(15, 59) = 1;
    dconstraints(16, 9) = 1;
    dconstraints(16, 15) = -1;
    dconstraints(16, 60) = 1;
    dconstraints(17, 10) = 1;
    dconstraints(17, 16) = -1;
    dconstraints(17, 61) = 1;
    dconstraints(18, 11) = 1;
    dconstraints(18, 17) = -1;
    dconstraints(18, 62) = 1;
    dconstraints(19, 12) = 1;
    dconstraints(19, 18) = -1;
    dconstraints(19, 63) = 1;
    dconstraints(20, 13) = 1;
    dconstraints(20, 19) = -1;
    dconstraints(20, 64) = 1;
    dconstraints(21, 1) = (0.5*((xopt[17]+(xopt[17]+(xopt[1]*xopt[29])))+(xopt[1]*xopt[29])));
    dconstraints(21, 14) = 1;
    dconstraints(21, 17) = xopt[1];
    dconstraints(21, 20) = -1;
    dconstraints(21, 29) = (0.5*sq(xopt[1]));
    dconstraints(21, 38) = 1;
    dconstraints(22, 1) = (0.5*((xopt[18]+(xopt[18]+(xopt[1]*xopt[30])))+(xopt[1]*xopt[30])));
    dconstraints(22, 15) = 1;
    dconstraints(22, 18) = xopt[1];
    dconstraints(22, 21) = -1;
    dconstraints(22, 30) = (0.5*sq(xopt[1]));
    dconstraints(22, 39) = 1;
    dconstraints(23, 1) = (0.5*((xopt[19]+(xopt[19]+(xopt[1]*xopt[31])))+(xopt[1]*xopt[31])));
    dconstraints(23, 16) = 1;
    dconstraints(23, 19) = xopt[1];
    dconstraints(23, 22) = -1;
    dconstraints(23, 31) = (0.5*sq(xopt[1]));
    dconstraints(23, 40) = 1;
    dconstraints(24, 1) = (0.5*(xopt[29]+xopt[29]));
    dconstraints(24, 17) = 1;
    dconstraints(24, 23) = -1;
    dconstraints(24, 29) = xopt[1];
    dconstraints(24, 41) = 1;
    dconstraints(25, 1) = (0.5*(xopt[30]+xopt[30]));
    dconstraints(25, 18) = 1;
    dconstraints(25, 24) = -1;
    dconstraints(25, 30) = xopt[1];
    dconstraints(25, 42) = 1;
    dconstraints(26, 1) = (0.5*(xopt[31]+xopt[31]));
    dconstraints(26, 19) = 1;
    dconstraints(26, 25) = -1;
    dconstraints(26, 31) = xopt[1];
    dconstraints(26, 43) = 1;
    dconstraints(27, 8) = param[3];
    dconstraints(27, 9) = param[4];
    dconstraints(27, 10) = param[5];
    dconstraints(28, 8) = param[9];
    dconstraints(28, 9) = param[10];
    dconstraints(28, 10) = param[11];
    dconstraints(29, 8) = param[15];
    dconstraints(29, 9) = param[16];
    dconstraints(29, 10) = param[17];
    dconstraints(30, 8) = param[21];
    dconstraints(30, 9) = param[22];
    dconstraints(30, 10) = param[23];
    dconstraints(31, 8) = param[27];
    dconstraints(31, 9) = param[28];
    dconstraints(31, 10) = param[29];
    dconstraints(32, 8) = param[33];
    dconstraints(32, 9) = param[34];
    dconstraints(32, 10) = param[35];
    dconstraints(33, 8) = param[39];
    dconstraints(33, 9) = param[40];
    dconstraints(33, 10) = param[41];
    dconstraints(34, 8) = param[45];
    dconstraints(34, 9) = param[46];
    dconstraints(34, 10) = param[47];
    dconstraints(35, 8) = param[51];
    dconstraints(35, 9) = param[52];
    dconstraints(35, 10) = param[53];
    dconstraints(36, 8) = param[57];
    dconstraints(36, 9) = param[58];
    dconstraints(36, 10) = param[59];
    dconstraints(37, 20) = param[3];
    dconstraints(37, 21) = param[4];
    dconstraints(37, 22) = param[5];
    dconstraints(38, 20) = param[9];
    dconstraints(38, 21) = param[10];
    dconstraints(38, 22) = param[11];
    dconstraints(39, 20) = param[15];
    dconstraints(39, 21) = param[16];
    dconstraints(39, 22) = param[17];
    dconstraints(40, 20) = param[21];
    dconstraints(40, 21) = param[22];
    dconstraints(40, 22) = param[23];
    dconstraints(41, 20) = param[27];
    dconstraints(41, 21) = param[28];
    dconstraints(41, 22) = param[29];
    dconstraints(42, 20) = param[33];
    dconstraints(42, 21) = param[34];
    dconstraints(42, 22) = param[35];
    dconstraints(43, 20) = param[39];
    dconstraints(43, 21) = param[40];
    dconstraints(43, 22) = param[41];
    dconstraints(44, 20) = param[45];
    dconstraints(44, 21) = param[46];
    dconstraints(44, 22) = param[47];
    dconstraints(45, 20) = param[51];
    dconstraints(45, 21) = param[52];
    dconstraints(45, 22) = param[53];
    dconstraints(46, 20) = param[57];
    dconstraints(46, 21) = param[58];
    dconstraints(46, 22) = param[59];

    return dconstraints;
}

void InterceptInPlanesQuadraticOptimizer::update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::VectorXd lamg = prev_qpsolution->lagrange_multiplier;

    P_x[0] = (((lamg[0]*((0.5)*(xopt[26]+xopt[26])))+(lamg[1]*((0.5)*(xopt[27]+xopt[27]))))+(lamg[2]*((0.5)*(xopt[28]+xopt[28]))));
    P_x[1] = (((lamg[21]*((0.5)*(xopt[29]+xopt[29])))+(lamg[22]*((0.5)*(xopt[30]+xopt[30]))))+(lamg[23]*((0.5)*(xopt[31]+xopt[31]))));
    P_x[2] = lamg[0];
    P_x[3] = lamg[1];
    P_x[4] = lamg[2];
    P_x[5] = lamg[21];
    P_x[6] = lamg[22];
    P_x[7] = lamg[23];
    P_x[8] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    P_x[9] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    P_x[10] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    P_x[11] = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    P_x[12] = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    P_x[13] = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    P_x[14] = 3.33333e+06;
    P_x[15] = 3.33333e+06;
    P_x[16] = 3.33333e+06;
    P_x[17] = 3.33333e+06;
    P_x[18] = 3.33333e+06;
    P_x[19] = 3.33333e+06;
    P_x[20] = 3.33333e+06;
    P_x[21] = 3.33333e+06;
    P_x[22] = 3.33333e+06;
    P_x[23] = 3.33333e+06;
    P_x[24] = 3.33333e+06;
    P_x[25] = 3.33333e+06;
    P_x[26] = lamg[6];
    P_x[27] = lamg[7];
    P_x[28] = lamg[8];
    P_x[29] = 66.6667;
    P_x[30] = 66.6667;
    P_x[31] = 66.6667;
    P_x[32] = 3.33333e+06;
    P_x[33] = 3.33333e+06;
    P_x[34] = 3.33333e+06;
    P_x[35] = 3.33333e+06;
    P_x[36] = 3.33333e+06;
    P_x[37] = 3.33333e+06;


    float regularisation_value = sqrt(norm_1(P_p, P_x, 65) * norm_inf(P_p, P_i, P_x, 65));
    add_diag(P_p, P_i, P_x, 65, regularisation_value);
}

void InterceptInPlanesQuadraticOptimizer::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    q[0] = 1;
    q[1] = 0.001;
    q[32] = (1.66667e+06*(xopt[32]+xopt[32]));
    q[33] = (1.66667e+06*(xopt[33]+xopt[33]));
    q[34] = (1.66667e+06*(xopt[34]+xopt[34]));
    q[35] = (1.66667e+06*(xopt[35]+xopt[35]));
    q[36] = (1.66667e+06*(xopt[36]+xopt[36]));
    q[37] = (1.66667e+06*(xopt[37]+xopt[37]));
    q[38] = (1.66667e+06*(xopt[38]+xopt[38]));
    q[39] = (1.66667e+06*(xopt[39]+xopt[39]));
    q[40] = (1.66667e+06*(xopt[40]+xopt[40]));
    q[41] = (1.66667e+06*(xopt[41]+xopt[41]));
    q[42] = (1.66667e+06*(xopt[42]+xopt[42]));
    q[43] = (1.66667e+06*(xopt[43]+xopt[43]));
    q[56] = (33.3333*(xopt[56]+xopt[56]));
    q[57] = (33.3333*(xopt[57]+xopt[57]));
    q[58] = (33.3333*(xopt[58]+xopt[58]));
    q[59] = (1.66667e+06*(xopt[59]+xopt[59]));
    q[60] = (1.66667e+06*(xopt[60]+xopt[60]));
    q[61] = (1.66667e+06*(xopt[61]+xopt[61]));
    q[62] = (1.66667e+06*(xopt[62]+xopt[62]));
    q[63] = (1.66667e+06*(xopt[63]+xopt[63]));
    q[64] = (1.66667e+06*(xopt[64]+xopt[64]));

}

void InterceptInPlanesQuadraticOptimizer::update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    A_x[0] = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[26])))+(xopt[0]*xopt[26])));
    A_x[1] = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[27])))+(xopt[0]*xopt[27])));
    A_x[2] = (0.5*((xopt[7]+(xopt[7]+(xopt[0]*xopt[28])))+(xopt[0]*xopt[28])));
    A_x[3] = (0.5*(xopt[26]+xopt[26]));
    A_x[4] = (0.5*(xopt[27]+xopt[27]));
    A_x[5] = (0.5*(xopt[28]+xopt[28]));
    A_x[6] = (0.5*(xopt[47]+xopt[47]));
    A_x[7] = (0.5*(xopt[48]+xopt[48]));
    A_x[8] = (0.5*(xopt[49]+xopt[49]));
    A_x[9] = 1;
    A_x[10] = (0.5*((xopt[17]+(xopt[17]+(xopt[1]*xopt[29])))+(xopt[1]*xopt[29])));
    A_x[11] = (0.5*((xopt[18]+(xopt[18]+(xopt[1]*xopt[30])))+(xopt[1]*xopt[30])));
    A_x[12] = (0.5*((xopt[19]+(xopt[19]+(xopt[1]*xopt[31])))+(xopt[1]*xopt[31])));
    A_x[13] = (0.5*(xopt[29]+xopt[29]));
    A_x[14] = (0.5*(xopt[30]+xopt[30]));
    A_x[15] = (0.5*(xopt[31]+xopt[31]));
    A_x[16] = 1;
    A_x[17] = 1;
    A_x[18] = 1;
    A_x[19] = 1;
    A_x[20] = 1;
    A_x[21] = 1;
    A_x[22] = 1;
    A_x[23] = xopt[0];
    A_x[24] = 1;
    A_x[25] = 1;
    A_x[26] = xopt[0];
    A_x[27] = 1;
    A_x[28] = 1;
    A_x[29] = xopt[0];
    A_x[30] = 1;
    A_x[31] = 1;
    A_x[32] = -1;
    A_x[33] = 1;
    A_x[34] = 1;
    A_x[35] = param[3];
    A_x[36] = param[9];
    A_x[37] = param[15];
    A_x[38] = param[21];
    A_x[39] = param[27];
    A_x[40] = param[33];
    A_x[41] = param[39];
    A_x[42] = param[45];
    A_x[43] = param[51];
    A_x[44] = param[57];
    A_x[45] = 1;
    A_x[46] = -1;
    A_x[47] = 1;
    A_x[48] = 1;
    A_x[49] = param[4];
    A_x[50] = param[10];
    A_x[51] = param[16];
    A_x[52] = param[22];
    A_x[53] = param[28];
    A_x[54] = param[34];
    A_x[55] = param[40];
    A_x[56] = param[46];
    A_x[57] = param[52];
    A_x[58] = param[58];
    A_x[59] = 1;
    A_x[60] = -1;
    A_x[61] = 1;
    A_x[62] = 1;
    A_x[63] = param[5];
    A_x[64] = param[11];
    A_x[65] = param[17];
    A_x[66] = param[23];
    A_x[67] = param[29];
    A_x[68] = param[35];
    A_x[69] = param[41];
    A_x[70] = param[47];
    A_x[71] = param[53];
    A_x[72] = param[59];
    A_x[73] = 1;
    A_x[74] = -1;
    A_x[75] = 1;
    A_x[76] = 1;
    A_x[77] = -1;
    A_x[78] = 1;
    A_x[79] = 1;
    A_x[80] = -1;
    A_x[81] = 1;
    A_x[82] = 1;
    A_x[83] = -1;
    A_x[84] = 1;
    A_x[85] = 1;
    A_x[86] = -1;
    A_x[87] = 1;
    A_x[88] = 1;
    A_x[89] = -1;
    A_x[90] = 1;
    A_x[91] = 1;
    A_x[92] = -1;
    A_x[93] = xopt[1];
    A_x[94] = 1;
    A_x[95] = 1;
    A_x[96] = -1;
    A_x[97] = xopt[1];
    A_x[98] = 1;
    A_x[99] = 1;
    A_x[100] = -1;
    A_x[101] = xopt[1];
    A_x[102] = 1;
    A_x[103] = 1;
    A_x[104] = -1;
    A_x[105] = param[3];
    A_x[106] = param[9];
    A_x[107] = param[15];
    A_x[108] = param[21];
    A_x[109] = param[27];
    A_x[110] = param[33];
    A_x[111] = param[39];
    A_x[112] = param[45];
    A_x[113] = param[51];
    A_x[114] = param[57];
    A_x[115] = 1;
    A_x[116] = -1;
    A_x[117] = param[4];
    A_x[118] = param[10];
    A_x[119] = param[16];
    A_x[120] = param[22];
    A_x[121] = param[28];
    A_x[122] = param[34];
    A_x[123] = param[40];
    A_x[124] = param[46];
    A_x[125] = param[52];
    A_x[126] = param[58];
    A_x[127] = 1;
    A_x[128] = -1;
    A_x[129] = param[5];
    A_x[130] = param[11];
    A_x[131] = param[17];
    A_x[132] = param[23];
    A_x[133] = param[29];
    A_x[134] = param[35];
    A_x[135] = param[41];
    A_x[136] = param[47];
    A_x[137] = param[53];
    A_x[138] = param[59];
    A_x[139] = 1;
    A_x[140] = -1;
    A_x[141] = 1;
    A_x[142] = -1;
    A_x[143] = 1;
    A_x[144] = -1;
    A_x[145] = 1;
    A_x[146] = (0.5*sq(xopt[0]));
    A_x[147] = xopt[0];
    A_x[148] = 1;
    A_x[149] = (0.5*sq(xopt[0]));
    A_x[150] = xopt[0];
    A_x[151] = 1;
    A_x[152] = (0.5*sq(xopt[0]));
    A_x[153] = xopt[0];
    A_x[154] = 1;
    A_x[155] = (0.5*sq(xopt[1]));
    A_x[156] = xopt[1];
    A_x[157] = 1;
    A_x[158] = (0.5*sq(xopt[1]));
    A_x[159] = xopt[1];
    A_x[160] = 1;
    A_x[161] = (0.5*sq(xopt[1]));
    A_x[162] = xopt[1];
    A_x[163] = 1;
    A_x[164] = 1;
    A_x[165] = 1;
    A_x[166] = 1;
    A_x[167] = 1;
    A_x[168] = 1;
    A_x[169] = 1;
    A_x[170] = 1;
    A_x[171] = 1;
    A_x[172] = 1;
    A_x[173] = 1;
    A_x[174] = 1;
    A_x[175] = 1;
    A_x[176] = 1;
    A_x[177] = 1;
    A_x[178] = 1;
    A_x[179] = 1;
    A_x[180] = 1;
    A_x[181] = 1;
    A_x[182] = 1;
    A_x[183] = 1;
    A_x[184] = 1;
    A_x[185] = 1;
    A_x[186] = 1;
    A_x[187] = 1;
    A_x[188] = 1;
    A_x[189] = 1;
    A_x[190] = 1;
    A_x[191] = 1;
    A_x[192] = 1;
    A_x[193] = 1;
    A_x[194] = xopt[0];
    A_x[195] = 1;
    A_x[196] = 1;
    A_x[197] = xopt[0];
    A_x[198] = 1;
    A_x[199] = 1;
    A_x[200] = xopt[0];
    A_x[201] = 1;
    A_x[202] = 1;
    A_x[203] = -1;
    A_x[204] = -1;
    A_x[205] = 1;
    A_x[206] = -1;
    A_x[207] = -1;
    A_x[208] = 1;
    A_x[209] = -1;
    A_x[210] = -1;
    A_x[211] = 1;
    A_x[212] = -1;
    A_x[213] = 1;
    A_x[214] = -1;
    A_x[215] = 1;
    A_x[216] = -1;
    A_x[217] = 1;
    A_x[218] = 1;
    A_x[219] = 1;
    A_x[220] = 1;
    A_x[221] = 1;
    A_x[222] = 1;
    A_x[223] = 1;
    A_x[224] = 1;
    A_x[225] = 1;
    A_x[226] = 1;
    A_x[227] = 1;
    A_x[228] = 1;
    A_x[229] = 1;
    A_x[230] = 1;
    A_x[231] = 1;
    A_x[232] = 1;
    A_x[233] = 1;
    A_x[234] = 1;
    A_x[235] = 1;

}

void InterceptInPlanesQuadraticOptimizer::update_vectors_bA(problem_parameters *prob_params[[maybe_unused]], problem_solution *prev_qpsolution) {
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
    l[15] = lbg[15];
    l[16] = lbg[16];
    l[17] = lbg[17];
    l[18] = lbg[18];
    l[19] = lbg[19];
    l[20] = lbg[20];
    l[21] = lbg[21];
    l[22] = lbg[22];
    l[23] = lbg[23];
    l[24] = lbg[24];
    l[25] = lbg[25];
    l[26] = lbg[26];
    l[27] = lbg[27];
    l[28] = lbg[28];
    l[29] = lbg[29];
    l[30] = lbg[30];
    l[31] = lbg[31];
    l[32] = lbg[32];
    l[33] = lbg[33];
    l[34] = lbg[34];
    l[35] = lbg[35];
    l[36] = lbg[36];
    l[37] = lbg[37];
    l[38] = lbg[38];
    l[39] = lbg[39];
    l[40] = lbg[40];
    l[41] = lbg[41];
    l[42] = lbg[42];
    l[43] = lbg[43];
    l[44] = lbg[44];
    l[45] = lbg[45];
    l[46] = lbg[46];
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
    u[15] = ubg[15];
    u[16] = ubg[16];
    u[17] = ubg[17];
    u[18] = ubg[18];
    u[19] = ubg[19];
    u[20] = ubg[20];
    u[21] = ubg[21];
    u[22] = ubg[22];
    u[23] = ubg[23];
    u[24] = ubg[24];
    u[25] = ubg[25];
    u[26] = ubg[26];
    u[27] = ubg[27];
    u[28] = ubg[28];
    u[29] = ubg[29];
    u[30] = ubg[30];
    u[31] = ubg[31];
    u[32] = ubg[32];
    u[33] = ubg[33];
    u[34] = ubg[34];
    u[35] = ubg[35];
    u[36] = ubg[36];
    u[37] = ubg[37];
    u[38] = ubg[38];
    u[39] = ubg[39];
    u[40] = ubg[40];
    u[41] = ubg[41];
    u[42] = ubg[42];
    u[43] = ubg[43];
    u[44] = ubg[44];
    u[45] = ubg[45];
    u[46] = ubg[46];


    for (uint i = 0; i < 47; i++) {
        u[i] -= prev_qpsolution->Z[65 + i];
        l[i] -= prev_qpsolution->Z[65 + i];
    }
}

void InterceptInPlanesQuadraticOptimizer::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        l[47 + lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        u[47 + ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

void InterceptInPlanesQuadraticOptimizer::update_init_guess(problem_solution *prev_qpsolution) {

    for (uint i = 0; i < 65; i++) {
        x0[i] = prev_qpsolution->Xopt[i];
    }
    for (uint i = 0; i < 47; i++) {
        lam0[i] = prev_qpsolution->lagrange_multiplier[i];
    }
}

problem_solution InterceptInPlanesQuadraticOptimizer::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution InterceptInPlanesQuadraticOptimizer::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init [[maybe_unused]], double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);
    update_init_guess(prev_qpsolution);

    if (!setup) {
        if (data) {
            data->n = 65;
            data->m = 112;
            data->P = csc_matrix(data->n, data->n, 38, P_x, P_i, P_p);
            data->q = q;
            data->A = csc_matrix(data->m, data->n, 236, A_x, A_i, A_p);
            data->l = l;
            data->u = u;
        }
        c_int exitflag = osqp_setup(&work, data, settings);
        if (!exitflag)
            setup = true;

    } else {
        osqp_update_P(work, P_x, OSQP_NULL, 38);
        osqp_update_lin_cost(work, q);
        osqp_update_A(work, A_x, OSQP_NULL, 236);
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
    // print_array("H", 65 * 65, H);

    return ret;
}


std::vector<Eigen::VectorXd> InterceptInPlanesQuadraticOptimizer::trajectory(Eigen::VectorXd xopt, trajectory_type traj_type) {

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
