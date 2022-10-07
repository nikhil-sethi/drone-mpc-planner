#include "intercept_in_planes_quad_opti_qpoases.h"
#include "intercept_in_planes_index.h"

void InterceptInPlanesQuadraticOptimizer::init() {
    solver = SQProblem(65, 47);

    Options options;
    options.setToDefault();
    options.printLevel = PL_NONE;

    // options.enableRegularisation = BT_TRUE;
    Eigen::VectorXd xopt = Eigen::VectorXd(65).setZero();

    solver.setOptions(options);
    memset(H, 0, sizeof(H));
    Eigen::VectorXd lamg = Eigen::VectorXd(47).setZero();
    H[0] = (((lamg[0]*((0.5)*(xopt[26]+xopt[26])))+(lamg[1]*((0.5)*(xopt[27]+xopt[27]))))+(lamg[2]*((0.5)*(xopt[28]+xopt[28]))));
    H[5] = lamg[0];
    H[6] = lamg[1];
    H[7] = lamg[2];
    H[26] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[27] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[28] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[47] = lamg[6];
    H[48] = lamg[7];
    H[49] = lamg[8];
    H[66] = (((lamg[21]*((0.5)*(xopt[29]+xopt[29])))+(lamg[22]*((0.5)*(xopt[30]+xopt[30]))))+(lamg[23]*((0.5)*(xopt[31]+xopt[31]))));
    H[82] = lamg[21];
    H[83] = lamg[22];
    H[84] = lamg[23];
    H[94] = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    H[95] = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    H[96] = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    H[325] = lamg[0];
    H[390] = lamg[1];
    H[455] = lamg[2];
    H[1106] = lamg[21];
    H[1171] = lamg[22];
    H[1236] = lamg[23];
    H[1690] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[1755] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[1820] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[1886] = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    H[1951] = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    H[2016] = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    H[2112] = 3.33333e+06;
    H[2178] = 3.33333e+06;
    H[2244] = 3.33333e+06;
    H[2310] = 3.33333e+06;
    H[2376] = 3.33333e+06;
    H[2442] = 3.33333e+06;
    H[2508] = 3.33333e+06;
    H[2574] = 3.33333e+06;
    H[2640] = 3.33333e+06;
    H[2706] = 3.33333e+06;
    H[2772] = 3.33333e+06;
    H[2838] = 3.33333e+06;
    H[3055] = lamg[6];
    H[3120] = lamg[7];
    H[3185] = lamg[8];
    H[3696] = 66.6667;
    H[3762] = 66.6667;
    H[3828] = 66.6667;
    H[3894] = 3.33333e+06;
    H[3960] = 3.33333e+06;
    H[4026] = 3.33333e+06;
    H[4092] = 3.33333e+06;
    H[4158] = 3.33333e+06;
    H[4224] = 3.33333e+06;


    memset(g, 0, sizeof(g));
    g[0] = 1;
    g[1] = 0.001;
    g[32] = (1.66667e+06*(xopt[32]+xopt[32]));
    g[33] = (1.66667e+06*(xopt[33]+xopt[33]));
    g[34] = (1.66667e+06*(xopt[34]+xopt[34]));
    g[35] = (1.66667e+06*(xopt[35]+xopt[35]));
    g[36] = (1.66667e+06*(xopt[36]+xopt[36]));
    g[37] = (1.66667e+06*(xopt[37]+xopt[37]));
    g[38] = (1.66667e+06*(xopt[38]+xopt[38]));
    g[39] = (1.66667e+06*(xopt[39]+xopt[39]));
    g[40] = (1.66667e+06*(xopt[40]+xopt[40]));
    g[41] = (1.66667e+06*(xopt[41]+xopt[41]));
    g[42] = (1.66667e+06*(xopt[42]+xopt[42]));
    g[43] = (1.66667e+06*(xopt[43]+xopt[43]));
    g[56] = (33.3333*(xopt[56]+xopt[56]));
    g[57] = (33.3333*(xopt[57]+xopt[57]));
    g[58] = (33.3333*(xopt[58]+xopt[58]));
    g[59] = (1.66667e+06*(xopt[59]+xopt[59]));
    g[60] = (1.66667e+06*(xopt[60]+xopt[60]));
    g[61] = (1.66667e+06*(xopt[61]+xopt[61]));
    g[62] = (1.66667e+06*(xopt[62]+xopt[62]));
    g[63] = (1.66667e+06*(xopt[63]+xopt[63]));
    g[64] = (1.66667e+06*(xopt[64]+xopt[64]));


    _H = Eigen::Map<Eigen::Matrix<double, 65, 65, Eigen::RowMajor>>(H);
    _g = Eigen::Map<Eigen::VectorXd>(g, 65);
}

void InterceptInPlanesQuadraticOptimizer::qp_setup(QPSettings qpsettings [[maybe_unused]]) {

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

    H[0] = (((lamg[0]*((0.5)*(xopt[26]+xopt[26])))+(lamg[1]*((0.5)*(xopt[27]+xopt[27]))))+(lamg[2]*((0.5)*(xopt[28]+xopt[28]))));
    H[5] = lamg[0];
    H[6] = lamg[1];
    H[7] = lamg[2];
    H[26] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[27] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[28] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[47] = lamg[6];
    H[48] = lamg[7];
    H[49] = lamg[8];
    H[66] = (((lamg[21]*((0.5)*(xopt[29]+xopt[29])))+(lamg[22]*((0.5)*(xopt[30]+xopt[30]))))+(lamg[23]*((0.5)*(xopt[31]+xopt[31]))));
    H[82] = lamg[21];
    H[83] = lamg[22];
    H[84] = lamg[23];
    H[94] = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    H[95] = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    H[96] = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    H[325] = lamg[0];
    H[390] = lamg[1];
    H[455] = lamg[2];
    H[1106] = lamg[21];
    H[1171] = lamg[22];
    H[1236] = lamg[23];
    H[1690] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[1755] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[1820] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[1886] = ((lamg[21]*(0.5*(xopt[1]+xopt[1])))+lamg[24]);
    H[1951] = ((lamg[22]*(0.5*(xopt[1]+xopt[1])))+lamg[25]);
    H[2016] = ((lamg[23]*(0.5*(xopt[1]+xopt[1])))+lamg[26]);
    H[2112] = 3.33333e+06;
    H[2178] = 3.33333e+06;
    H[2244] = 3.33333e+06;
    H[2310] = 3.33333e+06;
    H[2376] = 3.33333e+06;
    H[2442] = 3.33333e+06;
    H[2508] = 3.33333e+06;
    H[2574] = 3.33333e+06;
    H[2640] = 3.33333e+06;
    H[2706] = 3.33333e+06;
    H[2772] = 3.33333e+06;
    H[2838] = 3.33333e+06;
    H[3055] = lamg[6];
    H[3120] = lamg[7];
    H[3185] = lamg[8];
    H[3696] = 66.6667;
    H[3762] = 66.6667;
    H[3828] = 66.6667;
    H[3894] = 3.33333e+06;
    H[3960] = 3.33333e+06;
    H[4026] = 3.33333e+06;
    H[4092] = 3.33333e+06;
    H[4158] = 3.33333e+06;
    H[4224] = 3.33333e+06;

}

void InterceptInPlanesQuadraticOptimizer::update_vector_g(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    g[0] = 1;
    g[1] = 0.001;
    g[32] = (1.66667e+06*(xopt[32]+xopt[32]));
    g[33] = (1.66667e+06*(xopt[33]+xopt[33]));
    g[34] = (1.66667e+06*(xopt[34]+xopt[34]));
    g[35] = (1.66667e+06*(xopt[35]+xopt[35]));
    g[36] = (1.66667e+06*(xopt[36]+xopt[36]));
    g[37] = (1.66667e+06*(xopt[37]+xopt[37]));
    g[38] = (1.66667e+06*(xopt[38]+xopt[38]));
    g[39] = (1.66667e+06*(xopt[39]+xopt[39]));
    g[40] = (1.66667e+06*(xopt[40]+xopt[40]));
    g[41] = (1.66667e+06*(xopt[41]+xopt[41]));
    g[42] = (1.66667e+06*(xopt[42]+xopt[42]));
    g[43] = (1.66667e+06*(xopt[43]+xopt[43]));
    g[56] = (33.3333*(xopt[56]+xopt[56]));
    g[57] = (33.3333*(xopt[57]+xopt[57]));
    g[58] = (33.3333*(xopt[58]+xopt[58]));
    g[59] = (1.66667e+06*(xopt[59]+xopt[59]));
    g[60] = (1.66667e+06*(xopt[60]+xopt[60]));
    g[61] = (1.66667e+06*(xopt[61]+xopt[61]));
    g[62] = (1.66667e+06*(xopt[62]+xopt[62]));
    g[63] = (1.66667e+06*(xopt[63]+xopt[63]));
    g[64] = (1.66667e+06*(xopt[64]+xopt[64]));

}

void InterceptInPlanesQuadraticOptimizer::update_matrix_A(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;

    A[0] = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[26])))+(xopt[0]*xopt[26])));
    A[2] = 1;
    A[5] = xopt[0];
    A[8] = -1;
    A[26] = (0.5*sq(xopt[0]));
    A[32] = 1;
    A[65] = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[27])))+(xopt[0]*xopt[27])));
    A[68] = 1;
    A[71] = xopt[0];
    A[74] = -1;
    A[92] = (0.5*sq(xopt[0]));
    A[98] = 1;
    A[130] = (0.5*((xopt[7]+(xopt[7]+(xopt[0]*xopt[28])))+(xopt[0]*xopt[28])));
    A[134] = 1;
    A[137] = xopt[0];
    A[140] = -1;
    A[158] = (0.5*sq(xopt[0]));
    A[164] = 1;
    A[195] = (0.5*(xopt[26]+xopt[26]));
    A[200] = 1;
    A[206] = -1;
    A[221] = xopt[0];
    A[230] = 1;
    A[260] = (0.5*(xopt[27]+xopt[27]));
    A[266] = 1;
    A[272] = -1;
    A[287] = xopt[0];
    A[296] = 1;
    A[325] = (0.5*(xopt[28]+xopt[28]));
    A[332] = 1;
    A[338] = -1;
    A[353] = xopt[0];
    A[362] = 1;
    A[390] = (0.5*(xopt[47]+xopt[47]));
    A[434] = 1;
    A[437] = xopt[0];
    A[440] = -1;
    A[455] = (0.5*(xopt[48]+xopt[48]));
    A[500] = 1;
    A[503] = xopt[0];
    A[506] = -1;
    A[520] = (0.5*(xopt[49]+xopt[49]));
    A[566] = 1;
    A[569] = xopt[0];
    A[572] = -1;
    A[632] = 1;
    A[638] = -1;
    A[698] = 1;
    A[704] = -1;
    A[764] = 1;
    A[770] = -1;
    A[788] = 1;
    A[830] = -1;
    A[836] = 1;
    A[854] = 1;
    A[896] = -1;
    A[902] = 1;
    A[920] = 1;
    A[962] = -1;
    A[968] = 1;
    A[983] = 1;
    A[989] = -1;
    A[1034] = 1;
    A[1049] = 1;
    A[1055] = -1;
    A[1100] = 1;
    A[1115] = 1;
    A[1121] = -1;
    A[1166] = 1;
    A[1181] = 1;
    A[1187] = -1;
    A[1232] = 1;
    A[1247] = 1;
    A[1253] = -1;
    A[1298] = 1;
    A[1313] = 1;
    A[1319] = -1;
    A[1364] = 1;
    A[1366] = (0.5*((xopt[17]+(xopt[17]+(xopt[1]*xopt[29])))+(xopt[1]*xopt[29])));
    A[1379] = 1;
    A[1382] = xopt[1];
    A[1385] = -1;
    A[1394] = (0.5*sq(xopt[1]));
    A[1403] = 1;
    A[1431] = (0.5*((xopt[18]+(xopt[18]+(xopt[1]*xopt[30])))+(xopt[1]*xopt[30])));
    A[1445] = 1;
    A[1448] = xopt[1];
    A[1451] = -1;
    A[1460] = (0.5*sq(xopt[1]));
    A[1469] = 1;
    A[1496] = (0.5*((xopt[19]+(xopt[19]+(xopt[1]*xopt[31])))+(xopt[1]*xopt[31])));
    A[1511] = 1;
    A[1514] = xopt[1];
    A[1517] = -1;
    A[1526] = (0.5*sq(xopt[1]));
    A[1535] = 1;
    A[1561] = (0.5*(xopt[29]+xopt[29]));
    A[1577] = 1;
    A[1583] = -1;
    A[1589] = xopt[1];
    A[1601] = 1;
    A[1626] = (0.5*(xopt[30]+xopt[30]));
    A[1643] = 1;
    A[1649] = -1;
    A[1655] = xopt[1];
    A[1667] = 1;
    A[1691] = (0.5*(xopt[31]+xopt[31]));
    A[1709] = 1;
    A[1715] = -1;
    A[1721] = xopt[1];
    A[1733] = 1;
    A[1763] = param[3];
    A[1764] = param[4];
    A[1765] = param[5];
    A[1828] = param[9];
    A[1829] = param[10];
    A[1830] = param[11];
    A[1893] = param[15];
    A[1894] = param[16];
    A[1895] = param[17];
    A[1958] = param[21];
    A[1959] = param[22];
    A[1960] = param[23];
    A[2023] = param[27];
    A[2024] = param[28];
    A[2025] = param[29];
    A[2088] = param[33];
    A[2089] = param[34];
    A[2090] = param[35];
    A[2153] = param[39];
    A[2154] = param[40];
    A[2155] = param[41];
    A[2218] = param[45];
    A[2219] = param[46];
    A[2220] = param[47];
    A[2283] = param[51];
    A[2284] = param[52];
    A[2285] = param[53];
    A[2348] = param[57];
    A[2349] = param[58];
    A[2350] = param[59];
    A[2425] = param[3];
    A[2426] = param[4];
    A[2427] = param[5];
    A[2490] = param[9];
    A[2491] = param[10];
    A[2492] = param[11];
    A[2555] = param[15];
    A[2556] = param[16];
    A[2557] = param[17];
    A[2620] = param[21];
    A[2621] = param[22];
    A[2622] = param[23];
    A[2685] = param[27];
    A[2686] = param[28];
    A[2687] = param[29];
    A[2750] = param[33];
    A[2751] = param[34];
    A[2752] = param[35];
    A[2815] = param[39];
    A[2816] = param[40];
    A[2817] = param[41];
    A[2880] = param[45];
    A[2881] = param[46];
    A[2882] = param[47];
    A[2945] = param[51];
    A[2946] = param[52];
    A[2947] = param[53];
    A[3010] = param[57];
    A[3011] = param[58];
    A[3012] = param[59];

}

void InterceptInPlanesQuadraticOptimizer::update_vectors_bA(problem_parameters *prob_params[[maybe_unused]], problem_solution *prev_qpsolution) {
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
    lbA[15] = lbg[15];
    lbA[16] = lbg[16];
    lbA[17] = lbg[17];
    lbA[18] = lbg[18];
    lbA[19] = lbg[19];
    lbA[20] = lbg[20];
    lbA[21] = lbg[21];
    lbA[22] = lbg[22];
    lbA[23] = lbg[23];
    lbA[24] = lbg[24];
    lbA[25] = lbg[25];
    lbA[26] = lbg[26];
    lbA[27] = lbg[27];
    lbA[28] = lbg[28];
    lbA[29] = lbg[29];
    lbA[30] = lbg[30];
    lbA[31] = lbg[31];
    lbA[32] = lbg[32];
    lbA[33] = lbg[33];
    lbA[34] = lbg[34];
    lbA[35] = lbg[35];
    lbA[36] = lbg[36];
    lbA[37] = lbg[37];
    lbA[38] = lbg[38];
    lbA[39] = lbg[39];
    lbA[40] = lbg[40];
    lbA[41] = lbg[41];
    lbA[42] = lbg[42];
    lbA[43] = lbg[43];
    lbA[44] = lbg[44];
    lbA[45] = lbg[45];
    lbA[46] = lbg[46];
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
    ubA[15] = ubg[15];
    ubA[16] = ubg[16];
    ubA[17] = ubg[17];
    ubA[18] = ubg[18];
    ubA[19] = ubg[19];
    ubA[20] = ubg[20];
    ubA[21] = ubg[21];
    ubA[22] = ubg[22];
    ubA[23] = ubg[23];
    ubA[24] = ubg[24];
    ubA[25] = ubg[25];
    ubA[26] = ubg[26];
    ubA[27] = ubg[27];
    ubA[28] = ubg[28];
    ubA[29] = ubg[29];
    ubA[30] = ubg[30];
    ubA[31] = ubg[31];
    ubA[32] = ubg[32];
    ubA[33] = ubg[33];
    ubA[34] = ubg[34];
    ubA[35] = ubg[35];
    ubA[36] = ubg[36];
    ubA[37] = ubg[37];
    ubA[38] = ubg[38];
    ubA[39] = ubg[39];
    ubA[40] = ubg[40];
    ubA[41] = ubg[41];
    ubA[42] = ubg[42];
    ubA[43] = ubg[43];
    ubA[44] = ubg[44];
    ubA[45] = ubg[45];
    ubA[46] = ubg[46];


    for (uint i = 0; i < 47; i++) {
        ubA[i] -= prev_qpsolution->Z[65 + i];
        lbA[i] -= prev_qpsolution->Z[65 + i];
    }
}

void InterceptInPlanesQuadraticOptimizer::update_vectors_bx(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    for (uint lbxk = 0; lbxk < prob_params->lbx.size(); lbxk++) {
        lb[lbxk] = prob_params->lbx[lbxk] - prev_qpsolution->Xopt[lbxk];
    }

    for (uint ubxk = 0; ubxk < prob_params->ubx.size(); ubxk++) {
        ub[ubxk] = prob_params->ubx[ubxk] - prev_qpsolution->Xopt[ubxk];
    }
}

problem_solution InterceptInPlanesQuadraticOptimizer::solve(problem_parameters *prob_params, bool init, double cpu_time) {
    problem_solution prev_qpsolution(prob_params, constraints(prob_params->X0, prob_params->param));
    return solve(prob_params, &prev_qpsolution, init, cpu_time);
}

problem_solution InterceptInPlanesQuadraticOptimizer::solve(problem_parameters *prob_params, problem_solution *prev_qpsolution, bool init, double cpu_time) {
    update_matrix_H(prob_params, prev_qpsolution);
    update_vector_g(prob_params, prev_qpsolution);
    update_matrix_A(prob_params, prev_qpsolution);
    update_vectors_bA(prob_params, prev_qpsolution);
    update_vectors_bx(prob_params, prev_qpsolution);

    int solver_status;
    int nwsr = 5 * (112);
    if (cpu_time > 0)
        _cpu_time = &cpu_time;
    else
        _cpu_time = nullptr;

    if (!solver.isInitialised() || init)
        solver_status = solver.init(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);
    else
        solver_status = solver.hotstart(H, g, A, lb, ub, lbA, ubA, nwsr, _cpu_time);

    real_t primal[65];
    solver.getPrimalSolution(primal);
    Eigen::VectorXd xopt = prev_qpsolution->Xopt + Eigen::Map<Eigen::VectorXd>(primal, 65);
    real_t _dual[112];
    solver.getDualSolution(_dual);
    Eigen::VectorXd lagrange_multiplier = -Eigen::Map<Eigen::VectorXd>(&(_dual[65]), 47);

    problem_solution ret = problem_solution(xopt, constraints(xopt, prob_params->param), lagrange_multiplier, solver.getObjVal(), qp_return_status(solver_status));
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
