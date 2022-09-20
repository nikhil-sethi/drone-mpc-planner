#include "intercept_in_planes_quad_opti_qpoases.h"
#include <intercept_in_planes_index.h>

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
    H[260] = lamg[0];
    H[325] = lamg[1];
    H[390] = lamg[2];
    H[845] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[910] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[975] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[1056] = 3.33333e+06;
    H[1122] = 3.33333e+06;
    H[1188] = 3.33333e+06;
    H[1254] = 3.33333e+06;
    H[1320] = 3.33333e+06;
    H[1386] = 3.33333e+06;
    H[1625] = lamg[6];
    H[1690] = lamg[7];
    H[1755] = lamg[8];
    H[2244] = (((lamg[21]*((0.5)*(xopt[47]+xopt[47])))+(lamg[22]*((0.5)*(xopt[48]+xopt[48]))))+(lamg[23]*((0.5)*(xopt[49]+xopt[49]))));
    H[2248] = lamg[21];
    H[2249] = lamg[22];
    H[2250] = lamg[23];
    H[2257] = ((lamg[21]*(0.5*(xopt[34]+xopt[34])))+lamg[24]);
    H[2258] = ((lamg[22]*(0.5*(xopt[34]+xopt[34])))+lamg[25]);
    H[2259] = ((lamg[23]*(0.5*(xopt[34]+xopt[34])))+lamg[26]);
    H[2504] = lamg[21];
    H[2569] = lamg[22];
    H[2634] = lamg[23];
    H[3089] = ((lamg[21]*(0.5*(xopt[34]+xopt[34])))+lamg[24]);
    H[3154] = ((lamg[22]*(0.5*(xopt[34]+xopt[34])))+lamg[25]);
    H[3219] = ((lamg[23]*(0.5*(xopt[34]+xopt[34])))+lamg[26]);
    H[3300] = 3.33333e+06;
    H[3366] = 3.33333e+06;
    H[3432] = 3.33333e+06;
    H[3498] = 3.33333e+06;
    H[3564] = 3.33333e+06;
    H[3630] = 3.33333e+06;
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
    g[16] = (1.66667e+06*(xopt[16]+xopt[16]));
    g[17] = (1.66667e+06*(xopt[17]+xopt[17]));
    g[18] = (1.66667e+06*(xopt[18]+xopt[18]));
    g[19] = (1.66667e+06*(xopt[19]+xopt[19]));
    g[20] = (1.66667e+06*(xopt[20]+xopt[20]));
    g[21] = (1.66667e+06*(xopt[21]+xopt[21]));
    g[34] = 0.001;
    g[50] = (1.66667e+06*(xopt[50]+xopt[50]));
    g[51] = (1.66667e+06*(xopt[51]+xopt[51]));
    g[52] = (1.66667e+06*(xopt[52]+xopt[52]));
    g[53] = (1.66667e+06*(xopt[53]+xopt[53]));
    g[54] = (1.66667e+06*(xopt[54]+xopt[54]));
    g[55] = (1.66667e+06*(xopt[55]+xopt[55]));
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
    constraints[12] = ((xopt[7]-xopt[28])+xopt[56]);
    constraints[13] = ((xopt[8]-xopt[29])+xopt[57]);
    constraints[14] = ((xopt[9]-xopt[30])+xopt[58]);
    constraints[15] = ((xopt[7]-xopt[35])+xopt[59]);
    constraints[16] = ((xopt[8]-xopt[36])+xopt[60]);
    constraints[17] = ((xopt[9]-xopt[37])+xopt[61]);
    constraints[18] = ((xopt[10]-xopt[38])+xopt[62]);
    constraints[19] = ((xopt[11]-xopt[39])+xopt[63]);
    constraints[20] = ((xopt[12]-xopt[40])+xopt[64]);
    constraints[21] = (((xopt[35]+((xopt[34]*(xopt[38]+(xopt[38]+(xopt[34]*xopt[47]))))/2))-xopt[41])+xopt[50]);
    constraints[22] = (((xopt[36]+((xopt[34]*(xopt[39]+(xopt[39]+(xopt[34]*xopt[48]))))/2))-xopt[42])+xopt[51]);
    constraints[23] = (((xopt[37]+((xopt[34]*(xopt[40]+(xopt[40]+(xopt[34]*xopt[49]))))/2))-xopt[43])+xopt[52]);
    constraints[24] = (((xopt[38]+((xopt[34]*(xopt[47]+xopt[47]))/2))-xopt[44])+xopt[53]);
    constraints[25] = (((xopt[39]+((xopt[34]*(xopt[48]+xopt[48]))/2))-xopt[45])+xopt[54]);
    constraints[26] = (((xopt[40]+((xopt[34]*(xopt[49]+xopt[49]))/2))-xopt[46])+xopt[55]);
    constraints[27] = (((param[3]*(xopt[7]-param[0]))+(param[4]*(xopt[8]-param[1])))+(param[5]*(xopt[9]-param[2])));
    constraints[28] = (((param[9]*(xopt[7]-param[6]))+(param[10]*(xopt[8]-param[7])))+(param[11]*(xopt[9]-param[8])));
    constraints[29] = (((param[15]*(xopt[7]-param[12]))+(param[16]*(xopt[8]-param[13])))+(param[17]*(xopt[9]-param[14])));
    constraints[30] = (((param[21]*(xopt[7]-param[18]))+(param[22]*(xopt[8]-param[19])))+(param[23]*(xopt[9]-param[20])));
    constraints[31] = (((param[27]*(xopt[7]-param[24]))+(param[28]*(xopt[8]-param[25])))+(param[29]*(xopt[9]-param[26])));
    constraints[32] = (((param[33]*(xopt[7]-param[30]))+(param[34]*(xopt[8]-param[31])))+(param[35]*(xopt[9]-param[32])));
    constraints[33] = (((param[39]*(xopt[7]-param[36]))+(param[40]*(xopt[8]-param[37])))+(param[41]*(xopt[9]-param[38])));
    constraints[34] = (((param[45]*(xopt[7]-param[42]))+(param[46]*(xopt[8]-param[43])))+(param[47]*(xopt[9]-param[44])));
    constraints[35] = (((param[51]*(xopt[7]-param[48]))+(param[52]*(xopt[8]-param[49])))+(param[53]*(xopt[9]-param[50])));
    constraints[36] = (((param[57]*(xopt[7]-param[54]))+(param[58]*(xopt[8]-param[55])))+(param[59]*(xopt[9]-param[56])));
    constraints[37] = (((param[3]*(xopt[41]-param[0]))+(param[4]*(xopt[42]-param[1])))+(param[5]*(xopt[43]-param[2])));
    constraints[38] = (((param[9]*(xopt[41]-param[6]))+(param[10]*(xopt[42]-param[7])))+(param[11]*(xopt[43]-param[8])));
    constraints[39] = (((param[15]*(xopt[41]-param[12]))+(param[16]*(xopt[42]-param[13])))+(param[17]*(xopt[43]-param[14])));
    constraints[40] = (((param[21]*(xopt[41]-param[18]))+(param[22]*(xopt[42]-param[19])))+(param[23]*(xopt[43]-param[20])));
    constraints[41] = (((param[27]*(xopt[41]-param[24]))+(param[28]*(xopt[42]-param[25])))+(param[29]*(xopt[43]-param[26])));
    constraints[42] = (((param[33]*(xopt[41]-param[30]))+(param[34]*(xopt[42]-param[31])))+(param[35]*(xopt[43]-param[32])));
    constraints[43] = (((param[39]*(xopt[41]-param[36]))+(param[40]*(xopt[42]-param[37])))+(param[41]*(xopt[43]-param[38])));
    constraints[44] = (((param[45]*(xopt[41]-param[42]))+(param[46]*(xopt[42]-param[43])))+(param[47]*(xopt[43]-param[44])));
    constraints[45] = (((param[51]*(xopt[41]-param[48]))+(param[52]*(xopt[42]-param[49])))+(param[53]*(xopt[43]-param[50])));
    constraints[46] = (((param[57]*(xopt[41]-param[54]))+(param[58]*(xopt[42]-param[55])))+(param[59]*(xopt[43]-param[56])));

    return constraints;
}

Eigen::MatrixXd InterceptInPlanesQuadraticOptimizer::constraint_derivative(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
    Eigen::VectorXd xopt = prev_qpsolution->Xopt;
    Eigen::VectorXd param = prob_params->param;
    Eigen::MatrixXd dconstraints = Eigen::MatrixXd(47, 65).setZero();
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
    dconstraints(12, 56) = 1;
    dconstraints(13, 8) = 1;
    dconstraints(13, 29) = -1;
    dconstraints(13, 57) = 1;
    dconstraints(14, 9) = 1;
    dconstraints(14, 30) = -1;
    dconstraints(14, 58) = 1;
    dconstraints(15, 7) = 1;
    dconstraints(15, 35) = -1;
    dconstraints(15, 59) = 1;
    dconstraints(16, 8) = 1;
    dconstraints(16, 36) = -1;
    dconstraints(16, 60) = 1;
    dconstraints(17, 9) = 1;
    dconstraints(17, 37) = -1;
    dconstraints(17, 61) = 1;
    dconstraints(18, 10) = 1;
    dconstraints(18, 38) = -1;
    dconstraints(18, 62) = 1;
    dconstraints(19, 11) = 1;
    dconstraints(19, 39) = -1;
    dconstraints(19, 63) = 1;
    dconstraints(20, 12) = 1;
    dconstraints(20, 40) = -1;
    dconstraints(20, 64) = 1;
    dconstraints(21, 34) = (0.5*((xopt[38]+(xopt[38]+(xopt[34]*xopt[47])))+(xopt[34]*xopt[47])));
    dconstraints(21, 35) = 1;
    dconstraints(21, 38) = xopt[34];
    dconstraints(21, 41) = -1;
    dconstraints(21, 47) = (0.5*sq(xopt[34]));
    dconstraints(21, 50) = 1;
    dconstraints(22, 34) = (0.5*((xopt[39]+(xopt[39]+(xopt[34]*xopt[48])))+(xopt[34]*xopt[48])));
    dconstraints(22, 36) = 1;
    dconstraints(22, 39) = xopt[34];
    dconstraints(22, 42) = -1;
    dconstraints(22, 48) = (0.5*sq(xopt[34]));
    dconstraints(22, 51) = 1;
    dconstraints(23, 34) = (0.5*((xopt[40]+(xopt[40]+(xopt[34]*xopt[49])))+(xopt[34]*xopt[49])));
    dconstraints(23, 37) = 1;
    dconstraints(23, 40) = xopt[34];
    dconstraints(23, 43) = -1;
    dconstraints(23, 49) = (0.5*sq(xopt[34]));
    dconstraints(23, 52) = 1;
    dconstraints(24, 34) = (0.5*(xopt[47]+xopt[47]));
    dconstraints(24, 38) = 1;
    dconstraints(24, 44) = -1;
    dconstraints(24, 47) = xopt[34];
    dconstraints(24, 53) = 1;
    dconstraints(25, 34) = (0.5*(xopt[48]+xopt[48]));
    dconstraints(25, 39) = 1;
    dconstraints(25, 45) = -1;
    dconstraints(25, 48) = xopt[34];
    dconstraints(25, 54) = 1;
    dconstraints(26, 34) = (0.5*(xopt[49]+xopt[49]));
    dconstraints(26, 40) = 1;
    dconstraints(26, 46) = -1;
    dconstraints(26, 49) = xopt[34];
    dconstraints(26, 55) = 1;
    dconstraints(27, 7) = param[3];
    dconstraints(27, 8) = param[4];
    dconstraints(27, 9) = param[5];
    dconstraints(28, 7) = param[9];
    dconstraints(28, 8) = param[10];
    dconstraints(28, 9) = param[11];
    dconstraints(29, 7) = param[15];
    dconstraints(29, 8) = param[16];
    dconstraints(29, 9) = param[17];
    dconstraints(30, 7) = param[21];
    dconstraints(30, 8) = param[22];
    dconstraints(30, 9) = param[23];
    dconstraints(31, 7) = param[27];
    dconstraints(31, 8) = param[28];
    dconstraints(31, 9) = param[29];
    dconstraints(32, 7) = param[33];
    dconstraints(32, 8) = param[34];
    dconstraints(32, 9) = param[35];
    dconstraints(33, 7) = param[39];
    dconstraints(33, 8) = param[40];
    dconstraints(33, 9) = param[41];
    dconstraints(34, 7) = param[45];
    dconstraints(34, 8) = param[46];
    dconstraints(34, 9) = param[47];
    dconstraints(35, 7) = param[51];
    dconstraints(35, 8) = param[52];
    dconstraints(35, 9) = param[53];
    dconstraints(36, 7) = param[57];
    dconstraints(36, 8) = param[58];
    dconstraints(36, 9) = param[59];
    dconstraints(37, 41) = param[3];
    dconstraints(37, 42) = param[4];
    dconstraints(37, 43) = param[5];
    dconstraints(38, 41) = param[9];
    dconstraints(38, 42) = param[10];
    dconstraints(38, 43) = param[11];
    dconstraints(39, 41) = param[15];
    dconstraints(39, 42) = param[16];
    dconstraints(39, 43) = param[17];
    dconstraints(40, 41) = param[21];
    dconstraints(40, 42) = param[22];
    dconstraints(40, 43) = param[23];
    dconstraints(41, 41) = param[27];
    dconstraints(41, 42) = param[28];
    dconstraints(41, 43) = param[29];
    dconstraints(42, 41) = param[33];
    dconstraints(42, 42) = param[34];
    dconstraints(42, 43) = param[35];
    dconstraints(43, 41) = param[39];
    dconstraints(43, 42) = param[40];
    dconstraints(43, 43) = param[41];
    dconstraints(44, 41) = param[45];
    dconstraints(44, 42) = param[46];
    dconstraints(44, 43) = param[47];
    dconstraints(45, 41) = param[51];
    dconstraints(45, 42) = param[52];
    dconstraints(45, 43) = param[53];
    dconstraints(46, 41) = param[57];
    dconstraints(46, 42) = param[58];
    dconstraints(46, 43) = param[59];

    return dconstraints;
}

void InterceptInPlanesQuadraticOptimizer::update_matrix_H(problem_parameters *prob_params, problem_solution *prev_qpsolution) {
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
    H[260] = lamg[0];
    H[325] = lamg[1];
    H[390] = lamg[2];
    H[845] = ((lamg[0]*(0.5*(xopt[0]+xopt[0])))+lamg[3]);
    H[910] = ((lamg[1]*(0.5*(xopt[0]+xopt[0])))+lamg[4]);
    H[975] = ((lamg[2]*(0.5*(xopt[0]+xopt[0])))+lamg[5]);
    H[1056] = 3.33333e+06;
    H[1122] = 3.33333e+06;
    H[1188] = 3.33333e+06;
    H[1254] = 3.33333e+06;
    H[1320] = 3.33333e+06;
    H[1386] = 3.33333e+06;
    H[1625] = lamg[6];
    H[1690] = lamg[7];
    H[1755] = lamg[8];
    H[2244] = (((lamg[21]*((0.5)*(xopt[47]+xopt[47])))+(lamg[22]*((0.5)*(xopt[48]+xopt[48]))))+(lamg[23]*((0.5)*(xopt[49]+xopt[49]))));
    H[2248] = lamg[21];
    H[2249] = lamg[22];
    H[2250] = lamg[23];
    H[2257] = ((lamg[21]*(0.5*(xopt[34]+xopt[34])))+lamg[24]);
    H[2258] = ((lamg[22]*(0.5*(xopt[34]+xopt[34])))+lamg[25]);
    H[2259] = ((lamg[23]*(0.5*(xopt[34]+xopt[34])))+lamg[26]);
    H[2504] = lamg[21];
    H[2569] = lamg[22];
    H[2634] = lamg[23];
    H[3089] = ((lamg[21]*(0.5*(xopt[34]+xopt[34])))+lamg[24]);
    H[3154] = ((lamg[22]*(0.5*(xopt[34]+xopt[34])))+lamg[25]);
    H[3219] = ((lamg[23]*(0.5*(xopt[34]+xopt[34])))+lamg[26]);
    H[3300] = 3.33333e+06;
    H[3366] = 3.33333e+06;
    H[3432] = 3.33333e+06;
    H[3498] = 3.33333e+06;
    H[3564] = 3.33333e+06;
    H[3630] = 3.33333e+06;
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
    g[16] = (1.66667e+06*(xopt[16]+xopt[16]));
    g[17] = (1.66667e+06*(xopt[17]+xopt[17]));
    g[18] = (1.66667e+06*(xopt[18]+xopt[18]));
    g[19] = (1.66667e+06*(xopt[19]+xopt[19]));
    g[20] = (1.66667e+06*(xopt[20]+xopt[20]));
    g[21] = (1.66667e+06*(xopt[21]+xopt[21]));
    g[34] = 0.001;
    g[50] = (1.66667e+06*(xopt[50]+xopt[50]));
    g[51] = (1.66667e+06*(xopt[51]+xopt[51]));
    g[52] = (1.66667e+06*(xopt[52]+xopt[52]));
    g[53] = (1.66667e+06*(xopt[53]+xopt[53]));
    g[54] = (1.66667e+06*(xopt[54]+xopt[54]));
    g[55] = (1.66667e+06*(xopt[55]+xopt[55]));
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

    A[0] = (0.5*((xopt[4]+(xopt[4]+(xopt[0]*xopt[13])))+(xopt[0]*xopt[13])));
    A[1] = 1;
    A[4] = xopt[0];
    A[7] = -1;
    A[13] = (0.5*sq(xopt[0]));
    A[16] = 1;
    A[65] = (0.5*((xopt[5]+(xopt[5]+(xopt[0]*xopt[14])))+(xopt[0]*xopt[14])));
    A[67] = 1;
    A[70] = xopt[0];
    A[73] = -1;
    A[79] = (0.5*sq(xopt[0]));
    A[82] = 1;
    A[130] = (0.5*((xopt[6]+(xopt[6]+(xopt[0]*xopt[15])))+(xopt[0]*xopt[15])));
    A[133] = 1;
    A[136] = xopt[0];
    A[139] = -1;
    A[145] = (0.5*sq(xopt[0]));
    A[148] = 1;
    A[195] = (0.5*(xopt[13]+xopt[13]));
    A[199] = 1;
    A[205] = -1;
    A[208] = xopt[0];
    A[214] = 1;
    A[260] = (0.5*(xopt[14]+xopt[14]));
    A[265] = 1;
    A[271] = -1;
    A[274] = xopt[0];
    A[280] = 1;
    A[325] = (0.5*(xopt[15]+xopt[15]));
    A[331] = 1;
    A[337] = -1;
    A[340] = xopt[0];
    A[346] = 1;
    A[390] = (0.5*(xopt[25]+xopt[25]));
    A[412] = 1;
    A[415] = xopt[0];
    A[418] = -1;
    A[455] = (0.5*(xopt[26]+xopt[26]));
    A[478] = 1;
    A[481] = xopt[0];
    A[484] = -1;
    A[520] = (0.5*(xopt[27]+xopt[27]));
    A[544] = 1;
    A[547] = xopt[0];
    A[550] = -1;
    A[610] = 1;
    A[616] = -1;
    A[676] = 1;
    A[682] = -1;
    A[742] = 1;
    A[748] = -1;
    A[787] = 1;
    A[808] = -1;
    A[836] = 1;
    A[853] = 1;
    A[874] = -1;
    A[902] = 1;
    A[919] = 1;
    A[940] = -1;
    A[968] = 1;
    A[982] = 1;
    A[1010] = -1;
    A[1034] = 1;
    A[1048] = 1;
    A[1076] = -1;
    A[1100] = 1;
    A[1114] = 1;
    A[1142] = -1;
    A[1166] = 1;
    A[1180] = 1;
    A[1208] = -1;
    A[1232] = 1;
    A[1246] = 1;
    A[1274] = -1;
    A[1298] = 1;
    A[1312] = 1;
    A[1340] = -1;
    A[1364] = 1;
    A[1399] = (0.5*((xopt[38]+(xopt[38]+(xopt[34]*xopt[47])))+(xopt[34]*xopt[47])));
    A[1400] = 1;
    A[1403] = xopt[34];
    A[1406] = -1;
    A[1412] = (0.5*sq(xopt[34]));
    A[1415] = 1;
    A[1464] = (0.5*((xopt[39]+(xopt[39]+(xopt[34]*xopt[48])))+(xopt[34]*xopt[48])));
    A[1466] = 1;
    A[1469] = xopt[34];
    A[1472] = -1;
    A[1478] = (0.5*sq(xopt[34]));
    A[1481] = 1;
    A[1529] = (0.5*((xopt[40]+(xopt[40]+(xopt[34]*xopt[49])))+(xopt[34]*xopt[49])));
    A[1532] = 1;
    A[1535] = xopt[34];
    A[1538] = -1;
    A[1544] = (0.5*sq(xopt[34]));
    A[1547] = 1;
    A[1594] = (0.5*(xopt[47]+xopt[47]));
    A[1598] = 1;
    A[1604] = -1;
    A[1607] = xopt[34];
    A[1613] = 1;
    A[1659] = (0.5*(xopt[48]+xopt[48]));
    A[1664] = 1;
    A[1670] = -1;
    A[1673] = xopt[34];
    A[1679] = 1;
    A[1724] = (0.5*(xopt[49]+xopt[49]));
    A[1730] = 1;
    A[1736] = -1;
    A[1739] = xopt[34];
    A[1745] = 1;
    A[1762] = param[3];
    A[1763] = param[4];
    A[1764] = param[5];
    A[1827] = param[9];
    A[1828] = param[10];
    A[1829] = param[11];
    A[1892] = param[15];
    A[1893] = param[16];
    A[1894] = param[17];
    A[1957] = param[21];
    A[1958] = param[22];
    A[1959] = param[23];
    A[2022] = param[27];
    A[2023] = param[28];
    A[2024] = param[29];
    A[2087] = param[33];
    A[2088] = param[34];
    A[2089] = param[35];
    A[2152] = param[39];
    A[2153] = param[40];
    A[2154] = param[41];
    A[2217] = param[45];
    A[2218] = param[46];
    A[2219] = param[47];
    A[2282] = param[51];
    A[2283] = param[52];
    A[2284] = param[53];
    A[2347] = param[57];
    A[2348] = param[58];
    A[2349] = param[59];
    A[2446] = param[3];
    A[2447] = param[4];
    A[2448] = param[5];
    A[2511] = param[9];
    A[2512] = param[10];
    A[2513] = param[11];
    A[2576] = param[15];
    A[2577] = param[16];
    A[2578] = param[17];
    A[2641] = param[21];
    A[2642] = param[22];
    A[2643] = param[23];
    A[2706] = param[27];
    A[2707] = param[28];
    A[2708] = param[29];
    A[2771] = param[33];
    A[2772] = param[34];
    A[2773] = param[35];
    A[2836] = param[39];
    A[2837] = param[40];
    A[2838] = param[41];
    A[2901] = param[45];
    A[2902] = param[46];
    A[2903] = param[47];
    A[2966] = param[51];
    A[2967] = param[52];
    A[2968] = param[53];
    A[3031] = param[57];
    A[3032] = param[58];
    A[3033] = param[59];

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
