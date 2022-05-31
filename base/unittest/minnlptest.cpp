
#include "min_nlp_optimizer_interface.h"
#include "ocptester.h"
#include <chrono>
#include <iostream>

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(MinNLP) {
    MinNlpOptimizerInterface opti;
    OcpTester ocptester;
};

// TEST(MinNLP, functiontest_casadi) {
//     opti.init();
//     opti.init_casadi("../../ocp_design/casadi/min_nlp/min_nlp_optimizer.so");
//     // opti.use_trustregionmethod = true;
//     cv::Point2f x0 = {10, 10};
//     auto opti_res = opti.find_best_solution(x0);
//     // std::cout << "xopt" << opti_res.xopt << std::endl;

//     CHECK(abs(opti_res.xopt.x - 1.1) < 0.01);
//     CHECK(abs(opti_res.xopt.y - 1. / 1.1) < 0.01);
// }

// TEST(MinNLP, functiontest) {
//     opti.init();
//     // opti.init_casadi("../../ocp_design/casadi/min_nlp/min_nlp_optimizer.so");
//     // opti.use_trustregionmethod = true;
//     cv::Point2f x0 = {10, 10};
//     auto opti_res = opti.find_best_solution(x0);
//     // std::cout << "xopt" << opti_res.xopt << std::endl;

//     CHECK(abs(opti_res.xopt.x - 1.1) < 0.01);
//     CHECK(abs(opti_res.xopt.y - 1. / 1.1) < 0.01);
// }
