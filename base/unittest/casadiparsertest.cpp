#include <chrono>
#include <iostream>
#include "min_nlp_quadratic_optimizer.h"

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(CasadiParser) {
    MinNlpQuadraticOptimizer solver;
    double inf = std::numeric_limits<double>::infinity();
};

TEST(CasadiParser, debug) {
    solver.init();
    std::vector<double> X0 = {11, -1};
    std::vector<double> param = {};
    std::vector<double> lbg = {0, 0};
    std::vector<double> ubg = {0, 0};
    std::vector<double> lbx = {-inf, -inf};
    std::vector<double> ubx = {inf, inf};

    auto res = solver.solve(X0, lbg, ubg, lbx, ubx, param);

    CHECK(abs(res.Xopt.at(0) - 5) < 0.001);
    CHECK(abs(res.Xopt.at(1) - 5) < 0.001);

}
