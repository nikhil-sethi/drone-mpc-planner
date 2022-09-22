#include "common.h"
#include "intercept_in_planes_optimizer_interface.h"
#include "flightarea.h"
#include "plane.h"
#include "ocptester.h"
#include <chrono>

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(InterceptInPlanes) {
    xmls::DroneParameters dparams;
    InterceptInPlanesOptimizerInterface opti;
    OcpTester ocptester;
    tracking::TrackData drone;
    tracking::TrackData insect;
    FlightArea flightarea;
};

// TEST(InterceptInPlanes, functiontest) {
//     opti.init(&(ocptester.thrust));
//     tracking::TrackData drone;
//     drone.state.pos = {0, -2.9, -1.5};
//     drone.state.vel = {0, 0, 0};
//     tracking::TrackData insect;
//     insect.state.pos = {-1, -0.5, -2};
//     insect.state.vel = {0, 0, 0};
//     auto opti_res = opti.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 8));
//     std::cout << "position_to_intercept: " << opti_res.position_to_intercept << std::endl;

//     CHECK(opti_res.valid);
//     CHECK(norm(opti_res.position_to_intercept - insect.state.pos) < 0.01);
// }

// TEST(InterceptInPlanes, functiontest_copy) {
//     opti.init(&(ocptester.thrust));
//     tracking::TrackData drone;
//     drone.state.pos = {0, -2.9, -1.5};
//     drone.state.vel = {0, 0, 0};
//     tracking::TrackData insect;
//     insect.state.pos = {-1, -0.5, -2};
//     insect.state.vel = {0, 0, 0};
//     auto opti_res = opti.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 8));
//     std::cout << "position_to_intercept: " << opti_res.position_to_intercept << std::endl;

//     CHECK(opti_res.valid);
//     CHECK(norm(opti_res.position_to_intercept - insect.state.pos) < 0.01);
// }

// TEST(InterceptInPlanes, case_test) {
//     opti.init(&(ocptester.thrust));
//     drone.state.pos = {0, 0, -1};
//     drone.state.vel = {0, -1, 0};
//     insect.state.pos = {0, 0, 0};
//     insect.state.vel = {-0, -0, -0};

//     sqp_solver_configuration sqp_config(50, 1e-6, 1e-6, 1e-9);
//     opti.sqp_setup(sqp_config);
//     auto opti_res = opti.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 6));
//     CHECK(opti_res.time_to_intercept < 0.36);
//     CHECK(opti_res.valid);
// }


// TEST(InterceptInPlanes, case_test_casadi) {
//     opti.init(&(ocptester.thrust));
//     drone.state.pos = {0, 0, -1};
//     drone.state.vel = {0, -1, 0};
//     insect.state.pos = {0, 0, 0};
//     insect.state.vel = {-0, -0, -0};
//     sqp_solver_configuration sqp_config(50, 1e-6, 1e-6, 1e-9);
//     opti.sqp_setup(sqp_config);
//     opti.init_casadi("../../ocp_design/casadi/intercept_in_planes/intercept_in_planes_optimizer.so");
//     auto opti_res = opti.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 6));
//     CHECK(opti_res.time_to_intercept < 0.36);
//     CHECK(opti_res.valid);
// }

// TEST(InterceptInPlanes, application_test) {
//     opti.init(&(ocptester.thrust));
//     std::vector<Plane> planes;
//     std::tie(drone, insect, planes) = ocptester.application_test();
//     auto opti_res = opti.find_best_interception(drone, insect, planes);


//     std::cout << "drone: " << drone.state.pos << ", " << drone.state.vel << std::endl;
//     std::cout << "insect: " << insect.state.pos << ", " << insect.state.vel << std::endl;
//     std::cout << "time_to_intercept: " << opti_res.time_to_intercept << std::endl;
//     std::cout << "position_to_intercept: " << opti_res.position_to_intercept << std::endl;
//     std::cout << "acceleration_to_intercept: " << opti_res.acceleration_to_intercept << std::endl;

//     CHECK(opti_res.time_to_intercept < 0.36);
//     CHECK(opti_res.valid);
// }

// TEST(InterceptInPlanes, cmp_optimizers) {
//     opti.init(&(ocptester.thrust));

//     InterceptInPlanesOptimizerInterface opti_casadi;
//     opti_casadi.init(&(ocptester.thrust));
//     opti_casadi.init_casadi("../../ocp_design/casadi/intercept_in_planes/intercept_in_planes_optimizer.so");
//     std::tie(drone, insect) = ocptester.case_test();

//     auto opti_res = opti.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 6));
//     auto opti_res_casadi = opti_casadi.find_best_interception(drone, insect, ocptester.cube_planes(3.f, 6));

//     if (opti_res.valid != opti_res_casadi.valid) {
//         std::cout << "valid: " << opti_res.valid << " valid(casadi): " << opti_res_casadi.valid << std::endl;
//         std::cout << "tti: " << opti_res.time_to_intercept << " tti(casadi): " << opti_res_casadi.time_to_intercept << std::endl;
//     }
//     CHECK(opti_res.valid == opti_res_casadi.valid);

//     if (!(opti_res.time_to_intercept <= opti_res_casadi.time_to_intercept + 0.02))
//         std::cout << "tti: " << opti_res.time_to_intercept << " tti(casadi): " << opti_res_casadi.time_to_intercept << std::endl;
//     CHECK(opti_res.time_to_intercept <= opti_res_casadi.time_to_intercept + 0.02);
// }

// TEST(InterceptInPlanes, find_optimality_for_range_test_with_casadi) {
//     bool average_timing_ok, max_timing_ok, invalid_results_ok;
//     sqp_solver_configuration sqp_config(50, 1e-6, 1e-6, 1e-9);
//     std::cout << "Find optimum for given range test..." << std::endl;
//     // std::tie(average_timing_ok, max_timing_ok, invalid_results_ok) = ocptester.exec_range_test(intercept_in_planes, false, sqp_config);
//     std::tie(average_timing_ok, max_timing_ok, invalid_results_ok) = ocptester.exec_range_test(intercept_in_planes, true, sqp_config);
//     CHECK(average_timing_ok);
//     CHECK(max_timing_ok);
//     CHECK(invalid_results_ok);
// }

// TEST(InterceptInPlanes, overall_behavior_casadi) {
//     dparams.deserialize("../../xml/drone_anvil_superbee.xml");
//     auto stats = ocptester.exec_range_test(intercept_in_planes, true, sqp_solver_configuration());
//     CHECK(stats.average_optimizing_time_us < ocptester.realtime_boundary_ms);
//     CHECK(stats.max_optimizing_time_us < ocptester.realtime_boundary_ms);
//     CHECK(stats.invalid_optimization_results == 0);
// }


TEST(InterceptInPlanes, overall_behavior_linesearch) {
    dparams.deserialize("../../xml/drone_anvil_superbee.xml");
    QPSettings qpsettings = QPSettings();
    sqp_solver_configuration sqp_config = sqp_solver_configuration(30, 1e-6, 1e-6, 1e-9);
    ocptester.init_range_test(intercept_in_planes, false, sqp_config, &qpsettings);
    auto stats = ocptester.exec_range_test();
    CHECK(stats.average_optimizing_time_us < ocptester.realtime_boundary_ms);
    CHECK(stats.max_optimizing_time_us < ocptester.realtime_boundary_ms);
    CHECK(stats.invalid_optimization_results == 0);
}

// TEST(InterceptInPlanes, find_parameters) {
// ocptester.find_parameter(intercept_in_planes, false);
// }
