#include "ocptester.h"
#include <stdlib.h>


bool OcpTester::is_tti_good(float optimizer_tti, float estimated_tti) {
    if (!((optimizer_tti - estimated_tti - accpt_tti_error) <= 0))
        std::cout << "est_tti: " << estimated_tti << ", opti_tti: " << optimizer_tti << std::endl;
    return ((optimizer_tti - estimated_tti - accpt_tti_error) <= 0);
}

bool OcpTester::is_optimizer_results_good(tracking::TrackData drone, tracking::TrackData insect, bool res_valid, float res_tti) {
    if (normf(drone.pos() - insect.pos()) < eps) {
        return false;
    } else {
        // Worst case estimation
        float pos_err = norm(drone.pos() - insect.pos());
        float vel_err = norm(drone.vel() - insect.vel());
        float est_tti = vel_err / thrust + sqrt(2 * pos_err / thrust);

        return res_valid && is_tti_good(res_tti, est_tti);
    }
}

std::vector<Plane> OcpTester::cube_planes(float cube_size, uint n_planes) {
    if (n_planes < 6)
        std::cout << "WARNING: At least 6 planes must be exported to close the cube. Got " << n_planes << "." << std::endl;

    std::vector<Plane> _planes;
    _planes.push_back(Plane(cube_size / 2, 0, 0, -1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(-cube_size / 2, 0, 0, 1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(0, cube_size / 2, 0, 0, -1, 0, unspecified_plane));
    _planes.push_back(Plane(0, -cube_size / 2, 0, 0, 1, 0, unspecified_plane));
    _planes.push_back(Plane(0, 0, -cube_size / 2, 0, 0, 1, unspecified_plane));
    _planes.push_back(Plane(0, 0, cube_size / 2, 0, 0, -1, unspecified_plane));

    if (n_planes > 6) {
        for (uint p = 6; p < n_planes; p++) {
            _planes.push_back(Plane(0, -cube_size, 0, 0, 1, 0, unspecified_plane));
        }
    }

    return _planes;
}


range_stats OcpTester::eval_range_data(range_data data) {
    range_stats stats;
    stats.average_optimizing_time_us = data.accumulated_optimizing_time_us / data.n_optimizer_tests;
    double var_calculation_time_us = 0;
    for (auto timing : data.timings_us) {
        var_calculation_time_us += pow(timing - stats.average_optimizing_time_us, 2);
    }
    stats. max_optimizing_time_us = data.max_optimizing_time_us;
    stats.timing_variance_us = sqrt(1. / (data.timings_us.size() - 1) * var_calculation_time_us);
    stats.sigma2_optimizing_time_ms = (stats.average_optimizing_time_us + 2 * stats.timing_variance_us) / 1000;
    stats.valid_optimizing_timings_percent = static_cast<double>(data.n_optimizer_tests - data.invalid_timings) / static_cast<double>(data.n_optimizer_tests) * 100.;
    stats.average_interception_time_s = data.accumulated_interception_time_s / data.interceptions_found;

    return stats;
}

void OcpTester::cout_header(optimizer_test optimizer_select) {
    if (optimizer_select == time_to_intercept)
        std::cout << "---------------------(TTI) time_to_intercept-------------------------------" << std::endl;
    else if (optimizer_select == intercept_in_planes)
        std::cout << "---------------------(IIPv3) intercept_in_planes-----------------------------" << std::endl;

}

void OcpTester::cout_setup(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config, bool enable_stress, bool disable_time_ensurance) {
    std::cout << "Setup:" << std::endl;

    switch (optimizer_select) {
        case intercept_in_planes: {
                std::cout << "- Quadratic solver: " << iip.quadratic_solver_library() << std::endl;
                break;
            }
        case time_to_intercept: {
                std::cout << "- Quadratic solver: " << tti.quadratic_solver_library() << std::endl;
                break;
            }
    }

    if (use_casadi)
        std::cout << "- Used casadi: " << use_casadi << std::endl;

    std::cout << "- SQP setup: max_sqp_iterations: " << sqp_config.max_sqp_iterations << " tol_primal: " << sqp_config.tol_pr << " tol_dual: " << sqp_config.tol_du << " min_dx: " << sqp_config.min_step_size << std::endl;
    if (optimizer_select == intercept_in_planes) {
        int n_planes, n_samples_intercepting, n_samples_breaking;
        std::tie(n_samples_intercepting, n_samples_breaking, n_planes) =  iip.scenario_setup();
        std::cout << "- Scenario setup: n_samples_intercepting: " << n_samples_intercepting;
        if (optimizer_select == intercept_in_planes)
            std::cout << " n_samples_breaking: " << n_samples_breaking;
        std::cout << std::endl;
        std::cout << "- N planes: " << n_planes << std::endl;
    }
    std::cout << "- System under stress: " << enable_stress << std::endl;
    std::cout << "- Disable timing ensurance: " << disable_time_ensurance << std::endl;
}

void OcpTester::cout_optmization_stats(optimizer_test optimizer_select, range_data data, range_stats stats) {
    std::cout << std::endl << "Timing:" << std::endl;
    std::cout << "- Average optimization time: " << stats.average_optimizing_time_us << "us" << std::endl;
    std::cout << "- Variance optimization time: " << stats.timing_variance_us << "us" << std::endl;
    std::cout << "- Max optimization time: " << stats.max_optimizing_time_us << "us" << std::endl;
    std::cout << "- Valid optimization timings: "  << stats.valid_optimizing_timings_percent << "% in total: " << data.n_optimizer_tests - data.invalid_timings << std::endl;
    std::cout << "- Realtime applicable in 2*sigma: " << stats.sigma2_optimizing_time_ms << " <? " << realtime_boundary_ms << ": " << (stats.sigma2_optimizing_time_ms < realtime_boundary_ms) << std::endl;
    std::cout << std::endl << "Optimiality:" << std::endl;
    std::cout << "- Interceptions: " << static_cast<double>(data.interceptions_found) / data.n_optimizer_tests * 100. << "% in total: " << data.interceptions_found << std::endl;
    std::cout << "- Average interception time: " << stats.average_interception_time_s << "s" << std::endl;
    std::cout << std::endl << "Stats:" << std::endl;
    std::cout << "- N_optimizations: " << data.n_optimizer_tests << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;

}

range_stats OcpTester::exec_range_test(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config) {
    bool enable_stress = false;
    bool disable_time_ensurance = false;
    std::vector<Plane> planes = cube_planes(3.f, 6);

    FlightArea flightarea;
    flightarea.init(planes);
    switch (optimizer_select) {
        case time_to_intercept: {
                tti.init(&thrust);
                // if (disable_time_ensurance)
                //     tti.max_cpu_time(0);
#ifdef OCP_DEV
                if (use_casadi) {
                    tti.init_casadi("../../ocp_design/casadi/tti/tti_optimizer.so");
                }
#endif

                break;
            }
        case intercept_in_planes: {

                iip.init(&thrust, &flightarea, bare);
                iip.sqp_setup(sqp_config);
                if (disable_time_ensurance)
                    iip.max_cpu_time(0);
                if (enable_stress)
                    system("stress --cpu $(getconf _NPROCESSORS_ONLN) &");

#ifdef OCP_DEV
                if (use_casadi)
                    iip.init_casadi("../../ocp_design/casadi/intercept_in_planes/intercept_in_planes_optimizer.so");
#endif
                break;
            }

    }
    tracking::TrackData drone;
    tracking::TrackData insect;
    std::chrono::_V2::system_clock::time_point t_start, t_end;
    range_data range_dat;

    float summon_box_size = 2;
    int n_samples_per_direction = 2;
    float step_size = summon_box_size / n_samples_per_direction;

    std::chrono::_V2::system_clock::time_point t_test_start = std::chrono::high_resolution_clock::now();
    for (int idx_drone_posx = 0; idx_drone_posx <= n_samples_per_direction; idx_drone_posx++) {
        for (int idx_drone_posy = 0; idx_drone_posy <= n_samples_per_direction; idx_drone_posy++) {
            for (int idx_drone_posz = 0; idx_drone_posz <= n_samples_per_direction; idx_drone_posz++) {
                for (int idx_drone_velx = 0; idx_drone_velx <= n_samples_per_direction; idx_drone_velx++) {
                    for (int idx_drone_vely = 0; idx_drone_vely <= n_samples_per_direction; idx_drone_vely++) {
                        for (int idx_drone_velz = 0; idx_drone_velz <= n_samples_per_direction; idx_drone_velz++) {

                            // for (int idx_insect_posx = 0; idx_insect_posx <= n_samples_per_direction; idx_insect_posx++) {
                            //     for (int idx_insect_posy = 0; idx_insect_posy <= n_samples_per_direction; idx_insect_posy++) {
                            //         for (int idx_insect_posz = 0; idx_insect_posz <= n_samples_per_direction; idx_insect_posz++) {
                            //             for (int idx_insect_velx = 0; idx_insect_velx <= n_samples_per_direction; idx_insect_velx++) {
                            //                 for (int idx_insect_vely = 0; idx_insect_vely <= n_samples_per_direction; idx_insect_vely++) {
                            //                     for (int idx_insect_velz = 0; idx_insect_velz <= n_samples_per_direction; idx_insect_velz++) {


                            drone.state.pos = cv::Point3f(-summon_box_size / 2 + idx_drone_posx * step_size,
                                                          -summon_box_size / 2 + idx_drone_posy * step_size,
                                                          -summon_box_size / 2 + idx_drone_posz * step_size);
                            drone.state.vel = cv::Point3f(-summon_box_size / 2 + idx_drone_velx * step_size,
                                                          -summon_box_size / 2 + idx_drone_vely * step_size,
                                                          -summon_box_size / 2 + idx_drone_velz * step_size);

                            // insect.state.pos = cv::Point3f(-summon_box_size / 2 + idx_insect_posx * step_size,
                            //                                -summon_box_size / 2 + idx_insect_posy * step_size,
                            //                                -summon_box_size / 2 + idx_insect_posz * step_size);
                            // insect.state.vel = cv::Point3f(-summon_box_size / 2 + idx_insect_velx * step_size,
                            //                                -summon_box_size / 2 + idx_insect_vely * step_size,
                            //                                -summon_box_size / 2 + idx_insect_velz * step_size);
                            insect.state.pos = cv::Point3f(0, 0, 0);
                            insect.state.vel = cv::Point3f(0, 0, 0);

                            bool valid;
                            float timetointercept;
                            cv::Point3f positiontointercept;
                            t_start = std::chrono::high_resolution_clock::now();
                            switch (optimizer_select) {
                                case time_to_intercept: {
                                        auto opti_res = tti.find_best_interception(drone, insect);
                                        valid = opti_res.valid;
                                        timetointercept = opti_res.time_to_intercept;
                                        positiontointercept = opti_res.position_to_intercept;
                                        break;
                                    }
                                case intercept_in_planes: {
                                        auto opti_res = iip.find_best_interception(drone, insect);
                                        valid = opti_res.valid;
                                        timetointercept = opti_res.time_to_intercept;
                                        positiontointercept = opti_res.position_to_intercept;
                                        break;
                                    }
                            }
                            t_end = std::chrono::high_resolution_clock::now();

                            if (valid) {
                                range_dat.interceptions_found++;
                                range_dat.accumulated_interception_time_s += static_cast<double>(timetointercept);
                            }
                            // else
                            //     std::cout << "invalid for drone: " << drone.pos() << ", " << drone.vel() << ", insect: " << insect.pos() << ", " << insect.vel() << std::endl;
                            std::string opti_name;
                            if (use_casadi)
                                opti_name = "casadi: ";
                            else
                                opti_name = "pats: ";

                            // std::cout << opti_name << drone.state.pos <<  drone.state.vel << insect.state.pos << insect.state.vel <<
                            //           " " << valid << " " << timetointercept << " " << positiontointercept << std::endl;

                            range_dat.n_optimizer_tests++;
                            double dt_us = (t_end - t_start) / 1us;
                            range_dat.timings_us.push_back(dt_us);
                            if (dt_us > realtime_boundary_ms * 1000) {
                                range_dat.invalid_timings++;
                            }
                            range_dat.accumulated_optimizing_time_us += dt_us;
                            if (dt_us > range_dat.max_optimizing_time_us)
                                range_dat.max_optimizing_time_us = dt_us;

                        }
                    }
                }
            }
        }
    }
// }
// }
// }
// }
// }
// }
    if (enable_stress)
        system("pkill -9 stress");
    std::chrono::_V2::system_clock::time_point t_test_end = std::chrono::high_resolution_clock::now();
    std::cout << "Testing time: " << std::chrono::duration_cast<std::chrono::seconds>(t_test_end - t_test_start).count() << "s" << std::endl;

    auto stats = eval_range_data(range_dat);

    cout_header(optimizer_select);
    cout_setup(optimizer_select, use_casadi, sqp_config, enable_stress, disable_time_ensurance);
    cout_optmization_stats(optimizer_select, range_dat, stats);

    return stats;
}


std::tuple<tracking::TrackData, tracking::TrackData> OcpTester::case_test() {

    tracking::TrackData drone;
    drone.state.pos = {0, 0, 0};
    drone.state.vel = {0, 2, 2};
    tracking::TrackData insect;
    insect.state.pos = {0, 1, 0};
    insect.state.vel = {2, 2, -2};

    return std::tuple(drone, insect);
}

std::tuple<tracking::TrackData, tracking::TrackData, std::vector<Plane>> OcpTester::application_test() {
    tracking::TrackData drone;
    tracking::TrackData insect;
    std::vector<Plane> _planes;

    thrust = 26.3803;
    drone.state.pos = {-0.0571426, -1.1537, -1.3503};
    drone.state.vel = {0, 0, 0};
    insect.state.pos = {0.905563, -0.7, -1.7};
    insect.state.vel = {-0.5, 0, 0};
    _planes.push_back(Plane(-0.0012757, -0.149755, 0.00847481, -0.00850464, -0.998366, 0.0564987, unspecified_plane));
    _planes.push_back(Plane(-0.125681, -0.0565701, -0.0591959, -0.837872, -0.377134, -0.39464, unspecified_plane));
    _planes.push_back(Plane(0.85, 0, 0, -1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(0, 0, -1.3503, 0, 0, -1, unspecified_plane));
    _planes.push_back(Plane(0.12546, -0.0582635, -0.0580094, 0.836401, -0.388423, -0.38673, unspecified_plane));
    _planes.push_back(Plane(0, 0, -1.95, 0, 0, 1, unspecified_plane));
    _planes.push_back(Plane(-0.85, 0, 0, 1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(0, -0.9037, 0, 0, 1, 0, unspecified_plane)); (Plane(0, -0.91133, 0, 0, 1, 0, unspecified_plane));
    return std::tuple(drone, insect, _planes);
}

void OcpTester::find_parameter(optimizer_test optimizer_select, bool use_casadi) {
    std::vector<double> tol_params = {1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9};
    std::vector<int> max_sqp_iter = {10, 20, 30, 40, 50, 60, 70, 80, 90};
    std::vector<sqp_solver_configuration> sqp_configurations;
    for (auto mi : max_sqp_iter) {
        for (auto prtol : tol_params) {
            for (auto dutol : tol_params) {
                for (auto dx : tol_params) {
                    sqp_configurations.push_back(sqp_solver_configuration(mi, prtol, dutol, dx));
                }
            }
        }
    }
    std::vector<range_stats> stats = {};
    for (auto sqp_conf : sqp_configurations) {
        auto statsi = exec_range_test(optimizer_select, use_casadi, sqp_conf);
        stats.push_back(statsi);
    }

    cout_configuration_results(sqp_configurations, stats);
}

void OcpTester::cout_configuration_results(std::vector<sqp_solver_configuration> sqp_conf, std::vector<range_stats> stats) {

    for (uint i = 0; i < sqp_conf.size() - 1; i++) {
        std::cout << i << ": [" << sqp_conf[i].max_sqp_iterations << ", " << sqp_conf[i].tol_pr << ", " << sqp_conf[i].tol_du << ", " << sqp_conf[i].min_step_size << ", ";
        std::cout << stats[i].average_optimizing_time_us << ", " << stats[i].max_optimizing_time_us << ", " << stats[i].sigma2_optimizing_time_ms << ", " <<
                  stats[i].average_interception_time_s << "]" << std::endl;
    }
}
