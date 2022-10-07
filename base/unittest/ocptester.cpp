#include "ocptester.h"
#include <limits>
#include <stdlib.h>
#ifdef OPTI_ROSVIS
#include "rosvisualizerinterface.h"
#endif


bool OcpTester::is_tti_good(float optimizer_tti, float estimated_tti) {
    if (!((optimizer_tti - estimated_tti - accpt_tti_error) <= 0))
        std::cout << "est_tti: " << estimated_tti << ", opti_tti: " << optimizer_tti << std::endl;
    return ((optimizer_tti - estimated_tti - accpt_tti_error) <= 0);
}

bool OcpTester::is_optimizer_results_good(tracking::TrackData drone, tracking::TrackData insect, bool res_valid, float res_tti) {
    if (normf(drone.pos() - insect.pos()) < eps) {
        return false;
    }
    // Worst case estimation
    float pos_err = norm(drone.pos() - insect.pos());
    float vel_err = norm(drone.vel() - insect.vel());
    float est_tti = vel_err / thrust + sqrt(2 * pos_err / thrust);

    return res_valid && is_tti_good(res_tti, est_tti);
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
    stats.max_optimizing_time_us = data.max_optimizing_time_us;
    stats.timing_variance_us = sqrt(1. / (data.timings_us.size() - 1) * var_calculation_time_us);
    stats.sigma2_optimizing_time_ms = (stats.average_optimizing_time_us + 2 * stats.timing_variance_us) / 1000;
    stats.valid_optimizing_timings_percent = static_cast<double>(data.n_optimizer_tests - data.invalid_timings) / static_cast<double>(data.n_optimizer_tests) * 100.;
    stats.average_interception_time_s = data.accumulated_interception_time_s / data.interceptions_found;
    stats.invalid_optimization_results = data.invalid_optimization_results;
    stats.valid_optimization_results = static_cast<double>(data.interceptions_found) / data.n_optimizer_tests * 100.;

    return stats;
}

void OcpTester::cout_header(optimizer_test optimizer_select) {
    if (optimizer_select == time_to_intercept)
        std::cout << "---------------------(TTIv" + tti.version() + ") time to intercept-------------------------------" << std::endl;
    else if (optimizer_select == intercept_in_planes)
        std::cout << "---------------------(IIPv" + iip.version() + ") intercept_in_planes-----------------------------" << std::endl;

}

void OcpTester::cout_setup(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config) {
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

    std::cout << "- SQP setup: " << sqp_config << std::endl;
    std::cout << "- QP setup: " << range_test_config.qp_settings << std::endl;
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
    std::cout << "- Timing ensurance enabled: " << !disable_time_ensurance << std::endl;
#ifdef ROSVIS
    std::cout << "- Visualization enabled: " << 1 << std::endl;
#endif
}

void OcpTester::cout_optmization_stats(range_data data, range_stats stats) {
    std::cout << std::endl << "Timing:" << std::endl;
    std::cout << "- Average optimization time: " << stats.average_optimizing_time_us << "us" << std::endl;
    std::cout << "- Variance optimization time: " << stats.timing_variance_us << "us" << std::endl;
    std::cout << "- Max optimization time: " << stats.max_optimizing_time_us << "us" << std::endl;
    std::cout << "- Valid optimization timings: "  << stats.valid_optimizing_timings_percent << "% in total: " << data.n_optimizer_tests - data.invalid_timings << std::endl;
    std::cout << "- Realtime applicable in 2*sigma: " << stats.sigma2_optimizing_time_ms << " <? " << realtime_boundary_ms << ": " << (stats.sigma2_optimizing_time_ms < realtime_boundary_ms) << std::endl;
    std::cout << std::endl << "Optimiality:" << std::endl;
    std::cout << "- Interceptions: " << stats.valid_optimization_results << "% in total: " << data.interceptions_found << std::endl;
    std::cout << "- Average interception time: " << stats.average_interception_time_s << "s" << std::endl;
    std::cout << std::endl << "Stats:" << std::endl;
    std::cout << "- N_optimizations: " << data.n_optimizer_tests << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;

}

void OcpTester:: init_range_test(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config, QPSettings *qp_settings) {
    enable_stress = false;
    disable_time_ensurance = true;
    float cube_size = 2.f;
    std::vector<Plane> planes = cube_planes(cube_size, 6);
    range_test_config = range_test_configuration(optimizer_select, use_casadi, sqp_config, *qp_settings);

    flightarea.init(planes);
    flightarea.update_bottom_plane_based_on_blink(-cube_size / 2); //For update flight area config
    switch (optimizer_select) {
        case time_to_intercept: {
                // tti = TTIOptimizerInterface();
                tti.init(&thrust);
                tti.sqp_setup(sqp_config);
                tti.qp_setup(*qp_settings);
                if (disable_time_ensurance)
                    tti.max_cpu_time(0);
                if (disable_time_ensurance)
                    tti.max_cpu_time(0);
#ifdef OCP_DEV
                if (use_casadi) {
                    tti.init_casadi("../../ocp_design/tti/tti_optimizer.so");
                }
#endif

                break;
            }
        case intercept_in_planes: {
                // iip = InterceptInPlanesOptimizerInterface();
                iip.init(&thrust, &flightarea, bare);
                iip.sqp_setup(sqp_config);
                // iip.qp_setup(*qp_settings);
                if (disable_time_ensurance)
                    iip.max_cpu_time(0);
                if (enable_stress)
                    system("stress --cpu $(getconf _NPROCESSORS_ONLN) &");

#ifdef OCP_DEV
                if (use_casadi)
                    iip.init_casadi("../../ocp_design/intercept_in_planes/intercept_in_planes_optimizer.so");
#endif
                break;
            }

    }

}

range_stats OcpTester::exec_range_test() {
#ifdef OPTI_ROSVIS
    RosVisualizerInterface ros_interface;
    ros_interface.init();
    iip.ros_interface(&ros_interface);
    tti.ros_interface(&ros_interface);
#endif
    tracking::TrackData drone;
    drone.pos_valid = true;
    drone.vel_valid = true;
    tracking::TrackData insect;
    insect.pos_valid = true;
    insect.vel_valid = true;
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
                            switch (range_test_config.optimizer_select) {
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
                            } else {
                                range_dat.invalid_optimization_results++;
                                //     std::cout << "invalid for drone: " << drone.pos() << ", " << drone.vel() << ", insect: " << insect.pos() << ", " << insect.vel() << std::endl;
                            }

                            std::string opti_name;
                            if (range_test_config.use_casadi)
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
    cout_header(range_test_config.optimizer_select);
    cout_setup(range_test_config.optimizer_select, range_test_config.use_casadi, range_test_config.sqp_config);
    cout_optmization_stats(range_dat, stats);

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

void OcpTester::find_parameter(optimizer_test optimizer_select) {

    auto sqp_configs = generate_sqp_configs();
    auto qp_configs = generate_qp_configs();

    std::vector<range_stats> stats = {};
    std::vector<QPSettings> qp_config_combination = {};
    std::vector<sqp_solver_configuration> sqp_config_combination = {};

    for (auto sqp_conf : sqp_configs) {
        for (auto qp_conf : qp_configs) {
            init_range_test(optimizer_select, false, sqp_conf, &qp_conf);
            auto statsi = exec_range_test();
            qp_config_combination.push_back(qp_conf);
            sqp_config_combination.push_back(sqp_conf);
            stats.push_back(statsi);
        }
    }
    cout_configuration_results(qp_config_combination, sqp_config_combination, stats);
}

std::vector<sqp_solver_configuration> OcpTester::generate_sqp_configs() {
    std::vector<double> tol_params = {  1e-5, };//{1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9};
    std::vector<int> max_sqp_iter = { 50};
    std::vector<sqp_solver_configuration> sqp_configs;
    for (auto mi : max_sqp_iter) {
        for (auto prtol : tol_params) {
            for (auto dutol : tol_params) {
                for (auto dx : tol_params) {
                    sqp_configs.push_back(sqp_solver_configuration(mi, prtol, dutol, dx));
                }
            }
        }
    }
    return sqp_configs;
}

#ifdef USE_OSQP
std::vector<QPSettings> OcpTester::generate_qp_configs() {
    std::vector<float> rhos = { 1e-1,}; // {1e-3, 1e-2, 5e-2, 1e-1, 2e-1, 5e-1, 1e0, 2e0, 5e0, 1e1};
    std::vector<float> sigmas = {1e-7, 1e-6, 1e-5};
    std::vector<uint> max_iters = { 4000};
    std::vector<float> eps_abss = {1e-2, 1e-4, 1e-5};
    std::vector<float> eps_rels = {1e-2, 1e-4, 1e-5};
    std::vector<float> eps_prim_infs = { 1e-3, 1e-4, 1e-5};
    std::vector<float> eps_dual_infs = { 1e-3, 1e-4, 1e-5};
    std::vector<float> alphas = {0.5, 1, 1.5};
    std::vector<float> deltas = {1e-7, 1e-6, 1e-5};
    std::vector<bool> polishs = {false, true};
    std::vector<uint> polish_refine_iters = {1, 3, 5};
    std::vector<bool> verboses = {false};
    std::vector<bool> scaled_terminations = {false, true};
    std::vector<int> check_terminations = {10, 25,};
    std::vector<bool> warm_starts = {false, true};
    std::vector<uint> scalings = {2, 5, 10};
    std::vector<bool> adaptive_rhos = {false, true};
    std::vector<float> adaptive_rho_intervals = { 2, 5};
    std::vector<float> adaptive_rho_tolerances = {2, 5, 10};
    std::vector<float> adaptive_rho_fractions = { 0.2, 0.4, 0.6,};
    std::vector<float> time_limits = {0};

    std::vector<QPSettings> qp_configs;
    std::cout << "max combi: " << qp_configs.max_size() << " vs actual combinations: " << rhos.size() * sigmas.size() * max_iters.size() * eps_abss.size() * eps_rels.size()
              * eps_prim_infs.size() * eps_dual_infs.size() * alphas.size() * deltas.size() * polishs.size() * polish_refine_iters.size() * verboses.size() * scaled_terminations.size()
              * check_terminations.size() * warm_starts.size() * scalings.size() * adaptive_rhos.size() * adaptive_rho_intervals.size() * adaptive_rho_tolerances.size()
              * adaptive_rho_fractions.size() * time_limits.size() << std::endl;

    for (auto r : rhos) {
        for (auto s : sigmas) {
            for (auto mi : max_iters) {
                for (auto ea : eps_abss) {
                    for (auto er : eps_rels) {
                        for (auto epi : eps_prim_infs) {
                            for (auto  edi : eps_dual_infs) {
                                for (auto a : alphas) {
                                    for (auto d : deltas) {
                                        for (auto p : polishs) {
                                            for (auto pri : polish_refine_iters) {
                                                for (auto v : verboses) {
                                                    for (auto st : scaled_terminations) {
                                                        for (auto ct : check_terminations) {
                                                            for (auto ws : warm_starts) {
                                                                for (auto sc : scalings) {
                                                                    bool adapative_rho_deactivated_saved = false;
                                                                    for (auto ar : adaptive_rhos) {
                                                                        for (auto ari : adaptive_rho_intervals) {
                                                                            for (auto art : adaptive_rho_tolerances) {
                                                                                for (auto arf : adaptive_rho_fractions) {
                                                                                    for (auto tl : time_limits) {
                                                                                        QPSettings qpsettings;
                                                                                        qpsettings.rho(r);
                                                                                        qpsettings.sigma(s);
                                                                                        qpsettings.max_iter(mi);
                                                                                        qpsettings.eps_abs(ea);
                                                                                        qpsettings.eps_rel(er);
                                                                                        qpsettings.eps_prim_inf(epi);
                                                                                        qpsettings.eps_dual_inf(edi);
                                                                                        qpsettings.alpha(a);
                                                                                        qpsettings.delta(d);
                                                                                        qpsettings.polish(p);
                                                                                        qpsettings.polish_refine_iter(pri);
                                                                                        qpsettings.verbose(v);
                                                                                        qpsettings.scaled_termination(st);
                                                                                        qpsettings.check_termination(ct);
                                                                                        qpsettings.warm_start(ws);
                                                                                        qpsettings.scaling(sc);
                                                                                        qpsettings.adaptive_rho(ar);
                                                                                        qpsettings.adaptive_rho_interval(ari);
                                                                                        qpsettings.adaptive_rho_tolerance(art);
                                                                                        qpsettings.adaptive_rho_fraction(arf);
                                                                                        qpsettings.time_limit(tl);
                                                                                        if (!ar && !adapative_rho_deactivated_saved) {
                                                                                            adapative_rho_deactivated_saved = true;
                                                                                            qp_configs.push_back(qpsettings);
                                                                                        } else if (ar)
                                                                                            qp_configs.push_back(qpsettings);
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return qp_configs;
}
#else
std::vector<QPSettings> OcpTester::generate_qp_configs() {
    std::vector<QPSettings> qp_configs;
    qp_configs.push_back(QPSettings());

    return qp_configs;
}

#endif

void OcpTester::cout_configuration_results(std::vector<QPSettings> qp_confs, std::vector<sqp_solver_configuration> sqp_conf, std::vector<range_stats> stats) {
    uint i_best_average_interception_time = 0;
    double best_average_interception_time = std::numeric_limits<double>::infinity();
    uint i_best_sigma2_optimization_time = 0;
    double best_sigma2_optimization_time = std::numeric_limits<double>::infinity();
    uint i_best_valid_results = 0;
    float best_valid_results = 0;
    for (uint i = 0; i < sqp_conf.size() - 1; i++) {
        if (stats.at(i).sigma2_optimizing_time_ms < best_sigma2_optimization_time) {
            best_sigma2_optimization_time = stats.at(i).sigma2_optimizing_time_ms;
            i_best_sigma2_optimization_time = i;
        }
        if (stats.at(i).average_interception_time_s < best_average_interception_time) {
            best_average_interception_time = stats.at(i).average_interception_time_s;
            i_best_average_interception_time = i;
        }
        if (stats.at(i).valid_optimization_results > best_valid_results) {
            best_valid_results = stats.at(i).valid_optimization_results;
            i_best_valid_results = i;
        }
    }

    std::cout << "Best config for valid results:" << std::endl;
    uint i = i_best_valid_results;
    std::cout << i << ": [" << stats[i] << ", " << sqp_conf[i] << ", " << qp_confs[i] << "]" << std::endl;

    std::cout << "Best config for interception time:" << std::endl;
    i = i_best_average_interception_time;
    std::cout << i << ": [" << stats[i] << ", " << sqp_conf[i] << ", " << qp_confs[i] << "]" << std::endl;

    std::cout << "Best config for sigma2 optimization time:" << std::endl;
    i = i_best_sigma2_optimization_time;
    std::cout << i << ": [" << stats[i] << ", " << sqp_conf[i] << ", " << qp_confs[i] << "]" << std::endl;
}

void OcpTester::check_range_optimality(optimizer_test optimizer_select, range_stats range_stats) {
    double previous_intercept_time = 0;
    if (optimizer_select == time_to_intercept)
        previous_intercept_time = 0.537538;
    else if (optimizer_select == intercept_in_planes)
        previous_intercept_time = 0.509424;

    if (abs(range_stats.average_interception_time_s - previous_intercept_time) > 0.01)
        std::cout << "\033[33m" << "Optimality significantly changed!" << "\033[0m" <<  std::endl;
    else
        std::cout << "\033[92m" << "Optimality as expected" << "\033[0m" <<  std::endl;

}

std::ostream &operator<<(std::ostream &os, range_stats &rng_stats) {
    os << "valid results[%]: " << rng_stats.valid_optimization_results
       << ", sigma2_optimization_time[ms]: " << rng_stats.sigma2_optimizing_time_ms
       << ", average_interception_time[s]: " << rng_stats.average_interception_time_s;
    return os;
}
