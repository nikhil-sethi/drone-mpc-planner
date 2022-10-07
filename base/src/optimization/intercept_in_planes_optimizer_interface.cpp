#include "intercept_in_planes_optimizer_interface.h"
#include "intercept_in_planes_index.h"
#include <chrono>
#ifdef OCP_DEV
#include <casadi/casadi.hpp>
#endif
#include <sstream>

std::string InterceptInPlanesOptimizerInterface::version() {
    return INTERCEPT_IN_PLANES_VERSION;
}


void InterceptInPlanesOptimizerInterface::init(float *thrust, FlightArea  *flightarea, safety_margin_types safety_margin) {
    _thrust = thrust;
    _flight_area = flightarea;
    _safety_margin = safety_margin;
    opti_var = Eigen::VectorXd(N_OPTVARS).setZero();
    prob_params = problem_parameters(N_OPTVARS, N_CONSTRAINTS, N_PLANES * N_PLANE_PARAMETERS);

    init_box_constraints();
    init_inequality_constraints();
    qpsolver.init();
    sqpsolver.init(&qpsolver);
    sqp_solver_configuration config(30, 1e-2, 1e-2, 1e-1);
    sqpsolver.setup(config);
}

void InterceptInPlanesOptimizerInterface::init_box_constraints() {
    prob_params.lbx.setZero();
    prob_params.ubx.setZero();

    prob_params.lbx[dt_intercepting] = 1e-06;
    prob_params.ubx[dt_intercepting] = inf;
    prob_params.lbx[dt_breaking] = 1e-06;
    prob_params.ubx[dt_breaking] = max_dt_breaking;

    for (int i = drone_states_intercepting_first; i <= drone_states_intercepting_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }

    for (int i = insect_states_first; i <= insect_states_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }

    for (int i = drone_virtual_inputs_intercepting_first; i <= drone_virtual_inputs_intercepting_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }


    for (int i = drone_states_breaking_first; i <= drone_states_breaking_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }

    for (int i = drone_virtual_inputs_breaking_first; i <= drone_virtual_inputs_breaking_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }

    uint drone_velx_breaking_final = drone_states_breaking_last - 2;
    uint drone_velz_breaking_final = drone_states_breaking_last;
    for (uint i = drone_velx_breaking_final; i <= drone_velz_breaking_final; i++) {
        prob_params.lbx[i] = 0;
        prob_params.ubx[i] = 0;
    }


    for (int i = drone_transition_slacks_first; i <= drone_transition_slacks_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }

    for (int i = interception_slacks_first; i <= interception_slacks_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] =  inf;
    }
}

void InterceptInPlanesOptimizerInterface::init_inequality_constraints() {
    prob_params.lbg.setZero();
    prob_params.ubg.setZero();

    for (uint p = plane_constraints_first; p <= plane_constraints_last; p++) {
        prob_params.ubg[p] = inf;
    }
}


void InterceptInPlanesOptimizerInterface::update_initial_guess(tracking::TrackData drone, tracking::TrackData insect) {
    prob_params.X0.setZero();

    // The better the initial guess the faster the optimization time and the more
    // certain the optimizer returns a good result:
    Eigen::Vector3d drone_pos0(static_cast<double>(drone.state.pos.x), static_cast<double>(drone.state.pos.y), static_cast<double>(drone.state.pos.z));
    Eigen::Vector3d drone_vel0(static_cast<double>(drone.state.vel.x), static_cast<double>(drone.state.vel.y), static_cast<double>(drone.state.vel.z));
    Eigen::Vector3d insect_pos0(static_cast<double>(insect.state.pos.x), static_cast<double>(insect.state.pos.y), static_cast<double>(insect.state.pos.z));
    Eigen::Vector3d insect_vel0(static_cast<double>(insect.state.vel.x), static_cast<double>(insect.state.vel.y), static_cast<double>(insect.state.vel.z));

    float thrust = optimization_thrust(*_thrust) / sqrtf(3.);

    float pos_err = (insect_pos0 - drone_pos0).norm();
    float vel_err = (insect_vel0 - drone_vel0).norm();
    float est_tti = vel_err / thrust + sqrt(2 * pos_err / (thrust)); // The size of THIS is crucial for the optimality of the result!!
    est_tti *= 2.f;

    Eigen::Vector3d insect_posEnd = insect_pos0 + est_tti * insect_vel0;
    // std::cout << "Guessed intersect pos: " << insect_posEnd.transpose() << std::endl;
    Eigen::Vector3d u_dir = (insect_pos0 - drone_pos0).normalized();
    Eigen::Vector3d drone_vel_intercepting = drone_vel0 + est_tti * (thrust) * u_dir;
    Eigen::VectorXd drone_x0 = Eigen::VectorXd(6);
    drone_x0 << drone_pos0, drone_vel0;
    Eigen::VectorXd drone_x_intercepting = Eigen::VectorXd(6);
    drone_x_intercepting << insect_posEnd, drone_vel_intercepting;
    // std::cout << "drone_x_intercepting: " << drone_x_intercepting.transpose() << std::endl;
    // std::cout << "u_dir: " << u_dir.transpose() << std::endl;

    prob_params.X0[dt_intercepting] = static_cast<double>(est_tti) / N_STEPS_INTERCEPTING;
    prob_params.X0.segment(drone_states_intercepting_first, N_DRONE_STATES) = drone_x0;
    for (uint k = 1; k < N_SAMPLES_INTERCEPTING; k++) {
        double tau = static_cast<double>(k) / N_STEPS_INTERCEPTING;

        Eigen::VectorXd drone_state_k = (1 - tau) * drone_x0 + tau * drone_x_intercepting;
        cv::Point3f checked_drone_state_k = _flight_area->move_inside(cv::Point3f(drone_state_k[0], drone_state_k[1], drone_state_k[2]), relaxed, drone.pos());
        prob_params.X0.segment(drone_states_intercepting_first + k * N_DRONE_STATES, N_DRONE_STATES) = drone_state_k;
        prob_params.X0.segment(drone_states_intercepting_first + k * N_DRONE_STATES, 3) = Eigen::Vector3d(checked_drone_state_k.x, checked_drone_state_k.y, checked_drone_state_k.z);
    }

    Eigen::VectorXd u0 = (thrust) * u_dir;
    for (uint k = 0; k < N_STEPS_INTERCEPTING; k++) {
        prob_params.X0.segment(drone_inputs_intercepting_first + k * N_DRONE_INPUTS, N_DRONE_INPUTS) = u0;
    }

    float dt_breaking_guess = static_cast<float>(drone_vel_intercepting.norm()) / thrust;
    prob_params.X0[dt_breaking] = std::min(static_cast<double>(dt_breaking_guess) / N_STEPS_BREAKING, max_dt_breaking);
    Eigen::Vector3d u_breaking = -u0;

    Eigen::VectorXd drone_xstoped(N_DRONE_STATES);
    drone_xstoped.setZero();
    drone_xstoped.segment(0, 3) = 0.5 * static_cast<double>(dt_breaking_guess * dt_breaking_guess) *  u_breaking + drone_x_intercepting.segment(0, 3);

    for (uint k = 0; k < N_SAMPLES_BREAKING; k++) {
        double tau = static_cast<double>(k) / N_STEPS_BREAKING;
        Eigen::VectorXd drone_state_k = (1. - tau) * drone_x_intercepting + tau * drone_xstoped;
        cv::Point3f checked_drone_state_k = _flight_area->move_inside(cv::Point3f(drone_state_k[0], drone_state_k[1], drone_state_k[2]), relaxed, drone.pos());
        prob_params.X0.segment(drone_states_breaking_first + k * N_DRONE_STATES, N_DRONE_STATES) = drone_state_k;
        prob_params.X0.segment(drone_states_breaking_first + k * N_DRONE_STATES, 3) = Eigen::Vector3d(checked_drone_state_k.x, checked_drone_state_k.y, checked_drone_state_k.z);
    }

    for (uint k = 0; k < N_STEPS_BREAKING; k++) {
        prob_params.X0.segment(drone_inputs_breaking_first + k * N_DRONE_INPUTS, N_DRONE_INPUTS) = u_breaking;
    }

    Eigen::VectorXd insect_x0(N_INSECT_STATES);
    insect_x0 << insect_pos0, insect_vel0;
    Eigen::VectorXd insect_xF(N_INSECT_STATES);
    insect_xF <<  insect_posEnd, insect_vel0;
    for (uint k = 0; k < N_SAMPLES_INTERCEPTING; k++) {
        double tau = static_cast<double>(k) / N_SAMPLES_INTERCEPTING;

        prob_params.X0.segment(insect_states_first + k * N_INSECT_STATES, N_INSECT_STATES) = (1. - tau) * insect_x0 + tau * insect_xF;
    }
}

void InterceptInPlanesOptimizerInterface::update_box_constraints(tracking::TrackData drone, tracking::TrackData insect) {
    prob_params.lbx[drone_states_intercepting_first + DRONE_POSX] = static_cast<double>(drone.pos().x);
    prob_params.lbx[drone_states_intercepting_first + DRONE_POSY] = static_cast<double>(drone.pos().y);
    prob_params.lbx[drone_states_intercepting_first + DRONE_POSZ] = static_cast<double>(drone.pos().z);
    prob_params.lbx[drone_states_intercepting_first + DRONE_VELX] = static_cast<double>(drone.vel().x);
    prob_params.lbx[drone_states_intercepting_first + DRONE_VELY] = static_cast<double>(drone.vel().y);
    prob_params.lbx[drone_states_intercepting_first + DRONE_VELZ] = static_cast<double>(drone.vel().z);
    prob_params.ubx[drone_states_intercepting_first + DRONE_POSX] = static_cast<double>(drone.pos().x);
    prob_params.ubx[drone_states_intercepting_first + DRONE_POSY] = static_cast<double>(drone.pos().y);
    prob_params.ubx[drone_states_intercepting_first + DRONE_POSZ] = static_cast<double>(drone.pos().z);
    prob_params.ubx[drone_states_intercepting_first + DRONE_VELX] = static_cast<double>(drone.vel().x);
    prob_params.ubx[drone_states_intercepting_first + DRONE_VELY] = static_cast<double>(drone.vel().y);
    prob_params.ubx[drone_states_intercepting_first + DRONE_VELZ] = static_cast<double>(drone.vel().z);

    prob_params.lbx[insect_states_first + INSECT_POSX] = static_cast<double>(insect.pos().x);
    prob_params.lbx[insect_states_first + INSECT_POSY] = static_cast<double>(insect.pos().y);
    prob_params.lbx[insect_states_first + INSECT_POSZ] = static_cast<double>(insect.pos().z);
    prob_params.lbx[insect_states_first + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.lbx[insect_states_first + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.lbx[insect_states_first + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    prob_params.ubx[insect_states_first + INSECT_POSX] = static_cast<double>(insect.pos().x);
    prob_params.ubx[insect_states_first + INSECT_POSY] = static_cast<double>(insect.pos().y);
    prob_params.ubx[insect_states_first + INSECT_POSZ] = static_cast<double>(insect.pos().z);
    prob_params.ubx[insect_states_first + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.ubx[insect_states_first + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.ubx[insect_states_first + INSECT_VELZ] = static_cast<double>(insect.vel().z);

    double insect_vel[3] = {static_cast<double>(insect.vel().x), static_cast<double>(insect.vel().y), static_cast<double>(insect.vel().z)};
    for (uint k = 0; k < N_SAMPLES_INTERCEPTING; k++) {
        for (uint uk = 0; uk < 3; uk++) {
            prob_params.lbx[insect_states_first + k * N_INSECT_STATES + 3 + uk] = insect_vel[uk];
            prob_params.ubx[insect_states_first + k * N_INSECT_STATES + 3 + uk] = insect_vel[uk];
        }
    }

    double thrust =  static_cast<double>(optimization_thrust(*_thrust)) / sqrt(3);
    if (prob_params.ubx[drone_inputs_intercepting_first] != thrust) {
        for (uint k = 0; k < N_STEPS_INTERCEPTING; k++) {
            for (uint uk = 0; uk < N_DRONE_INPUTS; uk++) {
                prob_params.lbx[drone_inputs_intercepting_first + k * N_DRONE_INPUTS + uk] = -thrust;
                prob_params.ubx[drone_inputs_intercepting_first + k * N_DRONE_INPUTS + uk] =  thrust;
            }
        }
    }

    if (prob_params.ubx[drone_inputs_breaking_first] != thrust) {
        for (uint k = 0; k < N_STEPS_BREAKING; k++) {
            for (uint uk = 0; uk < N_DRONE_INPUTS; uk++) {
                prob_params.lbx[drone_inputs_breaking_first + k * N_DRONE_INPUTS + uk] = -thrust;
                prob_params.ubx[drone_inputs_breaking_first + k * N_DRONE_INPUTS + uk] =  thrust;
            }
        }
    }
}

void InterceptInPlanesOptimizerInterface::update_plane_parameters(std::vector<Plane> planes) {
    prob_params.param.setZero();
    for (uint p = 0; p < planes.size(); p++) {
        prob_params.param[p * N_PLANE_PARAMETERS + 0] = static_cast<double>(planes.at(p).support.x);
        prob_params.param[p * N_PLANE_PARAMETERS + 1] = static_cast<double>(planes.at(p).support.y);
        prob_params.param[p * N_PLANE_PARAMETERS + 2] = static_cast<double>(planes.at(p).support.z);
        prob_params.param[p * N_PLANE_PARAMETERS + 3] = static_cast<double>(planes.at(p).normal.x);
        prob_params.param[p * N_PLANE_PARAMETERS + 4] = static_cast<double>(planes.at(p).normal.y);
        prob_params.param[p * N_PLANE_PARAMETERS + 5] = static_cast<double>(planes.at(p).normal.z);
    }
    // for (uint p = planes.size(); p < N_PLANES; p++) {
    //     prob_params.param[p * N_PLANE_PARAMETERS + 0] = 0;
    //     prob_params.param[p * N_PLANE_PARAMETERS + 1] = 0;
    //     prob_params.param[p * N_PLANE_PARAMETERS + 2] = 0;
    //     prob_params.param[p * N_PLANE_PARAMETERS + 3] = 0;
    //     prob_params.param[p * N_PLANE_PARAMETERS + 4] = 0;
    //     prob_params.param[p * N_PLANE_PARAMETERS + 5] = 0;
    // }
}

bool InterceptInPlanesOptimizerInterface::feasible_trajectory(Eigen::VectorXd xopt) {
    // qpsolver.print_eigenvector("Xopt", xopt);
    // std::cout << "dt_intercepting: " << xopt[dt_intercepting] << ", dt_breaking: " << xopt[dt_breaking] << std::endl;

    if (xopt.norm() < 0.1) {
        // print_warning("INFEASIBLE: Xopt=0!");
        return false;
    }

    double virt_input_sum = 0;
    for (uint k = 0; k < N_STEPS_INTERCEPTING; k++) {
        for (uint suk = 0; suk < N_DRONE_INPUTS; suk++) {
            virt_input_sum += abs(xopt[drone_virtual_inputs_intercepting_first + k * N_DRONE_INPUTS + suk]);
        }
    }
    for (uint k = 0; k < N_STEPS_BREAKING; k++) {
        for (uint suk = 0; suk < N_DRONE_INPUTS; suk++) {
            virt_input_sum += abs(xopt[drone_virtual_inputs_breaking_first + k * N_DRONE_INPUTS + suk]);
        }
    }
    if (virt_input_sum > 0.001) {
        // print_warning("INFEASIBLE: virt_input too high! Sum virtual inputs: " + to_string(virt_input_sum));
        return false;
    }

    if (xopt[dt_intercepting] < 0 || xopt[dt_breaking] < 0) {
        // print_warning("INFEASIBLE: violated time constraint!");
        return false;
    }

    for (uint k = 1; k < N_SAMPLES_INTERCEPTING; k++) {
        if (abs(xopt[drone_states_intercepting_first + k * N_DRONE_STATES]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
        if (abs(xopt[drone_states_intercepting_first + k * N_DRONE_STATES + 1]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
        if (abs(xopt[drone_states_intercepting_first + k * N_DRONE_STATES + 2]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
    }

    for (uint k = 1; k < N_SAMPLES_BREAKING; k++) {
        if (abs(xopt[drone_states_breaking_first + k * N_DRONE_STATES]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
        if (abs(xopt[drone_states_breaking_first + k * N_DRONE_STATES + 1]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
        if (abs(xopt[drone_states_breaking_first + k * N_DRONE_STATES + 2]) > 100) {
            // std::cout << "INFEASIBLE: Single position component is too high" << std::endl;
            return false;
        }
    }

    Eigen::Vector3d interception_error = Eigen::Vector3d().setZero();
    for (uint i = interception_slacks_first; i < interception_slacks_last; i++) {
        interception_error[i - interception_slacks_first] = xopt[i];
    }
    double interception_error_abs = interception_error.norm();
    if (interception_error_abs > 0.02) {
        // print_warning("SUBOPTMIAL SOLUTION: Drone not intercepting insect!");
        return false;
    }

    double transition_error = 0;
    for (uint i = drone_transition_slacks_first; i < drone_transition_slacks_last; i++) {
        transition_error += abs(xopt[i]);
    }
    if (transition_error > 0.01) {
        // print_warning("INFEASIBLE: Transition condition invalid!");
        return false;
    }

    return true;
}


intercept_in_planes_result InterceptInPlanesOptimizerInterface::find_best_interception(tracking::TrackData drone, tracking::TrackData insect) {
#ifdef PATS_OCP_PROFILING
    std::chrono::_V2::system_clock::time_point t_start = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point t_now;
#endif
    intercept_in_planes_result res;

    if (norm(drone.pos() - insect.pos()) < 0.01) {
        // print_warning("Warning: Return before optimizing: Drone and insect at same location!");
        return res;
    }

    std::vector<Plane> planes = _flight_area->active_planes(_safety_margin);
    if (planes.size() > N_PLANES) {
        std::stringstream ss;
        ss << "Error: Return before optimizing: Too many planes! Expect " << N_PLANES << " got " << planes.size();
        print_error(ss.str());
        return res;
    }

    update_initial_guess(drone, insect);
    update_box_constraints(drone, insect);
    update_plane_parameters(planes);

#ifdef PATS_OCP_PROFILING
    t_now = std::chrono::high_resolution_clock::now();
    double cpu_time_passed = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start).count();
    std::cout << "IIP: cpu_time (build problem): " << cpu_time_passed << "us" << std::endl;
    t_start = std::chrono::high_resolution_clock::now();
#endif


#ifdef OCP_DEV
    if (use_casadi)
        opti_var = sqpsolver.solve_casadi(&prob_params);
    else
#endif
        opti_var = sqpsolver.solve_line_search(&prob_params);

#ifdef PATS_OCP_PROFILING
    t_now = std::chrono::high_resolution_clock::now();
    cpu_time_passed = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start).count();
    std::cout << "IIP: cpu_time (run sqpmethod): " << cpu_time_passed << "us" << std::endl;
    t_start = std::chrono::high_resolution_clock::now();
#endif

    int drone_posxF_intercepting = drone_states_intercepting_first + N_STEPS_INTERCEPTING * N_DRONE_STATES + DRONE_POSX;
    int drone_posyF_intercepting = drone_states_intercepting_first + N_STEPS_INTERCEPTING * N_DRONE_STATES + DRONE_POSY;
    int drone_poszF_intercepting = drone_states_intercepting_first + N_STEPS_INTERCEPTING * N_DRONE_STATES + DRONE_POSZ;
    int drone_posxF_breaking = drone_states_breaking_first + N_STEPS_BREAKING * N_DRONE_STATES + DRONE_POSX;
    int drone_posyF_breaking = drone_states_breaking_first + N_STEPS_BREAKING * N_DRONE_STATES + DRONE_POSY;
    int drone_poszF_breaking = drone_states_breaking_first + N_STEPS_BREAKING * N_DRONE_STATES + DRONE_POSZ;
    res.time_to_intercept = opti_var[dt_intercepting] * N_STEPS_INTERCEPTING;
    res.position_to_intercept = cv::Point3f(opti_var[drone_posxF_intercepting], opti_var[drone_posyF_intercepting], opti_var[drone_poszF_intercepting]);
    res.acceleration_to_intercept = cv::Point3f(opti_var[drone_inputs_intercepting_first + DRONE_ACCX], opti_var[drone_inputs_intercepting_first + DRONE_ACCY], opti_var[drone_inputs_intercepting_first + DRONE_ACCZ]);
    res.position_stopped = cv::Point3f(opti_var[drone_posxF_breaking], opti_var[drone_posyF_breaking], opti_var[drone_poszF_breaking]);

    // qpsolver.print_eigenvector("Xopt", opti_var);
    opti_res_valid = false;
    if (feasible_trajectory(opti_var)) {
        opti_res_valid = true;
        res.valid = true;
    }
    // else
    //     export_scenario(drone, insect, planes);
    // report_scenario_result(drone, insect, res);

#ifdef PATS_OCP_PROFILING
    t_now = std::chrono::high_resolution_clock::now();
    cpu_time_passed = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start).count();
    std::cout << "IIP: cpu_time (evaluate result): " << cpu_time_passed << "us" << std::endl;
#endif
    return res;
}

void InterceptInPlanesOptimizerInterface::export_scenario(tracking::TrackData drone, tracking::TrackData insect, std::vector<Plane> planes) {
    std::cout << "    thrust = " << *_thrust << "; " << std::endl;
    std::cout << "    drone.state.pos = {" << drone.state.pos.x << ", " << drone.state.pos.y << ", " << drone.state.pos.z << "};" << std::endl;
    std::cout << "    drone.state.vel = {" << drone.state.vel.x << ", " << drone.state.vel.y << ", " << drone.state.vel.z << "};" << std::endl;
    std::cout << "    insect.state.pos = {" << insect.state.pos.x << ", " << insect.state.pos.y << ", " << insect.state.pos.z << "};" << std::endl;
    std::cout << "    insect.state.vel = {" << insect.state.vel.x << ", " << insect.state.vel.y << ", " << insect.state.vel.z << "};" << std::endl;

    for (auto &plane : planes) {
        std::cout << "    _planes.push_back(Plane("
                  << plane.support.x << ", " << plane.support.y << ", " << plane.support.z << ", "
                  << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << ", unspecified_plane));" << std::endl;
    }
}

void InterceptInPlanesOptimizerInterface::report_scenario_result(tracking::TrackData drone, tracking::TrackData insect, intercept_in_planes_result res) {

    if (res.time_to_intercept < 0.01 || !res.valid) {
        std::cout << COLORED_RED;
    } else {
        std::cout << COLORED_GREEN;
    }
    std::cout << "drone: " << drone.pos() << drone.vel();
    std::cout << ", insect: " << insect.pos() << insect.vel();
    std::cout << " intercept at " << res.position_to_intercept << " in " << res.time_to_intercept << "s";
    std::cout << " is feasible: " << res.valid;
    std::cout << COLORED_RESET << std::endl;
}

std::vector<cv::Point3f> InterceptInPlanesOptimizerInterface::interception_trajectory() {
    // qpsolver.print_eigenvector("Xopt", opti_var);
    std::vector<cv::Point3f> intercept_trajectory = {};

    // if (feasible_trajectory(opti_var)) {
    if (opti_res_valid) {
        for (uint k = 0; k < N_SAMPLES_INTERCEPTING; k++) {
            Eigen::VectorXd state_k = opti_var.segment(drone_states_intercepting_first + k * N_DRONE_STATES, 3);
            intercept_trajectory.push_back(cv::Point3f(state_k[0], state_k[1], state_k[2]));
        }

        for (uint k = 1; k < N_SAMPLES_BREAKING; k++) {
            Eigen::VectorXd state_k = opti_var.segment(drone_states_breaking_first + k * N_DRONE_STATES, 3);
            intercept_trajectory.push_back(cv::Point3f(state_k[0], state_k[1], state_k[2]));
        }
    }
    // qpsolver.print_eigenvector("Opti var", opti_var);

    return intercept_trajectory;
}

std::tuple<int, int, int> InterceptInPlanesOptimizerInterface::scenario_setup() {
    return std::tuple(N_SAMPLES_INTERCEPTING, N_SAMPLES_BREAKING, N_PLANES);
};
