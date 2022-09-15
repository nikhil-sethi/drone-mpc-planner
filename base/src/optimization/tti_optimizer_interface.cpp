#include "tti_optimizer_interface.h"
#include <chrono>

enum op_idx {
    dt = 0,
    drone_posx_1,
    drone_posy_1,
    drone_posz_1,
    drone_velx_1,
    drone_vely_1,
    drone_velz_1,
    drone_posx_2,
    drone_posy_2,
    drone_posz_2,
    drone_velx_2,
    drone_vely_2,
    drone_velz_2,
    drone_accx,
    drone_accy,
    drone_accz,
    drone_virt_posx,
    drone_virt_posy,
    drone_virt_posz,
    drone_virt_velx,
    drone_virt_vely,
    drone_virt_velz,
    insect_posx_1,
    insect_posy_1,
    insect_posz_1,
    insect_velx_1,
    insect_vely_1,
    insect_velz_1,
    insect_posx_2,
    insect_posy_2,
    insect_posz_2,
    insect_velx_2,
    insect_vely_2,
    insect_velz_2,
};

void TTIOptimizerInterface::init(float *thrust) {
    _thrust = thrust;
    qpsolver.init();
    sqpsolver.init(&qpsolver);
    sqp_solver_configuration config(30, 1e-2, 1e-2, 1e-1);
    sqpsolver.setup(config);
    prob_params = problem_parameters(34, 15, 0);
    init_const_variable_bounds();
}

void TTIOptimizerInterface::qp_setup(QPSettings qpsettings) {
    qpsolver.qp_setup(qpsettings);
}


void TTIOptimizerInterface::init_const_variable_bounds() {
    prob_params.lbx.setZero();
    prob_params.ubx.setZero();
    prob_params.lbx[dt] = 1e-06; //, 0.0616, -inf, -1.28, -inf, -1.322, -inf, 0, -inf, 0, -inf, 0, -inf, -20, -20, -20, -inf, -inf, -inf, -inf, -inf, -inf, 0.99, -inf, -0.4, -inf, -1.899, -inf, -0.5, -0.5, 0, 0, 0, 0};
    prob_params.ubx[dt] = inf; //, 0.0616, inf, -1.28, inf, -1.322, inf, 0, inf, 0, inf, 0, inf, 20, 20, 20, inf, inf, inf, inf, inf, inf, 0.99, inf, -0.4, inf, -1.899, inf, -0.5, -0.5, 0, 0, 0, 0};

    prob_params.lbx[drone_posx_2] = -inf;
    prob_params.lbx[drone_posy_2] = -inf;
    prob_params.lbx[drone_posz_2] = -inf;
    prob_params.lbx[drone_velx_2] = -inf;
    prob_params.lbx[drone_vely_2] = -inf;
    prob_params.lbx[drone_velz_2] = -inf;

    prob_params.ubx[drone_posx_2] = inf;
    prob_params.ubx[drone_posy_2] = inf;
    prob_params.ubx[drone_posz_2] = inf;
    prob_params.ubx[drone_velx_2] = inf;
    prob_params.ubx[drone_vely_2] = inf;
    prob_params.ubx[drone_velz_2] = inf;

    prob_params.lbx[insect_posx_2] = -inf;
    prob_params.lbx[insect_posy_2] = -inf;
    prob_params.lbx[insect_posz_2] = -inf;

    prob_params.ubx[insect_posx_2] = inf;
    prob_params.ubx[insect_posy_2] = inf;
    prob_params.ubx[insect_posz_2] = inf;

    prob_params.lbx[drone_virt_posx] = -inf;
    prob_params.lbx[drone_virt_posy] = -inf;
    prob_params.lbx[drone_virt_posz] = -inf;
    prob_params.lbx[drone_virt_velx] = -inf;
    prob_params.lbx[drone_virt_vely] = -inf;
    prob_params.lbx[drone_virt_velz] = -inf;

    prob_params.ubx[drone_virt_posx] = inf;
    prob_params.ubx[drone_virt_posy] = inf;
    prob_params.ubx[drone_virt_posz] = inf;
    prob_params.ubx[drone_virt_velx] = inf;
    prob_params.ubx[drone_virt_vely] = inf;
    prob_params.ubx[drone_virt_velz] = inf;

}

void TTIOptimizerInterface::update_initial_guess(tracking::TrackData drone, tracking::TrackData insect) {

    // The better the initial guess the faster the optimization time and the more certain the optimizer returns a good result:
    float pos_err = norm(drone.pos() - insect.pos());
    float vel_err = norm(drone.vel() - insect.vel());
    float effective_thrust = optimization_thrust(*_thrust) / sqrtf(3.f);
    float est_tti =  vel_err / effective_thrust + sqrt(2 * pos_err / effective_thrust);

    prob_params.X0[dt] = static_cast<double>(est_tti); // It seems better for the optimizer stability to estimate the time too high than to low

    prob_params.X0[drone_posx_1] = static_cast<double>(drone.pos().x);
    prob_params.X0[drone_posy_1] = static_cast<double>(drone.pos().y);
    prob_params.X0[drone_posz_1] = static_cast<double>(drone.pos().z);

    prob_params.X0[drone_velx_1] = static_cast<double>(drone.vel().x);
    prob_params.X0[drone_vely_1] = static_cast<double>(drone.vel().y);
    prob_params.X0[drone_velz_1] = static_cast<double>(drone.vel().z);

    prob_params.X0[insect_posx_1] = static_cast<double>(insect.pos().x);
    prob_params.X0[insect_posy_1] = static_cast<double>(insect.pos().y);
    prob_params.X0[insect_posz_1] = static_cast<double>(insect.pos().z);

    prob_params.X0[insect_velx_1] = static_cast<double>(insect.vel().x);
    prob_params.X0[insect_vely_1] = static_cast<double>(insect.vel().y);
    prob_params.X0[insect_velz_1] = static_cast<double>(insect.vel().z);

    cv::Point3f insect_predicted_pos = insect.pos() + static_cast<float>(prob_params.X0[dt]) * insect.vel();

    cv::Point3f drone_vel2 = drone.vel() + est_tti * (drone.pos() - insect.pos()) / pos_err * effective_thrust;

    prob_params.X0[drone_posx_2] = static_cast<double>(insect_predicted_pos.x);
    prob_params.X0[drone_posy_2] = static_cast<double>(insect_predicted_pos.y);
    prob_params.X0[drone_posz_2] = static_cast<double>(insect_predicted_pos.z);

    prob_params.X0[drone_velx_2] = static_cast<double>(drone_vel2.x);
    prob_params.X0[drone_vely_2] = static_cast<double>(drone_vel2.y);
    prob_params.X0[drone_velz_2] = static_cast<double>(drone_vel2.z);

    prob_params.X0[insect_posx_2] = static_cast<double>(insect_predicted_pos.x);
    prob_params.X0[insect_posy_2] = static_cast<double>(insect_predicted_pos.y);
    prob_params.X0[insect_posz_2] = static_cast<double>(insect_predicted_pos.z);

    prob_params.X0[insect_velx_2] = static_cast<double>(insect.vel().x);
    prob_params.X0[insect_vely_2] = static_cast<double>(insect.vel().y);
    prob_params.X0[insect_velz_2] = static_cast<double>(insect.vel().z);


    cv::Point3f insect_direction = (insect_predicted_pos - drone.pos());
    insect_direction /= norm(insect_direction);

    prob_params.X0[drone_accx] = static_cast<double>(insect_direction.x * effective_thrust);
    prob_params.X0[drone_accy] = static_cast<double>(insect_direction.y * effective_thrust);
    prob_params.X0[drone_accz] = static_cast<double>(insect_direction.z * effective_thrust);

}

void TTIOptimizerInterface::update_variable_bounds(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect) {
    float effective_max_thrust = optimization_thrust(*_thrust) / sqrtf(3.f);
    prob_params.lbx[drone_posx_1] = static_cast<double>(track_data_drone.pos().x);
    prob_params.lbx[drone_posy_1] = static_cast<double>(track_data_drone.pos().y);
    prob_params.lbx[drone_posz_1] = static_cast<double>(track_data_drone.pos().z);
    prob_params.lbx[drone_velx_1] = static_cast<double>(track_data_drone.vel().x);
    prob_params.lbx[drone_vely_1] = static_cast<double>(track_data_drone.vel().y);
    prob_params.lbx[drone_velz_1] = static_cast<double>(track_data_drone.vel().z);
    prob_params.lbx[insect_posx_1] = static_cast<double>(track_data_insect.pos().x);
    prob_params.lbx[insect_posy_1] = static_cast<double>(track_data_insect.pos().y);
    prob_params.lbx[insect_posz_1] = static_cast<double>(track_data_insect.pos().z);
    prob_params.lbx[insect_velx_1] = static_cast<double>(track_data_insect.vel().x);
    prob_params.lbx[insect_vely_1] = static_cast<double>(track_data_insect.vel().y);
    prob_params.lbx[insect_velz_1] = static_cast<double>(track_data_insect.vel().z);
    prob_params.lbx[insect_velx_2] = static_cast<double>(track_data_insect.vel().x);
    prob_params.lbx[insect_vely_2] = static_cast<double>(track_data_insect.vel().y);
    prob_params.lbx[insect_velz_2] = static_cast<double>(track_data_insect.vel().z);
    prob_params.lbx[drone_accx] = static_cast<double>(- effective_max_thrust);
    prob_params.lbx[drone_accy] = static_cast<double>(- effective_max_thrust);
    prob_params.lbx[drone_accz] = static_cast<double>(- effective_max_thrust);

    prob_params.ubx[drone_posx_1] = static_cast<double>(track_data_drone.pos().x);
    prob_params.ubx[drone_posy_1] = static_cast<double>(track_data_drone.pos().y);
    prob_params.ubx[drone_posz_1] = static_cast<double>(track_data_drone.pos().z);
    prob_params.ubx[drone_velx_1] = static_cast<double>(track_data_drone.vel().x);
    prob_params.ubx[drone_vely_1] = static_cast<double>(track_data_drone.vel().y);
    prob_params.ubx[drone_velz_1] = static_cast<double>(track_data_drone.vel().z);
    prob_params.ubx[insect_posx_1] = static_cast<double>(track_data_insect.pos().x);
    prob_params.ubx[insect_posy_1] = static_cast<double>(track_data_insect.pos().y);
    prob_params.ubx[insect_posz_1] = static_cast<double>(track_data_insect.pos().z);
    prob_params.ubx[insect_velx_1] = static_cast<double>(track_data_insect.vel().x);
    prob_params.ubx[insect_vely_1] = static_cast<double>(track_data_insect.vel().y);
    prob_params.ubx[insect_velz_1] = static_cast<double>(track_data_insect.vel().z);
    prob_params.ubx[insect_velx_2] = static_cast<double>(track_data_insect.vel().x);
    prob_params.ubx[insect_vely_2] = static_cast<double>(track_data_insect.vel().y);
    prob_params.ubx[insect_velz_2] = static_cast<double>(track_data_insect.vel().z);
    prob_params.ubx[drone_accx] = static_cast<double>(effective_max_thrust);
    prob_params.ubx[drone_accy] = static_cast<double>(effective_max_thrust);
    prob_params.ubx[drone_accz] = static_cast<double>(effective_max_thrust);

}

tti_result TTIOptimizerInterface::find_best_interception(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect) {
    tti_result res;
    if (norm(track_data_drone.pos() - track_data_insect.pos()) < 0.01) {
        return res;
    }

    update_initial_guess(track_data_drone, track_data_insect);
    update_variable_bounds(track_data_drone, track_data_insect);

#ifdef PATS_OCP_PROFILING
    std::chrono::_V2::system_clock::time_point t_start, t_end;
    t_start = std::chrono::high_resolution_clock::now();
#endif

    Eigen::VectorXd opti_var;

#ifdef OCP_DEV
    if (use_casadi)
        opti_var = sqpsolver.solve_casadi(&prob_params);
    else
#endif
        opti_var = sqpsolver.solve_line_search(&prob_params);


#ifdef PATS_OCP_PROFILING
    t_end = std::chrono::high_resolution_clock::now();
    std::cout << "TTI: Optimization time: " << (t_end - t_start).count() * 1E-6 << " ms" << std::endl;
#endif
    res.time_to_intercept = opti_var[dt];
    res.position_to_intercept = cv::Point3f(opti_var[drone_posx_2], opti_var[drone_posy_2], opti_var[drone_posz_2]);
    if (feasible_solution(opti_var))
        res.valid = true;
    return res;
}


bool TTIOptimizerInterface::feasible_solution(Eigen::VectorXd opti_var) {

    if (opti_var.norm() < 0.1) {
        // std::cout << "INFEASIBLE: Xopt=0!" << std::endl;
        return false;
    }

    double virt_input_sum = abs(opti_var[drone_virt_posx]) + abs(opti_var[drone_virt_posy]) + abs(opti_var[drone_virt_posz])
                            + abs(opti_var[drone_virt_velx]) + abs(opti_var[drone_virt_vely]) + abs(opti_var[drone_virt_velz]);
    if (virt_input_sum > 0.001) {
        // std::cout << "virt_input_sum(tti): " << virt_input_sum << std::endl;
        return false;
    }

    cv::Point3f intercept_pos_drone = cv::Point3f(opti_var[drone_posx_2], opti_var[drone_posy_2], opti_var[drone_posz_2]);
    cv::Point3f intercept_pos_insect = cv::Point3f(opti_var[insect_posx_2], opti_var[insect_posy_2], opti_var[insect_posz_2]);
    float intercept_error = normf(intercept_pos_drone - intercept_pos_insect);
    if (intercept_error > 0.02f) {
        // std::cout << "intercept error(tti): " << intercept_error << std::endl;
        return false;
    }

    return true;
}
