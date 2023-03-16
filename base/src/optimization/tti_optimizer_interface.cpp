#include "tti_optimizer_interface.h"
#include "tti_index.h"
#include <chrono>

void TTIOptimizerInterface::init(float *thrust) {
    _thrust = thrust;
    qpsolver.init();
    sqpsolver.init(&qpsolver);
    sqp_solver_configuration config(30, 1e-2, 1e-2, 1e-1);
    sqpsolver.setup(config);
    prob_params = problem_parameters(34, 15, 0);
    init_box_constraints();
}

void TTIOptimizerInterface::qp_setup(QPSettings qpsettings) {
    qpsolver.qp_setup(qpsettings);
}

std::string TTIOptimizerInterface::version() {return TTI_VERSION;};

void TTIOptimizerInterface::init_box_constraints() {
    prob_params.lbx.setZero();
    prob_params.ubx.setZero();
    prob_params.lbx[delta_tI] = 1e-06; //, 0.0616, -inf, -1.28, -inf, -1.322, -inf, 0, -inf, 0, -inf, 0, -inf, -20, -20, -20, -inf, -inf, -inf, -inf, -inf, -inf, 0.99, -inf, -0.4, -inf, -1.899, -inf, -0.5, -0.5, 0, 0, 0, 0};
    prob_params.ubx[delta_tI] = inf; //, 0.0616, inf, -1.28, inf, -1.322, inf, 0, inf, 0, inf, 0, inf, 20, 20, 20, inf, inf, inf, inf, inf, inf, 0.99, inf, -0.4, inf, -1.899, inf, -0.5, -0.5, 0, 0, 0, 0};
    for (uint i = statesI_first + N_DRONE_STATES; i <= statesI_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] = inf;

    }
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_POSX] = -inf;
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_POSY] = -inf;
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_POSZ] = -inf;
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_POSX] = inf;
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_POSY] = inf;
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_POSZ] = inf;

    for (uint i = virtual_inputsI_first; i <= virtual_inputsI_last; i++) {
        prob_params.lbx[i] = -inf;
        prob_params.ubx[i] = inf;
    }
}

void TTIOptimizerInterface::update_initial_guess(tracking::TrackData drone, tracking::TrackData insect) {

    // The better the initial guess the faster the optimization time and the more certain the optimizer returns a good result:
    float pos_err = norm(drone.pos() - insect.pos());
    float vel_err = norm(drone.vel() - insect.vel());
    float effective_thrust = optimization_thrust(*_thrust) / sqrtf(3.f);
    float est_tti =  vel_err / effective_thrust + sqrt(2 * pos_err / effective_thrust);
    prob_params.X0[delta_tI] = static_cast<double>(est_tti); // It seems better for the optimizer stability to estimate the time too high than to low
    prob_params.X0[statesI_first + DRONE_POSX] = static_cast<double>(drone.pos().x);
    prob_params.X0[statesI_first + DRONE_POSY] = static_cast<double>(drone.pos().y);
    prob_params.X0[statesI_first + DRONE_POSZ] = static_cast<double>(drone.pos().z);
    prob_params.X0[statesI_first + DRONE_VELX] = static_cast<double>(drone.vel().x);
    prob_params.X0[statesI_first + DRONE_VELY] = static_cast<double>(drone.vel().y);
    prob_params.X0[statesI_first + DRONE_VELZ] = static_cast<double>(drone.vel().z);
    prob_params.X0[insect_states_first + INSECT_POSX] = static_cast<double>(insect.pos().x);
    prob_params.X0[insect_states_first + INSECT_POSY] = static_cast<double>(insect.pos().y);
    prob_params.X0[insect_states_first + INSECT_POSZ] = static_cast<double>(insect.pos().z);
    prob_params.X0[insect_states_first + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.X0[insect_states_first + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.X0[insect_states_first + INSECT_VELZ] = static_cast<double>(insect.vel().z);

    cv::Point3f insect_predicted_pos = insect.pos() + static_cast<float>(prob_params.X0[delta_tI]) * insect.vel();
    cv::Point3f drone_vel2 = drone.vel() + est_tti * (insect.pos() - drone.pos()) / pos_err * effective_thrust;
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_POSX] = static_cast<double>(insect_predicted_pos.x);
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_POSY] = static_cast<double>(insect_predicted_pos.y);
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_POSZ] = static_cast<double>(insect_predicted_pos.z);
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_VELX] = static_cast<double>(drone_vel2.x);
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_VELY] = static_cast<double>(drone_vel2.y);
    prob_params.X0[statesI_first + N_DRONE_STATES + DRONE_VELZ] = static_cast<double>(drone_vel2.z);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_POSX] = static_cast<double>(insect_predicted_pos.x);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_POSY] = static_cast<double>(insect_predicted_pos.y);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_POSZ] = static_cast<double>(insect_predicted_pos.z);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.X0[insect_states_first + N_INSECT_STATES + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    cv::Point3f insect_direction = (insect_predicted_pos - drone.pos());
    insect_direction /= norm(insect_direction);
    prob_params.X0[inputsI_first + DRONE_ACCX] = static_cast<double>(insect_direction.x * effective_thrust);
    prob_params.X0[inputsI_first + DRONE_ACCY] = static_cast<double>(insect_direction.y * effective_thrust);
    prob_params.X0[inputsI_first + DRONE_ACCZ] = static_cast<double>(insect_direction.z * effective_thrust);
    // qpsolver.print_eigenvector("X0", prob_params.X0);
}

void TTIOptimizerInterface::update_box_constraints(tracking::TrackData drone, tracking::TrackData insect) {
    float effective_max_thrust = optimization_thrust(*_thrust) / sqrtf(3.f);
    prob_params.lbx[statesI_first + DRONE_POSX] = static_cast<double>(drone.pos().x);
    prob_params.lbx[statesI_first + DRONE_POSY] = static_cast<double>(drone.pos().y);
    prob_params.lbx[statesI_first + DRONE_POSZ] = static_cast<double>(drone.pos().z);
    prob_params.lbx[statesI_first + DRONE_VELX] = static_cast<double>(drone.vel().x);
    prob_params.lbx[statesI_first + DRONE_VELY] = static_cast<double>(drone.vel().y);
    prob_params.lbx[statesI_first + DRONE_VELZ] = static_cast<double>(drone.vel().z);
    prob_params.lbx[insect_states_first + INSECT_POSX] = static_cast<double>(insect.pos().x);
    prob_params.lbx[insect_states_first + INSECT_POSY] = static_cast<double>(insect.pos().y);
    prob_params.lbx[insect_states_first + INSECT_POSZ] = static_cast<double>(insect.pos().z);
    prob_params.lbx[insect_states_first + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.lbx[insect_states_first + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.lbx[insect_states_first + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.lbx[insect_states_first + N_INSECT_STATES + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    prob_params.lbx[inputsI_first + DRONE_ACCX] = static_cast<double>(- effective_max_thrust);
    prob_params.lbx[inputsI_first + DRONE_ACCY] = static_cast<double>(- effective_max_thrust);
    prob_params.lbx[inputsI_first + DRONE_ACCZ] = static_cast<double>(- effective_max_thrust);

    prob_params.ubx[statesI_first + DRONE_POSX] = static_cast<double>(drone.pos().x);
    prob_params.ubx[statesI_first + DRONE_POSY] = static_cast<double>(drone.pos().y);
    prob_params.ubx[statesI_first + DRONE_POSZ] = static_cast<double>(drone.pos().z);
    prob_params.ubx[statesI_first + DRONE_VELX] = static_cast<double>(drone.vel().x);
    prob_params.ubx[statesI_first + DRONE_VELY] = static_cast<double>(drone.vel().y);
    prob_params.ubx[statesI_first + DRONE_VELZ] = static_cast<double>(drone.vel().z);
    prob_params.ubx[insect_states_first + INSECT_POSX] = static_cast<double>(insect.pos().x);
    prob_params.ubx[insect_states_first + INSECT_POSY] = static_cast<double>(insect.pos().y);
    prob_params.ubx[insect_states_first + INSECT_POSZ] = static_cast<double>(insect.pos().z);
    prob_params.ubx[insect_states_first + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.ubx[insect_states_first + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.ubx[insect_states_first + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_VELX] = static_cast<double>(insect.vel().x);
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_VELY] = static_cast<double>(insect.vel().y);
    prob_params.ubx[insect_states_first + N_INSECT_STATES + INSECT_VELZ] = static_cast<double>(insect.vel().z);
    prob_params.ubx[inputsI_first + DRONE_ACCX] = static_cast<double>(effective_max_thrust);
    prob_params.ubx[inputsI_first + DRONE_ACCY] = static_cast<double>(effective_max_thrust);
    prob_params.ubx[inputsI_first + DRONE_ACCZ] = static_cast<double>(effective_max_thrust);
}

tti_result TTIOptimizerInterface::find_best_interception(tracking::TrackData drone, tracking::TrackData insect) {
    tti_result res;
    if (norm(drone.pos() - insect.pos()) < 0.01) {
        return res;
    }

    update_initial_guess(drone, insect);
    update_box_constraints(drone, insect);

#ifdef OPTI_ROSVIS
    _ros_interface->drone(drone);
    _ros_interface->insect(insect);
    _ros_interface->path(qpsolver.trajectory(prob_params.X0, state_trajectory_t), opti_initial_guess);
    _ros_interface->state_trajectory(qpsolver.trajectory(prob_params.X0, state_trajectory_t));
    _ros_interface->input_trajectory(qpsolver.trajectory(prob_params.X0, state_trajectory_t), qpsolver.trajectory(prob_params.X0, input_trajectory_t), drone_input_trajectory);
    _ros_interface->input_trajectory(qpsolver.trajectory(prob_params.X0, state_trajectory_t), qpsolver.trajectory(prob_params.X0, virtual_input_trajectory_t), drone_virtual_input_trajectory);
    _ros_interface->publish();
#endif
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
    res.time_to_intercept = opti_var[delta_tI];
    res.position_to_intercept = cv::Point3f(opti_var[statesI_first + N_DRONE_STATES + DRONE_POSX], opti_var[statesI_first + N_DRONE_STATES + DRONE_POSY], opti_var[statesI_first + N_DRONE_STATES + DRONE_POSZ]);

    if (feasible_solution(opti_var))
        res.valid = true;

#ifdef OPTI_ROSVIS
    _ros_interface->drone(drone);
    _ros_interface->insect(insect);
    _ros_interface->path(qpsolver.trajectory(opti_var, state_trajectory_t), opti_solution);
    _ros_interface->state_trajectory(qpsolver.trajectory(opti_var, state_trajectory_t));
    _ros_interface->input_trajectory(qpsolver.trajectory(opti_var, state_trajectory_t), qpsolver.trajectory(opti_var, input_trajectory_t), drone_input_trajectory);
    _ros_interface->input_trajectory(qpsolver.trajectory(opti_var, state_trajectory_t), qpsolver.trajectory(opti_var, virtual_input_trajectory_t), drone_virtual_input_trajectory);
    _ros_interface->publish();
#endif

    return res;
}


bool TTIOptimizerInterface::feasible_solution(Eigen::VectorXd opti_var) {
    // qpsolver.print_eigenvector("Xopt(tti):", opti_var);

    if (opti_var.norm() < 0.1) {
        // std::cout << "INFEASIBLE: Xopt=0!" << std::endl;
        return false;
    }

    double virt_input_sum = 0;

    for (uint i = virtual_inputsI_first; i <= virtual_inputsI_last; i++) {
        virt_input_sum += abs(opti_var[i]);
    }

    if (virt_input_sum > 0.001) {
        // std::cout << "virt_input_sum(tti): " << virt_input_sum << std::endl;
        return false;
    }

    cv::Point3f intercept_pos_drone = cv::Point3f(opti_var[statesI_first + N_DRONE_STATES + DRONE_POSX], opti_var[statesI_first + N_DRONE_STATES + DRONE_POSY], opti_var[statesI_first + N_DRONE_STATES + DRONE_POSZ]);
    // cv::Point3f intercept_vel_drone = cv::Point3f(opti_var[statesI_first + N_DRONE_STATES + DRONE_VELX], opti_var[statesI_first + N_DRONE_STATES + DRONE_VELY], opti_var[statesI_first + N_DRONE_STATES + DRONE_VELZ]);
    // std::cout << "Intercept vel drone: " << intercept_vel_drone << std::endl;
    cv::Point3f intercept_pos_insect = cv::Point3f(opti_var[insect_states_first + N_INSECT_STATES + INSECT_POSX], opti_var[insect_states_first + N_INSECT_STATES + INSECT_POSY], opti_var[insect_states_first + N_INSECT_STATES + INSECT_POSZ]);
    float intercept_error = normf(intercept_pos_drone - intercept_pos_insect);
    if (intercept_error > 0.01f) {
        // std::cout << "intercept error(tti): " << intercept_error << std::endl;
        return false;
    }

    return true;
}
