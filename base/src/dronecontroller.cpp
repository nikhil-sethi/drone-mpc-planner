#include "dronecontroller.h"
#include "rc.h"
#include "flightplan.h"
#include "quaternion.h"
#include "linalg.h"

using namespace cv;
using namespace tracking;

// Create an instance of Joystick
static Joystick joystick("/dev/input/js0");
static JoystickEvent event;

bool DroneController::joystick_ready() {
    return joystick.isFound();
}

void DroneController::init(std::ofstream *logger, string replay_dir, bool generator, bool airsim, Rc *rc, tracking::DroneTracker *dtrk, FlightArea *flight_area, float exposure) {
    _rc = rc;
    _dtrk = dtrk;
    _logger = logger;
    log_replay_mode = replay_dir != "";
    generator_mode = generator;
    airsim_mode = airsim;
    _flight_area = flight_area;

    control_history_max_size = pparams.fps;
    (*_logger) << "valid;flight_mode;flight_mode_str;" <<
               "target_pos_x;target_pos_y;target_pos_z;" <<
               "autoThrottle;autoRoll;autoPitch;autoYaw;" <<
               "joyThrottle;joyRoll;joyPitch;joyYaw; " <<
               "joyArmSwitch;joyModeSwitch;joyTakeoffSwitch;" <<
               "mmArmSwitch;mmModeSwitch;" <<
               "dt;kiv;" <<
               "thrust; integrator_x;integrator_y;integrator_z;" <<
               "batt_cell_v;rssi;arm;";
    ;
    std::cout << "Initialising control." << std::endl;

    load_calibration(replay_dir);
    load_control_parameters();

    if (pparams.control_tuning) {
        std::cout << "Creating control tuning window." << std::endl;
        // create GUI to set control parameters
        namedWindow("Control", WINDOW_NORMAL);
        createTrackbar("p_pos_roll", "Control", &kp_pos_roll, 3000);
        createTrackbar("p_pos_pitch", "Control", &kp_pos_pitch, 3000);
        createTrackbar("p_pos_throttle", "Control", &kp_pos_throttle, 3000);
        createTrackbar("i_pos_roll", "Control", &ki_pos_roll, 1000);
        createTrackbar("i_pos_pitch", "Control", &ki_pos_pitch, 1000);
        createTrackbar("i_thrust", "Control", &ki_thrust, 1000);
        createTrackbar("d_pos_roll", "Control", &kd_pos_roll, 1000);
        createTrackbar("d_pos_pitch", "Control", &kd_pos_pitch, 1000);
        createTrackbar("d_pos_throttle", "Control", &kd_pos_throttle, 1000);
        createTrackbar("p_v_roll", "Control", &kp_v_roll, 1000);
        createTrackbar("p_v_pitch", "Control", &kp_v_pitch, 1000);
        createTrackbar("p_v_throttle", "Control", &kp_v_throttle, 1000);
        createTrackbar("d_v_roll", "Control", &kd_v_roll, 100);
        createTrackbar("d_v_pitch", "Control", &kd_v_pitch, 100);
        createTrackbar("d_v_throttle", "Control", &kd_v_throttle, 200);
        createTrackbar("Hover p_pos_roll", "Control", &kp_pos_roll_hover, 3000);
        createTrackbar("Hover p_pos_pitch", "Control", &kp_pos_pitch_hover, 3000);
        createTrackbar("Hover p_pos_throttle", "Control", &kp_pos_throttle_hover, 3000);
        createTrackbar("Hover i_pos_roll", "Control", &ki_pos_roll_hover, 1000);
        createTrackbar("Hover i_pos_pitch", "Control", &ki_pos_pitch_hover, 1000);
        createTrackbar("Hover i_thrust", "Control", &ki_thrust_hover, 1000);
        createTrackbar("Hover d_pos_roll", "Control", &kd_pos_roll_hover, 1000);
        createTrackbar("Hover d_pos_pitch", "Control", &kd_pos_pitch_hover, 1000);
        createTrackbar("Hover d_pos_throttle", "Control", &kd_pos_throttle_hover, 1000);
    }

    d_vel_err_x.init(1.f / pparams.fps);
    d_vel_err_y.init(1.f / pparams.fps);
    d_vel_err_z.init(1.f / pparams.fps);
    pos_err_i = {0};
    pos_modelx.init(1.f / pparams.fps, 1, 0.2, 0.38);
    pos_modely.init(1.f / pparams.fps, 1, 0.2, 0.38);
    pos_modelz.init(1.f / pparams.fps, 1, 0.2, 0.38);

    set_led_strength(exposure);

    calibration.thrust = std::clamp(calibration.thrust, dparams.default_thrust / 2.f, dparams.default_thrust * 2.f);
    pad_att_calibration_roll.init(required_pad_att_calibration_cnt);
    pad_att_calibration_pitch.init(required_pad_att_calibration_cnt);
    initial_hover_throttle_guess_non3d = GRAVITY / dparams.default_thrust * RC_BOUND_RANGE + dparams.min_throttle;
    initialized = true;
}

void DroneController::set_led_strength(float exposure) {
    if (dparams.led_type == led_fiber_uv || dparams.led_type == led_top_uv)
        dparams.drone_led_strength = 100;
    else {
        float max_ae = 20000;
        if (pparams.fps == 90)
            max_ae = 10000;
        else if (pparams.fps == 60)
            max_ae = 15000;
        else
            std::cout << "Warning, led strength not properly implemented for this fps!" << std::endl;

        float thresh_dark = (0.1333f * max_ae);
        float thresh_bright = (0.0333f * max_ae);

        if (exposure < thresh_bright)
            dparams.drone_led_strength = 100;
        else if (exposure < thresh_dark)
            dparams.drone_led_strength = 100 - (exposure / thresh_dark * 50.f);
        else
            dparams.drone_led_strength = 50 - ((std::clamp(exposure, thresh_bright, max_ae) - thresh_bright) / (max_ae - thresh_bright) * 35.f);
    }
    std::cout << "Led strength set to: " << dparams.drone_led_strength << std::endl;
}

void DroneController::control(TrackData data_drone, TrackData data_target_new, TrackData data_raw_insect, double time) {
    _time = time;

    if (!log_replay_mode && pparams.joystick != rc_none)
        read_joystick();
    process_joystick();

    if (_joy_state == js_waypoint || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_aim)
        data_raw_insect = data_target_new; // the takeoff burn uses raw insect, but wp flight mode also uses takeoff burn
    _dtrk->update_target(data_raw_insect.pos());

    if (!data_raw_insect.pos_valid) // if tracking is lost, set the raw setpoint to just above takeoff location, because otherwise the (takeoff) may go towards 0,0,0 (the camera)
        data_raw_insect.state.pos = _dtrk->pad_location() + cv::Point3f(0, 0.5, 0);

    _dist_to_setpoint = normf(data_drone.state.pos - data_target_new.state.pos);

    int throttle = 0, roll = 0, pitch = 0, yaw = 0, mode = bf_angle;
    bool joy_control = false;
    switch (_flight_mode) {
        case fm_manual: {
                mode += bf_headless_disabled;
                throttle = joy_throttle;
                _rc->arm(_joy_arm_switch);
                roll = joy_roll;
                pitch = joy_pitch;
                yaw = joy_yaw;
                joy_control = true;
                break;
        } case fm_blink: {
                _rc->arm(bf_disarmed);
                blink(time);
                calibrate_pad_attitude();
                //#268:
                // mode+=bf_spin_motor;
                // blink_motors(time);
                break;
        } case fm_spinup: {
                _rc->arm(bf_armed);
#if ENABLE_SPINUP==true
                start_takeoff_burn_time = 0;
                if (spin_up_start_time < 0.01)
                    spin_up_start_time = time;
                auto_throttle = spinup_throttle();
#else
                auto_throttle = JOY_BOUND_MIN;
                spin_up_start_time = 0;
#endif
                mode += bf_PID_loop_disabled;
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                break;
        } case fm_start_takeoff: {
                take_off_start_time = time;
                remaining_spinup_duration_t0 = max(0.f, dparams.full_bat_and_throttle_spinup_duration - aim_duration - time_spent_spinning_up(time));
                _flight_mode = fm_take_off_aim;
                _burn_direction_for_thrust_approx = {0};
                auto_throttle = spinup_throttle();
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;

                if (dparams.mode3d) {
                    _rc->arm(bf_armed);
                    break;
                }
                [[fallthrough]];
        } case fm_take_off_aim: {
                StateData state_drone_takeoff = data_drone.state;
                state_drone_takeoff.pos = _dtrk->pad_location() + cv::Point3f(0, lift_off_dist_take_off_aim, 0);
                state_drone_takeoff.vel = {0};

                bool burn_limit_hack = true;
                float remaining_aim_duration = remaining_spinup_duration_t0 + aim_duration  - static_cast<float>(time - take_off_start_time);
                if (remaining_aim_duration <= aim_duration) {
                    cv::Point3f burn_direction;
                    std::tie(auto_roll, auto_pitch, auto_burn_duration, burn_direction) = calc_directional_burn(state_drone_takeoff, data_raw_insect.state, 0);
                    if (burn_limit_hack)
                        auto_burn_duration = dparams.target_takeoff_velocity / calibration.thrust;
                    aim_direction_history.push_back(burn_direction);
                    auto_throttle = initial_hover_throttle_guess();

                    _burn_direction_for_thrust_approx = burn_direction; // to be used later to approx effective thrust
                    std::vector<StateData> trajectory = predict_trajectory(auto_burn_duration, 0, burn_direction, state_drone_takeoff);
                    if (log_replay_mode)
                        draw_viz(state_drone_takeoff, data_raw_insect.state, time, burn_direction, auto_burn_duration, remaining_aim_duration, trajectory);
                } else
                    auto_throttle = spinup_throttle();
                if (remaining_aim_duration <= 0) {
                    remaining_aim_duration = 0;

                    //check if insect path was constant:
                    cv::Point3f avg_dir = {0};
                    for (auto dir : aim_direction_history)
                        avg_dir += dir;

                    avg_dir /= static_cast<float>(aim_direction_history.size());
                    cv::Point3f predicted_drone_pos;
                    std::tie(predicted_drone_pos, std::ignore, std::ignore) = predict_drone_after_burn(state_drone_takeoff, avg_dir, 0, auto_burn_duration);
                    cv::Point3f predicted_target_pos = data_raw_insect.pos() + auto_burn_duration * data_raw_insect.vel();

                    double predicted_error = cv::norm(predicted_drone_pos - predicted_target_pos);
                    if (predicted_error > 0.1 && !burn_limit_hack) {
                        _flight_mode = fm_spinup;
                        spin_up_start_time = time - static_cast<double>(dparams.full_bat_and_throttle_spinup_duration); // normal spinup may not have happened always, so just set it hard so that we don't spinup wait next attempt
                    } else {
                        spin_up_start_time = 0;
                        _flight_mode = fm_max_burn;
                        start_takeoff_burn_time = time;
                        std::cout << "Take off burn" << std::endl;
                    }

                    aim_direction_history.clear();

                }
                break;
        } case fm_max_burn: {
                auto_throttle = RC_BOUND_MAX;
                if (static_cast<float>(time - start_takeoff_burn_time) >  auto_burn_duration)
                    _flight_mode = fm_max_burn_spin_down;
                break;
        } case fm_max_burn_spin_down: {
                auto_throttle = initial_hover_throttle_guess();
                if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration)
                    _flight_mode = fm_1g;
                break;
        }  case fm_1g: {
                // Wait until velocity of drone after take-off is constant, then estimate this velocity using sufficient samples
                if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration + transmission_delay_duration - 1.f / pparams.fps)
                    _flight_mode = fm_flying_pid_init;
                break;
        }  case fm_interception_aim_start: {
                std::cout << "Aiming" << std::endl;
                interception_start_time = time;
                approx_effective_thrust(data_drone, _burn_direction_for_thrust_approx, auto_burn_duration, static_cast<float>(time - start_takeoff_burn_time));
                _burn_direction_for_thrust_approx = {0};
                _flight_mode =   fm_interception_aim;
                [[fallthrough]];
        }   case fm_interception_aim: {
                cv::Point3f burn_direction;
                StateData state_drone_better = data_drone.state;
                state_drone_better.vel = vel_after_takeoff;

                float remaining_aim_duration = aim_duration - static_cast<float>(time - interception_start_time);
                data_raw_insect.state.vel = {0}; // aim to the current target position

                if (remaining_aim_duration < 0)
                    remaining_aim_duration = 0;
                std::vector<StateData> traj;
                std::tie(auto_roll, auto_pitch, auto_burn_duration, burn_direction, traj) = calc_burn(state_drone_better, data_raw_insect.state, remaining_aim_duration);

                if (log_replay_mode)
                    draw_viz(state_drone_better, data_raw_insect.state, time, burn_direction, auto_burn_duration, remaining_aim_duration, traj);

                auto_throttle = initial_hover_throttle_guess();

                if (!_flight_area->trajectory_in_view(traj, relaxed) || auto_burn_duration > 1.1f || auto_burn_duration == 0.0f) {
                    _flight_mode = fm_flying_pid_init;
                } else {
                    std::vector<StateData> traj_back;
                    cv::Point3f burn_direction_back;
                    float auto_burn_duration_back;
                    StateData target_back = traj.front();
                    target_back.vel = {0};
                    target_back.acc = {0};
                    std::tie(std::ignore, std::ignore, auto_burn_duration_back, burn_direction_back, traj_back) = calc_burn(traj.back(), target_back, aim_duration);
                    if (!_flight_area->trajectory_in_view(traj_back, relaxed)) {
                        _flight_mode = fm_flying_pid_init;
                    } else if (remaining_aim_duration < 0.01f)
                        _flight_mode = fm_interception_burn_start;
                }
                break;
        } case fm_interception_burn_start: {
                _flight_mode = fm_interception_burn;
                auto_throttle = RC_BOUND_MAX;
                std::cout << "Burning: " << auto_burn_duration << std::endl;
                [[fallthrough]];
        } case fm_interception_burn: {
                if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration) {
                    _flight_mode = fm_interception_burn_spin_down;
                    std::cout << "Spindown" << std::endl;
                }
                break;
        } case fm_interception_burn_spin_down: {
                auto_throttle = initial_hover_throttle_guess();

                float spindown_duration = effective_burn_spin_down_duration;
                if (effective_burn_spin_down_duration > auto_burn_duration * 0.8f)
                    spindown_duration = auto_burn_duration * 0.8f;

                if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration + spindown_duration)
                    _flight_mode = fm_retry_aim_start;
                break;
        }  case fm_retry_aim_start: {
                interception_start_time = time;
                _flight_mode = fm_interception_aim;
                std::cout << "Re-aiming" << std::endl;
                _burn_direction_for_thrust_approx = {0};
                break;
        } case fm_flying_pid_init: {
                _flight_mode = fm_flying_pid;

                d_vel_err_x.preset(data_target_new.vel().x - data_drone.state.vel.x);
                d_vel_err_y.preset(data_target_new.vel().y - data_drone.state.vel.y);
                d_vel_err_z.preset(data_target_new.vel().z - data_drone.state.vel.z);

                if (!data_drone.pos_valid && _time - take_off_start_time < 0.5) {
                    pos_modelx.internal_states(_dtrk->pad_location().x, _dtrk->pad_location().x);
                    pos_modely.internal_states(_dtrk->pad_location().y, _dtrk->pad_location().y);
                    pos_modelz.internal_states(_dtrk->pad_location().z, _dtrk->pad_location().z);

                } else {
                    pos_modelx.internal_states(data_drone.pos().x, data_drone.pos().x);
                    pos_modely.internal_states(data_drone.pos().y, data_drone.pos().y);
                    pos_modelz.internal_states(data_drone.pos().z, data_drone.pos().z);

                    if (data_drone.vel_valid) {
                        float correction_gain = 5.f;
                        pos_modelx.internal_states(data_drone.pos().x - data_drone.vel().x / pparams.fps * correction_gain, data_drone.pos().x - 2 * data_drone.vel().x / pparams.fps * correction_gain);
                        pos_modely.internal_states(data_drone.pos().y - data_drone.vel().y / pparams.fps * correction_gain, data_drone.pos().y - 2 * data_drone.vel().y / pparams.fps * correction_gain);
                        pos_modelz.internal_states(data_drone.pos().z - data_drone.vel().z / pparams.fps * correction_gain, data_drone.pos().z - 2 * data_drone.vel().z / pparams.fps * correction_gain);

                    }
                }
                [[fallthrough]];
        } case fm_flying_pid: {
                mode += bf_airmode;
                if (!data_drone.pos_valid) {
                    pos_err_i = {0};
                    if (_time - take_off_start_time < 0.5)
                        data_drone.state.pos = _dtrk->pad_location();
                }
                control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
                break;
        } case fm_long_range_forth: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
                auto_pitch = RC_MIDDLE + RC_BOUND_RANGE / 2 / 8;
                break;
        } case fm_long_range_back: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
                auto_pitch = RC_MIDDLE - RC_BOUND_RANGE / 2 / 8;
                break;
        } case fm_calib_thrust:
        case fm_prep_to_land:
        case fm_reset_headless_yaw:
        case fm_headed: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
                break;
        } case fm_reset_yaw_on_pad: {
                mode += bf_headless_disabled;
                auto_throttle = dparams.spinup_throttle_non3d;
                auto_roll = RC_MIDDLE + static_cast<int>(0.5f * att_reset_yaw_on_pad.x / 180.f  / 2.f * RC_BOUND_RANGE);
                auto_pitch = RC_MIDDLE + static_cast<int>(0.5f * att_reset_yaw_on_pad.y / 180.f  / 2.f * RC_BOUND_RANGE);
                auto_yaw = RC_MIDDLE;
                break;
        } case fm_correct_yaw: {
                mode += bf_headless_disabled;
                control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
                if (data_drone.yaw_deviation_valid)
                    correct_yaw(data_drone.yaw_deviation);
                break;
        } case fm_ff_landing_start: {
                _flight_mode = fm_ff_landing;
                ff_land_start_time = time;
                ff_auto_throttle_start = auto_throttle;
                auto_yaw = RC_MIDDLE;
                [[fallthrough]];
        } case fm_ff_landing: {
                mode += bf_headless_disabled;
                control_model_based(data_drone, data_drone.pos(), data_target_new.vel());
                float dt = static_cast<float>(time - ff_land_start_time);
                auto_throttle = land_ctrl.ff_auto_throttle(ff_auto_throttle_start, dt);

                if (dt > land_ctrl.time_ff_landing()) {
                    auto_throttle = RC_BOUND_MIN;
                    if (dparams.static_shakeit_throttle > 0)
                        _flight_mode = fm_wait;
                    else
                        _flight_mode = fm_inactive;
                }
                break;
        } case fm_start_shake: {
                _n_shakes = 0;
                state_start_time = time;
                _flight_mode = fm_shake_it_baby;
                [[fallthrough]];
        } case fm_shake_it_baby: {
                _rc->arm(bf_disarmed);

                const double shake_period = 0.2;
                const double shake_pause_period = 0.2;
                const int spin_value_static = RC_BOUND_MIN + dparams.static_shakeit_throttle;
                const int spin_value_shake = spin_value_static + 75;
                const int spin_value_shake_reversed = spin_value_static + 150;

                //defaults:
                mode += bf_spin_motor;
                auto_roll = RC_BOUND_MIN;
                auto_pitch = RC_BOUND_MIN;
                auto_throttle = RC_BOUND_MIN;
                auto_yaw = RC_BOUND_MIN;

                static int shake_state = 0;
                switch (shake_state) {
                    case 0: // yaw
                        auto_pitch = spin_value_shake;
                        auto_throttle = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 2:
                        auto_pitch = spin_value_shake;
                        auto_throttle = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 4:
                        mode += bf_spin_motor_reversed;
                        auto_roll = spin_value_shake_reversed;
                        auto_yaw = spin_value_shake_reversed;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 6:
                        mode += bf_spin_motor_reversed;
                        auto_roll = spin_value_shake_reversed;
                        auto_yaw = spin_value_shake_reversed;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;

                    case 8: // pitch
                        auto_roll = spin_value_shake;
                        auto_throttle = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 10:
                        auto_pitch = spin_value_shake;
                        auto_yaw = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;

                    case 12: // roll
                        auto_roll = spin_value_shake;
                        auto_pitch = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 14:
                        auto_throttle = spin_value_shake;
                        auto_yaw = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 16:
                        _n_shakes++;
                        shake_state = 0;
                        break;
                    default:
                        auto_roll = spin_value_static;
                        auto_pitch = spin_value_static;
                        auto_throttle = spin_value_static;
                        auto_yaw = spin_value_static;
                        if (time - state_start_time > shake_pause_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                }

                // // double yaw
                // if (time -shake_period_start_time < shake_period) {
                //     auto_pitch = spin_value_shake;
                //     auto_throttle = spin_value_shake;
                // } else if (time -shake_period_start_time > 2*shake_period && time -shake_period_start_time < 3*shake_period) {
                //     auto_pitch = spin_value_shake;
                //     auto_throttle = spin_value_shake;
                // } else if (time -shake_period_start_time > 4*shake_period && time -shake_period_start_time < 5*shake_period) {
                //     auto_roll = spin_value_shake;
                //     auto_yaw = spin_value_shake;
                // } else if (time -shake_period_start_time > 6*shake_period && time -shake_period_start_time < 7*shake_period) {
                //     auto_roll = spin_value_shake;
                //     auto_yaw = spin_value_shake;

                //     // pitch
                // } else if (time -shake_period_start_time > 8*shake_period && time -shake_period_start_time < 9*shake_period) {
                //     auto_roll = spin_value_shake;
                //     auto_throttle = spin_value_shake;
                // } else if (time -shake_period_start_time > 10*shake_period && time -shake_period_start_time < 11*shake_period) {
                //     auto_pitch = spin_value_shake;
                //     auto_yaw = spin_value_shake;

                //     // roll
                // } else if (time -shake_period_start_time > 12*shake_period && time -shake_period_start_time < 13*shake_period) {
                //     auto_roll = spin_value_shake;
                //     auto_pitch = spin_value_shake;
                // } else if (time -shake_period_start_time > 14*shake_period && time -shake_period_start_time < 15*shake_period) {
                //     auto_throttle = spin_value_shake;
                //     auto_yaw = spin_value_shake;

                // } else if (time -shake_period_start_time > 17*shake_period) {
                //     _n_shakes++;
                //     shake_period_start_time = time;
                // }
                break;
        } case fm_disarmed: {
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                if (dparams.mode3d)
                    auto_throttle = RC_MIDDLE;
                else
                    auto_throttle = RC_BOUND_MIN;
                _rc->arm(bf_disarmed);
                calibrate_pad_attitude();
                break;
        } case fm_inactive: {
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                spin_up_start_time = 0;
                mode += bf_yaw_reset;

                if (dparams.mode3d)
                    auto_throttle = RC_MIDDLE;
                else
                    auto_throttle = RC_BOUND_MIN;
                if (pparams.joystick == rc_none && !dparams.mode3d)
                    _rc->arm(bf_armed);
                else if (dparams.mode3d)
                    _rc->arm(bf_disarmed);
                calibrate_pad_attitude();
                break;

        } case fm_wait: {
                auto_throttle = RC_BOUND_MIN;
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                break;
        } case fm_monitoring: {
                break;
        } case fm_abort: {
                if (dparams.mode3d)
                    auto_throttle = RC_MIDDLE;
                else
                    auto_throttle = RC_BOUND_MIN;
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                break;
        } case fm_joystick_check: {
                if (pparams.joystick == rc_none) {
                    _flight_mode = fm_disarmed;
                    _joy_state = js_none;
                    break;
                }
                _joy_state = js_checking;
                if (dparams.mode3d)
                    auto_throttle = RC_MIDDLE;
                else
                    auto_throttle = RC_BOUND_MIN;
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                _rc->arm(bf_disarmed);
                break;
            }
    }

    if (!joy_control) {
        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;
        yaw = auto_yaw;
    }

    throttle = std::clamp(throttle, RC_BOUND_MIN, RC_BOUND_MAX);
    roll = std::clamp(roll, RC_BOUND_MIN, RC_BOUND_MAX);
    pitch = std::clamp(pitch, RC_BOUND_MIN, RC_BOUND_MAX);
    yaw = std::clamp(yaw, RC_BOUND_MIN, RC_BOUND_MAX);
    auto_throttle = std::clamp(auto_throttle, RC_BOUND_MIN, RC_BOUND_MAX);
    auto_pitch = std::clamp(auto_pitch, RC_BOUND_MIN, RC_BOUND_MAX);
    auto_roll = std::clamp(auto_roll, RC_BOUND_MIN, RC_BOUND_MAX);
    auto_yaw = std::clamp(auto_yaw, RC_BOUND_MIN, RC_BOUND_MAX);

    // std::cout <<  " rpyt: " << roll << ", " << pitch << ", " << yaw << ", " << throttle << std::endl;
    _rc->queue_commands(throttle, roll, pitch, yaw, mode, time);

    ControlData c(Roll(), Throttle(), Pitch(), time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    (*_logger) << static_cast<int>(data_drone.pos_valid)  << ";" <<
               _flight_mode << ";" <<
               flight_mode_names[_flight_mode] << ";" <<
               data_target_new.pos().x << ";" <<
               data_target_new.pos().y  << ";" <<
               data_target_new.pos().z << ";" <<
               auto_throttle << ";" <<
               auto_roll << ";" <<
               auto_pitch << ";" <<
               auto_yaw <<  ";" <<
               joy_throttle <<  ";" <<
               joy_roll <<  ";" <<
               joy_pitch <<  ";" <<
               joy_yaw << ";" <<
               static_cast<int>(_joy_arm_switch) << ";" <<
               static_cast<int>(_joy_mode_switch) << ";" <<
               static_cast<int>(_joy_takeoff_switch) << ";" <<
               _rc->arm_switch << ";" <<
               _rc->mode << ";" <<
               data_drone.dt << ";" <<
               kiv_ctrl.active << ";" <<
               calibration.thrust << ";" <<
               pos_err_i.x << ";" << pos_err_i.y << ";" << pos_err_i.z << ";" <<
               _rc->telemetry.batt_cell_v  << ";" <<
               static_cast<int>(_rc->telemetry.rssi)  << ";" <<
               _rc->telemetry.arming_state  << ";";
}

std::tuple<int, int, float, Point3f, std::vector<StateData> > DroneController::calc_burn(StateData state_drone, StateData state_target, float remaining_aim_duration) {
    remaining_aim_duration +=  transmission_delay_duration + 1.f / pparams.fps;
    auto [auto_roll_burn, auto_pitch_burn, burn_duration, burn_direction] = calc_directional_burn(state_drone, state_target, remaining_aim_duration);
    auto traj = predict_trajectory(burn_duration, remaining_aim_duration, burn_direction, state_drone);
    return std::make_tuple(auto_roll_burn, auto_pitch_burn, burn_duration, burn_direction, traj);
}

std::tuple<int, int, float, cv::Point3f> DroneController::calc_directional_burn(StateData state_drone, StateData state_target, float remaining_aim_duration) {

    cv::Point3f target_pos_after_aim = state_target.pos + remaining_aim_duration * state_target.vel;

    //do an initial guess of the necessary burn direction and duration. Crude, but better algorithms are used during the iterative process
    cv::Point3f drone_pos_after_aim = state_drone.pos + remaining_aim_duration * state_drone.vel;
    cv::Point3f burn_dist = target_pos_after_aim - drone_pos_after_aim;
    float burn_duration = 5.f / pparams.fps; // initial guess: minimum burn time
    cv::Point3f drone_pos_after_burn, target_pos_after_burn, burn_direction;
    burn_direction = burn_dist / norm(burn_dist);
    float conversion_speed = 1;
    //iterative approximate the acceleration needed to hit the insect
    for (int i = 0; i < 100; i++) {
        cv::Point3f integrated_pos, integrated_vel;
        std::tie(integrated_pos, integrated_vel, std::ignore) = predict_drone_after_burn(
                    state_drone, burn_direction, remaining_aim_duration, burn_duration);

        drone_pos_after_burn = integrated_pos;
        target_pos_after_burn = target_pos_after_aim + burn_duration * state_target.vel;

        cv::Point3f pos_err = target_pos_after_burn - integrated_pos;
        burn_dist += pos_err;
        burn_direction = burn_dist / norm(burn_dist);

        double derr = norm(pos_err);
        if (derr < 0.01) { // if converged
            break;
        }

        //to determine whether the burn_duration needs to in/decrease we check if the pos after
        //the burn is moving away or towards the insect. This is done by checking if the angle between
        //the vel vector and the delta_pos vector is bigger then 90 degrees:
        if (integrated_vel.dot(target_pos_after_burn - integrated_pos) > 0)
            burn_duration += conversion_speed * static_cast<float>(norm(pos_err) / norm(integrated_vel));
        else
            burn_duration -= conversion_speed * static_cast<float>(norm(pos_err) / norm(integrated_vel));

        //in/decrease the conversion speed and retry if the loop converged to a wrong maximum:
        if (burn_duration < 0) {
            burn_duration = 1.0;
            conversion_speed *= 0.5f;
        } else if (burn_duration > 1) {
            burn_duration = 0.2f;
            conversion_speed *= 0.5f;
        }

        if (i == 20 && conversion_speed > 0.9f && conversion_speed  < 1.1f)
            conversion_speed *= 0.25f;

        if (i >= 99) {
            std::cout << "Warning: calc_directional_burn not converged!" << std::endl;
            burn_duration = 0.0;
            burn_direction = {0, 1, 0};
        }
    }

    burn_direction = lowest_direction_to_horizontal(burn_direction, min_takeoff_angle);

    auto [roll_quat, pitch_quat] = acc_to_quaternion(burn_direction);
    int roll_cmd =  roundf((roll_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE); // convert to RC commands range
    int pitch_cmd = roundf((-pitch_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE);

    return std::make_tuple(roll_cmd, pitch_cmd, burn_duration, burn_direction);
}

//calculate the predicted drone location after a burn
std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::predict_drone_after_burn(
    StateData state_drone, cv::Point3f burn_direction, float remaining_aim_duration, float burn_duration) {

    cv::Point3f integrated_pos = state_drone.pos;
    cv::Point3f integrated_vel = state_drone.vel;

    float partial_effective_burn_spin_up_duration = effective_burn_spin_up_duration;
    if (burn_duration  < effective_burn_spin_up_duration)
        partial_effective_burn_spin_up_duration = burn_duration;

    //state after aiming:
    if (remaining_aim_duration > 0) {
        cv::Point3f drone_acc_after_aim = burn_direction * GRAVITY; // thrust at then end of aim
        drone_acc_after_aim.y -= GRAVITY; // acc = thrust - gravity
        cv::Point3f acc_now = (aim_duration - remaining_aim_duration) / aim_duration *  drone_acc_after_aim; // acc now (somewhere during the aim), assuming linear acc
        cv::Point3f drone_acc_during_aim = 0.5f * (drone_acc_after_aim + acc_now); // average acc during aim time, assuming linear acc
        integrated_pos += 0.5f * drone_acc_during_aim * powf(remaining_aim_duration, 2) + integrated_vel * remaining_aim_duration;
        integrated_vel += drone_acc_during_aim * remaining_aim_duration;
    }

    cv::Point3f burn_accelleration = burn_direction * calibration.thrust;
    if (burn_duration  > effective_burn_spin_up_duration) {
        burn_accelleration -= cv::Point3f(0, GRAVITY, 0);
        // Phase: spin up
        integrated_pos += 0.25f * burn_accelleration * powf(effective_burn_spin_up_duration, 2) + integrated_vel * effective_burn_spin_up_duration;
        integrated_vel += 0.5f * burn_accelleration  * effective_burn_spin_up_duration;
        // Phase: max burn
        float t_mt = burn_duration - effective_burn_spin_up_duration;
        integrated_pos += 0.5f * burn_accelleration * powf(t_mt, 2) + integrated_vel * t_mt;
        integrated_vel += burn_accelleration * t_mt;
    } else {
        burn_accelleration = burn_accelleration * (partial_effective_burn_spin_up_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
        burn_accelleration -= cv::Point3f(0, GRAVITY, 0);

        // Phase: spin up
        integrated_pos += 0.25f * burn_accelleration * powf(partial_effective_burn_spin_up_duration, 2) + integrated_vel * partial_effective_burn_spin_up_duration;
        integrated_vel += 0.5f * burn_accelleration * partial_effective_burn_spin_up_duration;
    }
    return std::make_tuple(integrated_pos, integrated_vel, burn_accelleration);
}

void DroneController::draw_viz(
    StateData state_drone, StateData state_target, double time, cv::Point3f burn_direction,
    float burn_duration, float remaining_aim_duration, std::vector<StateData> traj) {

    double viz_time_after_aim = time + static_cast<double>(remaining_aim_duration + transmission_delay_duration + 1.f / pparams.fps);
    viz_target_pos_after_burn = state_target.pos + (remaining_aim_duration + burn_duration) * state_target.vel; //transmission_delay_duration + 1.f/pparams.fps ???
    if (_flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn) {
        cv::Point3f integrated_pos, integrated_vel;
        cv::Point3f burn_accelleration;
        std::tie(integrated_pos, integrated_vel, burn_accelleration) = predict_drone_after_burn(
                    state_drone, burn_direction, 0, burn_duration);

        viz_drone_pos_after_burn =  traj.back().pos;
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration);
        viz_trajectory = predict_trajectory(burn_duration, 0, burn_direction, state_drone);
    } else {
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration);
        viz_trajectory = traj;
        viz_drone_pos_after_burn =  traj.back().pos;
    }
}

bool check_att_bounds(cv::Point2f att, cv::Point2f att_min, cv::Point2f att_max) {
    bool att_check = att.x > att_min.x && att.x < att_max.x &&
                     att.y > att_min.y && att.y < att_max.y;
    return att_check;
}
void DroneController::calibrate_pad_attitude() {
    if (new_attitude_package_available()) {
        if (check_att_bounds(cv::Point2f(_rc->telemetry.roll, _rc->telemetry.pitch), cv::Point2f(-360, -360), cv::Point2f(360, 360))) {
            pad_att_calibration_roll.addSample(_rc->telemetry.roll);
            pad_att_calibration_pitch.addSample(_rc->telemetry.pitch);
        }
    }
}
bool DroneController::new_attitude_package_available() {
    static uint32_t prev_roll_pitch_package_id = 0;
    if (_rc->telemetry.roll_pitch_package_id > prev_roll_pitch_package_id) {
        prev_roll_pitch_package_id = _rc->telemetry.roll_pitch_package_id;
        return true;
    }
    return false;
}

bool DroneController::attitude_on_pad_OK() {
    static bool landing_att_calibration_msg_printed = false;
    if (pad_att_calibration_roll.ready()) {
        cv::Point2f current_att(pad_att_calibration_roll.latest(), pad_att_calibration_pitch.latest());
        cv::Point2f pad_att_calibration(calibration.pad_roll, calibration.pad_pitch);
        if (!landing_att_calibration_msg_printed)
            std::cout << "Received enough att telemetry samples from drone. Diff with original blink att: " << current_att - pad_att_calibration << std::endl;
        landing_att_calibration_msg_printed = true;
        return check_att_bounds(current_att,
                                pad_att_calibration - allowed_att_calibration_range,
                                pad_att_calibration + allowed_att_calibration_range);
    } else
        return false;
}

decision_type DroneController::somewhere_on_pad() {
    if(_rc->telemetry.roll<-180.f || _rc->telemetry.roll>180.f
       || _rc->telemetry.pitch<-180.f || _rc->telemetry.pitch>180.f)
        return maybe;
    cv::Point2f current_att(_rc->telemetry.roll, _rc->telemetry.pitch);
    cv::Point2f pad_att_calibration(calibration.pad_roll, calibration.pad_pitch);
    if(check_att_bounds(current_att,
                            pad_att_calibration - somewhere_on_pad_att_range,
                            pad_att_calibration + somewhere_on_pad_att_range))
        return yes;
    else
        return no;
}

void DroneController::invalidize_blink() {
    calibration.pad_calib_date = "2000/01/01 00:00:00";
    save_calibration();
}
bool DroneController::pad_calibration_done() {
    if (!dparams.Telemetry()) {
        std::cout << "Warning: drone has no telemetry capability. Skipping att calibration." << std::endl;
        return true;
    } else if (!pad_att_calibration_roll.ready()) {
        std::cout << "Drone calibration failed: attitude not received!" << std::endl;
        return false;
    } else if (check_att_bounds(cv::Point2f(pad_att_calibration_roll.latest(), pad_att_calibration_pitch.latest()), -allowed_pad_att_calibration_range, allowed_pad_att_calibration_range)) {
        calibration.pad_pos_x = _dtrk->pad_location().x;
        calibration.pad_pos_y = _dtrk->pad_location().y;
        calibration.pad_pos_z = _dtrk->pad_location().z;
        calibration.pad_roll = pad_att_calibration_roll.latest();
        calibration.pad_pitch = pad_att_calibration_pitch.latest();
        auto date = chrono::system_clock::to_time_t(chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&date), "%Y/%m/%d %T");
        calibration.pad_calib_date = ss.str();
        save_calibration();
        _dtrk->set_pad_location(calibration.pad_pos());
        _flight_area->update_bottom_plane_based_on_blink(calibration.pad_pos_y);
        kiv_ctrl.init(_flight_area->flight_area_config(relaxed), &calibration); // kiv can only be initialized after the latest (potential) adding of a plane (bottom_plane)
        return true;
    } else {
        std::cout << "Drone calibration failed: attitude not within accetable bounds: [" << pad_att_calibration_roll.latest() << ", " << pad_att_calibration_pitch.latest() << "]" << std::endl;
        return false;
    }
}

void DroneController::init_thrust_calibration() {
    calibration.thrust -= ki_thrust_hover * 0.001f * pos_err_i.y; //TODO: get the 0.001 from the scale constants
    pos_err_i.y = 0;
    thrust_calibration = true;
}
void DroneController::save_thrust_calibration() {
    auto date = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&date), "%Y/%m/%d %T");
    calibration.thrust_calib_date = ss.str();
    thrust_calibration = false;
    save_calibration();
}

std::vector<StateData> DroneController::predict_trajectory(float burn_duration, float remaining_aim_duration, cv::Point3f burn_direction, StateData state_drone) {
    std::vector<StateData> traj;
    cv::Point3f integrated_pos, integrated_vel;
    for (float f = 0; f <= burn_duration; f += 1.f / pparams.fps) {
        std::tie(integrated_pos, integrated_vel, std::ignore) = predict_drone_after_burn(
                    state_drone, burn_direction, remaining_aim_duration, f);
        StateData state;
        state.pos = integrated_pos;
        state.vel = integrated_vel;
        traj.push_back(state);
    }
    return traj;
}


std::tuple<float, float> DroneController::acc_to_deg(cv::Point3f acc) {
    float norm_burn_vector_XZ = sqrtf(powf(acc.x, 2) + powf(acc.z, 2));
    float rotation_angle = atan2f(norm_burn_vector_XZ, (acc.y)) * rad2deg;
    float roll = -acc.x / norm_burn_vector_XZ * rotation_angle;
    float pitch = -acc.z / norm_burn_vector_XZ * rotation_angle;

    roll = std::clamp(roll, -max_bank_angle, max_bank_angle);
    pitch = std::clamp(pitch, -max_bank_angle, max_bank_angle);
    return std::make_tuple(roll, pitch);
}

std::tuple<float, float> DroneController::acc_to_quaternion(cv::Point3f acc) {
    cv::Point3f acc_BF = {-acc.z, -acc.x, -acc.y};
    cv::Point3f acc_BF_hover = {0, 0, -1};
    Quaternion q = rot_quat(acc_BF_hover, acc_BF);
    return std::make_tuple(q.v.x, q.v.y);
}

//considering a take off order from just after the aiming phase of: spinning up, burning, spin down, 1g, find the max drone acceleration that best desccribes the current position given dt
void DroneController::approx_effective_thrust(TrackData data_drone, cv::Point3f burn_direction, float burn_duration, float dt_burn) {

    cv::Point3f pos_after_aim =  _dtrk->pad_location() + cv::Point3f(0, lift_off_dist_take_off_aim, 0);
    float partial_effective_burn_spin_up_duration = effective_burn_spin_down_duration; // if the burn duration is long enough this is equal otherwise it may be shortened
    cv::Point3f acc = burn_direction * calibration.thrust ; // initial guess, variable to be optimized
    cv::Point3f meas_pos =  data_drone.pos(); // current measured drone position
    cv::Point3f integrated_vel = {0};
    for (int i = 0; i < 100; i++) {

        cv::Point3f max_acc = acc;
        cv::Point3f integrated_pos = pos_after_aim;
        cv::Point3f max_jerk; // = acc/effective_burn_spin_up_duration;

        if (burn_duration  > effective_burn_spin_up_duration) {
            max_acc -= cv::Point3f(0, GRAVITY, 0);
            max_jerk = max_acc / effective_burn_spin_up_duration;
            // Phase: spin up
            // using jerk  x=x0+v0t+1/2a0*t^2+1/6j*t^3 v=v0+a0t+1/2j*t^2
            integrated_pos += 0.167f * max_jerk * powf(effective_burn_spin_up_duration, 3);
            integrated_vel = 0.5f * max_jerk  * powf(effective_burn_spin_up_duration, 2);

            // Phase: max burn
            float t_mt = burn_duration - effective_burn_spin_up_duration;
            integrated_pos += 0.5f * max_acc * powf(t_mt, 2) + integrated_vel * t_mt;
            integrated_vel += max_acc * t_mt;

        } else {
            partial_effective_burn_spin_up_duration = burn_duration;
            max_acc = acc * (burn_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
            max_acc -= cv::Point3f(0, GRAVITY, 0);
            max_jerk = max_acc / effective_burn_spin_up_duration;

            // Phase: spin up
            // using jerk  x=x0+v0t+1/2a0*t^2+1/6j*t^3 v=v0+a0t+1/2j*t^2
            integrated_pos += 0.167f * max_jerk * powf(partial_effective_burn_spin_up_duration, 3);
            integrated_vel = 0.5f * max_jerk  * powf(partial_effective_burn_spin_up_duration, 2);
        }

        float partial_effective_burn_spin_down_duration = dt_burn - burn_duration;
        if (partial_effective_burn_spin_down_duration > 0) {
            // Phase: spin down
            integrated_pos += 0.167f * max_jerk * powf(effective_burn_spin_down_duration, 3) + integrated_vel * effective_burn_spin_down_duration;
            integrated_vel += 0.5f * max_jerk  * powf(effective_burn_spin_down_duration, 2) ;
        } else
            std::cout << "Error: not implemented" << std::endl;

        // Phase: 1G: (the remaining time)
        integrated_pos += integrated_vel * (dt_burn - burn_duration - partial_effective_burn_spin_up_duration);

        cv::Point3f err = meas_pos - integrated_pos;
        acc += err / powf(dt_burn, 2);
        if (norm(err) < 0.01)
            break;

    }
    burn_thrust = static_cast<float>(norm(acc));
    vel_after_takeoff = integrated_vel * 1.15f; // add some magical battery healt factor. Better ideas welcome

    std::cout << "Estimated burn thrust: " << burn_thrust << " vs calibrated thrust: " << calibration.thrust << std::endl;
}

float DroneController::thrust_to_throttle(float thrust_ratio) {
    float p1 = 3433.2;
    float p2 = -4833.5;
    float p3 = 1838.6;
    float p4 = 1011.6;
    float p5 = 355.45;
    return p1 * powf(thrust_ratio, 4) + p2 * powf(thrust_ratio, 3) + p3 * powf(thrust_ratio, 2) + p4 * thrust_ratio + p5;
}


std::tuple<int, int, int> DroneController::calc_feedforward_control(cv::Point3f desired_acc) {

    cv::Point3f des_acc_drone = compensate_gravity_and_crop_to_limit(desired_acc, calibration.thrust);
    cv::Point3f direction = des_acc_drone / normf(des_acc_drone);

    float throttlef = normf(des_acc_drone) / calibration.thrust;
    int throttle_cmd =  static_cast<uint16_t>(roundf(thrust_to_throttle(throttlef)));

    if (throttle_cmd < dparams.min_throttle)
        throttle_cmd = dparams.min_throttle;

    auto [roll_quat, pitch_quat] = acc_to_quaternion(direction);
    int roll_cmd =  roundf((roll_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE); // convert to RC commands range
    int pitch_cmd = roundf((-pitch_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE);
    return std::make_tuple(roll_cmd, pitch_cmd, throttle_cmd);
}


cv::Point3f DroneController::compensate_gravity_and_crop_to_limit(cv::Point3f des_acc, float thrust) {
    // Calculate safe acceleration (max desired acceleration which cannot the
    // thrust constraints if it gravity compensated) for case in the further calculation
    // something goes wrong
    cv::Point3f acc_backup = des_acc / normf(des_acc) * (thrust - GRAVITY) + cv::Point3f(0, GRAVITY, 0);

    cv::Point3f req_acc = des_acc + cv::Point3f(0, GRAVITY, 0);

    if (normf(req_acc) < thrust)
        return req_acc;

    if (normf(des_acc) > thrust)
        des_acc = des_acc * thrust / normf(des_acc);

    float denominator = des_acc.dot(des_acc);
    float p = 2 * des_acc.y * GRAVITY / denominator;
    float q = (powf(GRAVITY, 2) - powf(thrust, 2)) / denominator;

    float root_square = powf(p / 2.f, 2) - q;
    if (root_square < 0.f) {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration." << std::endl;
        return acc_backup;
    }

    float k1 = -p / 2.f + sqrtf(root_square);
    float k2 = -p / 2.f - sqrtf(root_square);

    // Avoid warnings based on rounding errors
    if (k1 > 1.f && k1 < 1.05f)
        k1 = 1.;
    if (k2 > 1.f && k2 < 1.05f)
        k2 = 1.;

    bool k1_valid = 0 <= k1 && k1 <= 1;
    bool k2_valid = 0 <= k2 && k2 <= 1;

    if (k1_valid && !k2_valid) {
        return k1 * des_acc + cv::Point3f(0, GRAVITY, 0);
    } else if (!k1_valid && k2_valid) {
        return k2 * des_acc + cv::Point3f(0, GRAVITY, 0);
    } else if (k1_valid && k2_valid) {
        if (k1 >= k2) {
            return k1 * des_acc + cv::Point3f(0, GRAVITY, 0);
        } else {
            return k2 * des_acc + cv::Point3f(0, GRAVITY, 0);
        }
    } else if (!k1_valid && !k2_valid) {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration." << std::endl;
        return acc_backup;
    }
    else {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration." << std::endl;
        return acc_backup;
    }

}


void DroneController::control_model_based(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel) {

    if (_dtrk->image_predict_item().out_of_image) {
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        auto_throttle = spinup_throttle();
        return;
    }
    cv::Point3f desired_acc = pid_error(data_drone, setpoint_pos, setpoint_vel, false);
    std::tie(auto_roll, auto_pitch, auto_throttle) = calc_feedforward_control(desired_acc);
}

cv::Point3f DroneController::pid_error(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, bool dry_run) {
    //WARNING: this function is not allowed to store any information (or apply filters that store),
    //because it is also being used for dummy calculations in the interceptor! See also the dry_run variable
    integrator_state enable_horizontal_integrators = hold;

    if (!dry_run) {
        pos_modelx.new_sample(setpoint_pos.x);
        pos_modely.new_sample(setpoint_pos.y);
        pos_modelz.new_sample(setpoint_pos.z);
        enable_horizontal_integrators = horizontal_integrators(setpoint_vel, data_drone.time);
    }


    auto [kp_pos, ki_pos, kd_pos] = adjust_control_gains(data_drone, enable_horizontal_integrators);
    auto [pos_err_p, pos_err_d] = control_error(data_drone, setpoint_pos, enable_horizontal_integrators, dry_run);

    cv::Point3f error = {0};
    error += multf(kp_pos, pos_err_p) + multf(ki_pos, pos_err_i) + multf(kd_pos, pos_err_d); // position controld

    bool flight_mode_with_kiv = _flight_mode == fm_flying_pid || _flight_mode == fm_reset_headless_yaw || _flight_mode == fm_correct_yaw;

    if (data_drone.pos_valid && data_drone.vel_valid && !dry_run)
        error += kiv_ctrl.update(data_drone, transmission_delay_duration, flight_mode_with_kiv && !_time - start_takeoff_burn_time < 0.45);

    return error;
}

integrator_state DroneController::horizontal_integrators(cv::Point3f setpoint_vel, double time) {
    float duration_waypoint_update = duration_since_waypoint_moved(time);

    if (thrust_calibration
            || (_flight_mode == fm_headed && duration_waypoint_update > 1)
            || (normf(setpoint_vel) < 0.01f && duration_waypoint_update > 2))
        return running;
    else if (_flight_mode == fm_headed && duration_waypoint_update <= 1)
        return hold;
    else
        return reset;
}

std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::adjust_control_gains(TrackData data_drone, bool enable_horizontal_integrators) {
    float kp_pos_roll_scaled, kp_pos_throttle_scaled, kp_pos_pitch_scaled, ki_pos_roll_scaled, ki_thrust_scaled, ki_pos_pitch_scaled, kd_pos_roll_scaled, kd_pos_throttle_scaled, kd_pos_pitch_scaled, kp_v_roll_scaled, kp_v_throttle_scaled, kp_v_pitch_scaled, kd_v_roll_scaled, kd_v_throttle_scaled, kd_v_pitch_scaled;

    cv::Point3f scale_pos_p = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_pos_i = {0.001f, 0.001f, 0.001f};
    cv::Point3f scale_pos_d = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_vel_p = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_vel_d = {0.01f, 0.01f, 0.01f};

    if (!enable_horizontal_integrators) {
        scale_pos_i.x = 0;
        scale_pos_i.z = 0;
    }

    if (_hover_mode) {
        kp_pos_roll_scaled = scale_pos_p.x * kp_pos_roll_hover;
        kp_pos_throttle_scaled = scale_pos_p.y * kp_pos_throttle_hover;
        kp_pos_pitch_scaled = scale_pos_p.z * kp_pos_pitch_hover;
        ki_pos_roll_scaled = scale_pos_i.x * ki_pos_roll_hover;
        ki_thrust_scaled = scale_pos_i.y * ki_thrust_hover;
        ki_pos_pitch_scaled = scale_pos_i.z * ki_pos_pitch_hover;
        kd_pos_roll_scaled = scale_pos_d.x * kd_pos_roll_hover;
        kd_pos_throttle_scaled = scale_pos_d.y * kd_pos_throttle_hover;
        kd_pos_pitch_scaled = scale_pos_d.z * kd_pos_pitch_hover;
        kp_v_roll_scaled = 0;
        kp_v_throttle_scaled = 0;
        kp_v_pitch_scaled = 0;
        kd_v_roll_scaled = 0;
        kd_v_throttle_scaled = 0;
        kd_v_pitch_scaled = 0;
    } else {
        if (!data_drone.pos_valid) {
            scale_pos_p *= 0.8;
        } else if(norm(data_drone.vel())>2) { //Our tuning is such that the drone is aggressiv which includes overshoots. To limit the overshoot after high speed increase D at high speeds.
            scale_pos_d *= 1.2;
        }
        kp_pos_roll_scaled = scale_pos_p.x * kp_pos_roll;
        kp_pos_throttle_scaled = scale_pos_p.y * kp_pos_throttle;
        kp_pos_pitch_scaled = scale_pos_p.z * kp_pos_pitch;
        ki_pos_roll_scaled = scale_pos_i.x * ki_pos_roll;
        ki_thrust_scaled = scale_pos_i.y * ki_thrust;
        ki_pos_pitch_scaled = scale_pos_i.z * ki_pos_pitch;
        kd_pos_roll_scaled = scale_pos_d.x * kd_pos_roll;
        kd_pos_throttle_scaled = scale_pos_d.y * kd_pos_throttle;
        kd_pos_pitch_scaled = scale_pos_d.z * kd_pos_pitch;
        kp_v_roll_scaled = scale_vel_p.x * kp_v_roll;
        kp_v_throttle_scaled = scale_vel_p.x * kp_v_throttle;
        kp_v_pitch_scaled = scale_vel_p.x * kp_v_pitch;
        kd_v_roll_scaled = scale_vel_d.x * kd_v_roll;
        kd_v_throttle_scaled = scale_vel_d.x * kd_v_throttle;
        kd_v_pitch_scaled = scale_vel_d.x * kd_v_pitch;
    }

    // Arrange gains (needed because may be updated from trackbars)
    cv::Point3f kp_pos(kp_pos_roll_scaled, kp_pos_throttle_scaled, kp_pos_pitch_scaled);
    cv::Point3f kd_pos(kd_pos_roll_scaled, kd_pos_throttle_scaled, kd_pos_pitch_scaled);
    cv::Point3f ki_pos(ki_pos_roll_scaled, ki_thrust_scaled, ki_pos_pitch_scaled);
    cv::Point3f kp_vel(kp_v_roll_scaled, kp_v_throttle_scaled, kp_v_pitch_scaled);
    cv::Point3f kd_vel(kd_v_roll_scaled, kd_v_throttle_scaled, kd_v_pitch_scaled);

    return std::tuple(kp_pos, ki_pos, kd_pos);
}

std::tuple<cv::Point3f, cv::Point3f> DroneController::control_error(TrackData data_drone, cv::Point3f setpoint_pos, integrator_state enable_horizontal_integrators, bool dry_run) {
    //WARNING: this function is not allowed to store any information (or apply filters that store), because it is also being used for dummy calculations in the interceptor!

    float err_x_filtered = 0, err_y_filtered = 0, err_z_filtered = 0;
    if (data_drone.pos_valid) {
        err_x_filtered = setpoint_pos.x - data_drone.state.spos.x;
        err_y_filtered = setpoint_pos.y - data_drone.state.spos.y;
        err_z_filtered = setpoint_pos.z - data_drone.state.spos.z;
    }
    cv::Point3f pos_err_p = {err_x_filtered, err_y_filtered, err_z_filtered};

    float errDx = 0, errDy = 0, errDz = 0;
    if (data_drone.vel_valid) {
        errDx = -data_drone.state.vel.x;
        errDy = -data_drone.state.vel.y;
        errDz = -data_drone.state.vel.z;
    }

    if (_flight_mode == fm_long_range_back || _flight_mode == fm_long_range_forth) {
        err_z_filtered = 0;
        pos_err_p.z = 0;
        errDz = 0;
    } else if (data_drone.world_item.distance > 1) { // reduce sensitivity at longer distances because the disparity gets whacky
        err_z_filtered /= 1 + 0.1f * (data_drone.world_item.distance * data_drone.world_item.distance);
        pos_err_p.z = err_z_filtered;
        errDz /= 1 + 0.1f * (data_drone.world_item.distance * data_drone.world_item.distance);
    } else  if (data_drone.world_item.distance > 10) {
        err_z_filtered = 0.4f;
        pos_err_p.z = err_z_filtered;
        errDz = 0;
    }

    cv::Point3f pos_err_d = {errDx, errDy, errDz};

    if (enable_horizontal_integrators == running) {
        pos_err_i.x += (err_x_filtered - setpoint_pos.x + pos_modelx.current_output());
        pos_err_i.z += (err_z_filtered - setpoint_pos.z + pos_modelz.current_output());
        if (thrust_calibration) {
            calibration.thrust -= 0.1f * ((err_y_filtered - setpoint_pos.y + pos_modely.current_output()));
            pos_err_i.y = 0;
        } else
            pos_err_i.y += (err_y_filtered - setpoint_pos.y + pos_modely.current_output());
    } else if (enable_horizontal_integrators == reset && !dry_run) {
        pos_err_i = {0};
    }

    return std::tuple(pos_err_p, pos_err_d);
}

void DroneController::correct_yaw(float deviation_angle) {
    int target_theta_velocity = 0;

    if (deviation_angle < - _dtrk->min_yaw_deviation)
        target_theta_velocity = -3;
    else if (deviation_angle > _dtrk->min_yaw_deviation)
        target_theta_velocity = 3;

    auto_yaw = RC_MIDDLE + target_theta_velocity;
}

void DroneController::blink(double time) {
    static double last_blink_time = time;
    static bool blink_state;

    if (static_cast<float>(time - last_blink_time) > dparams.blink_period) {
        if (blink_state)
            blink_state = false;
        else
            blink_state = true;
        last_blink_time = time;
    }
    LED(blink_state);
}

void DroneController::blink_motors(double time) {
    int itime = round(time) * 3;
    auto_roll = RC_BOUND_MIN;
    auto_pitch = RC_BOUND_MIN;
    auto_throttle = RC_BOUND_MIN;
    auto_yaw = RC_BOUND_MIN;

    int spin_value = RC_BOUND_MIN + 100;
    if (itime % 4 == 0) {
        auto_roll = spin_value;
    } else if (itime % 4 == 1) {
        auto_pitch = spin_value;
    } else if (itime % 4 == 2) {
        auto_yaw = spin_value;
    } else if (itime % 4 == 3) {
        auto_throttle = spin_value;
    }
}

void DroneController::read_joystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (pparams.joystick == rc_devo) {
                switch (event.number) {
                    case 0: // roll
                        joy_roll = (event.value >> 5) + RC_MIDDLE;
                        break;
                    case 1: // pitch
                        joy_pitch = (event.value >> 5) + RC_MIDDLE;
                        break;
                    case 2: //throttle
                        joy_throttle = (event.value >> 5) + RC_MIDDLE - 100;
                        break;
                    case 5:  //switch
                        if (event.value > 0) {
                            _joy_arm_switch = bf_armed;
                        } else {
                            _joy_arm_switch = bf_disarmed;
                        }
                        break;
                    case 3: //dial
                        joyDial = event.value; // goes between +/-32768
                        scaledjoydial = joyDial + 32767;
                        scaledjoydial = (scaledjoydial / 65536) * 100 + 35;
                        break;
                    case 4: //yaw
                        joy_yaw = (event.value >> 5) + RC_MIDDLE;
                        break;
                    default:
                        std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                        break;
                }
            }  else if (pparams.joystick == rc_xlite) {
                switch (event.number) {
                    case 0: // roll
                        joy_roll = static_cast<uint16_t>((event.value >> 5) * 0.8 + RC_MIDDLE);
                        break;
                    case 1: // pitch
                        joy_pitch = static_cast<uint16_t>((event.value >> 5) * 0.8 + RC_MIDDLE);
                        break;
                    case 2: //throttle
                        joy_throttle =  static_cast<uint16_t>((event.value >> 5) * 0.8 + RC_MIDDLE);
                        break;
                    case 3: //yaw
                        joy_yaw = static_cast<uint16_t>((event.value >> 5) * 0.4 + RC_MIDDLE);
                        break;
                    case 4: //arm switch (two way)
                        if (event.value > 0) {
                            _joy_arm_switch = bf_armed;
                        } else {
                            _joy_arm_switch = bf_disarmed;
                        }
                        break;
                    case 5: //mode switch (3 way)
                        if (event.value < -16384) {
                            _joy_mode_switch = jmsm_manual;
                        } else if (event.value > 16384) {
                            _joy_mode_switch = jmsm_hunt;
                        } else {
                            _joy_mode_switch = jmsm_waypoint;
                        }
                        break;
                    case 6: //switch (3 way)
                        _joy_takeoff_switch = event.value > 0; //fix for weird x-lite channel problem https://github.com/pats-drones/pats/issues/93
                        break;
                    case 7: //switch (2 way)
                        _joy_takeoff_switch = event.value > 0;
                        break;
                    default:
                        //this joystick seems to have 16 extra buttons... weird whatever
                        //                std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                        break;
                }
            }
        }
    }
}

void DroneController::process_joystick() {

    //check the demo flightplan trigger, which overrides the joystick_takeoff_switch (even though there was no joystick connected)
    if (log_replay_mode && pparams.joystick == rc_none && _joy_takeoff_switch == true)
        _manual_override_take_off_now = true;
    else
        _manual_override_take_off_now = false;

    if (pparams.joystick == rc_none || _joy_state == js_none)
        return;

    // prevent accidental take offs at start up
    if (_joy_state == js_checking) {
        if (_joy_arm_switch == bf_disarmed &&
                ((joy_throttle <= RC_MIN_THRESH && !dparams.mode3d) || (abs(joy_throttle - RC_MIDDLE) < 50 && dparams.mode3d)) &&
                !_joy_takeoff_switch) {
            _flight_mode = fm_disarmed;
            _joy_state = js_disarmed;
        } else {
            _flight_mode = fm_joystick_check;
        }
    } else {

        if (_flight_mode == fm_inactive) {
            _manual_override_take_off_now = _joy_takeoff_switch;
        }
        if (_joy_arm_switch == bf_disarmed) {
            if (!dparams.mode3d)
                _rc->arm(bf_disarmed);
            _joy_takeoff_switch = false;
            _manual_override_take_off_now = false;
            _joy_state = js_disarmed;
            if (_flight_mode != fm_blink)
                _flight_mode = fm_disarmed;
        } else {
            if (!dparams.mode3d)
                _rc->arm(bf_armed);
            if (_joy_mode_switch == jmsm_manual) {
                _joy_state = js_manual;
                _flight_mode = fm_manual;
            } else if (_joy_mode_switch == jmsm_waypoint)
                _joy_state = js_waypoint;
            else if (_joy_mode_switch == jmsm_hunt)
                _joy_state = js_hunt;
        }
    }
}

void DroneController::load_calibration(std::string replay_dir) {
    std::string calib_wfn = "./logging/initial_" + calib_fn;
    std::string calib_rfn = "../xml/" + calib_fn;
    if (replay_dir != "") {
        calib_rfn = replay_dir + "/initial_" + calib_fn;
        calib_wfn = "./logging/replay/initial_" + calib_fn;
    }

    if (file_exist(calib_rfn)) {
        xmls::DroneCalibration tmp;
        try {
            tmp.deserialize(calib_rfn);
        } catch (MyExit const &err) {
            std::cout << "Error, corrupted drone calibration xml:" << err.msg << std::endl;
        }
        if ((tmp.drone_id != _rc->drone_id() || tmp.drone_name.compare(dparams.name)) && replay_dir == "") {
            std::cout << "Drone mismatch. Using default values, but calibration is needed!" << std::endl;
            calibration.thrust = dparams.default_thrust;
            calibration.drone_id = _rc->drone_id();
            calibration.drone_name = dparams.name;
        } else {
            calibration.deserialize(calib_rfn);
        }
    } else {
        std::cout << "Drone calibration missing. Using default values, but calibration is needed!" << std::endl;
        calibration.thrust = dparams.default_thrust;
        calibration.drone_id = _rc->drone_id();
        calibration.drone_name = dparams.name;
    }
    calibration.serialize(calib_wfn);

    _dtrk->set_pad_location(calibration.pad_pos());
    _flight_area->update_bottom_plane_based_on_blink(calibration.pad_pos_y);
    kiv_ctrl.init(_flight_area->flight_area_config(relaxed), &calibration);// kiv can only be initialized after the latest (potential) adding of a plane (bottom_plane)

}

void DroneController::save_calibration() {
    if (log_replay_mode)
        calibration.serialize("./logging/replay/" + calib_fn);
    else {
        calibration.serialize("../xml/" + calib_fn);
        calibration.serialize("./logging/" + calib_fn);
    }
}
bool DroneController::takeoff_calib_valid() {
    if (!calibration.pad_calib_date.length())
        return false;
    if (log_replay_mode)
        return true;
    std::stringstream ss;
    ss << calibration.pad_calib_date;
    std::tm t = {};
    ss >> std::get_time(&t, "%Y/%m/%d %T");
    auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    double  diff = std::difftime(now, std::mktime(&t)) / (60. * 60. * 24.);
    return diff < 0.8;
}
bool DroneController::thrust_calib_valid() {
    if (!calibration.thrust_calib_date.length())
        return false;
    if (log_replay_mode)
        return true;
    std::stringstream ss;
    ss << calibration.thrust_calib_date;
    std::tm t = {};
    ss >> std::get_time(&t, "%Y/%m/%d %T");
    auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    double  diff = std::difftime(now, std::mktime(&t)) / (60. * 60. * 24.);
    return diff < 7;
}

void DroneController::load_control_parameters() {
    control_parameters_rfn = "../xml/" + dparams.control + ".xml";
    std::cout << "Reading settings from: " << control_parameters_rfn << std::endl;
    ControlParameters params;
    if (file_exist(control_parameters_rfn)) {
        std::ifstream infile(control_parameters_rfn);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
            throw MyExit("Cannot read: " + control_parameters_rfn);

        ControlParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw MyExit("XML version difference detected from " + control_parameters_rfn);
        }
        infile.close();
    } else {
        throw MyExit("File not found: " + control_parameters_rfn);
    }

    kp_pos_roll = params.kp_pos_roll.value();
    kp_pos_pitch = params.kp_pos_pitch.value();
    kp_pos_throttle = params.kp_pos_throttle.value();
    ki_pos_roll = params.ki_pos_roll.value();
    ki_pos_pitch = params.ki_pos_pitch.value();
    ki_thrust = params.ki_thrust.value();
    kd_pos_roll = params.kd_pos_roll.value();
    kd_pos_pitch = params.kd_pos_pitch.value();
    kd_pos_throttle = params.kd_pos_throttle.value();
    kp_pos_roll_hover = params.kp_pos_roll_hover.value();
    kp_pos_pitch_hover = params.kp_pos_pitch_hover.value();
    kp_pos_throttle_hover = params.kp_pos_throttle_hover.value();
    ki_pos_roll_hover = params.ki_pos_roll_hover.value();
    ki_pos_pitch_hover = params.ki_pos_pitch_hover.value();
    ki_thrust_hover = params.ki_thrust_hover.value();
    kd_pos_roll_hover = params.kd_pos_roll_hover.value();
    kd_pos_pitch_hover = params.kd_pos_pitch_hover.value();
    kd_pos_throttle_hover = params.kd_pos_throttle_hover.value();

    kp_v_roll = params.kp_v_roll.value();
    kp_v_pitch = params.kp_v_pitch.value();
    kp_v_throttle = params.kp_v_throttle.value();
    kd_v_roll = params.kd_v_roll.value();
    kd_v_pitch = params.kd_v_pitch.value();
    kd_v_throttle = params.kd_v_throttle.value();
}

void DroneController::serialize_settings() {
    ControlParameters params;
    params.kp_pos_roll = kp_pos_roll;
    params.kp_pos_pitch = kp_pos_pitch;
    params.kp_pos_throttle = kp_pos_throttle;
    params.ki_pos_roll = ki_pos_roll;
    params.ki_pos_pitch = ki_pos_pitch;
    params.ki_thrust = ki_thrust;
    params.kd_pos_roll = kd_pos_roll;
    params.kd_pos_pitch = kd_pos_pitch;
    params.kd_pos_throttle = kd_pos_throttle;
    params.kp_pos_roll_hover = kp_pos_roll_hover;
    params.kp_pos_pitch_hover = kp_pos_pitch_hover;
    params.kp_pos_throttle_hover = kp_pos_throttle_hover;
    params.ki_pos_roll_hover = ki_pos_roll_hover;
    params.ki_pos_pitch_hover = ki_pos_pitch_hover;
    params.ki_thrust_hover = ki_thrust_hover;
    params.kd_pos_roll_hover = kd_pos_roll_hover;
    params.kd_pos_pitch_hover = kd_pos_pitch_hover;
    params.kd_pos_throttle_hover = kd_pos_throttle_hover;
    params.kp_v_roll = kp_v_roll;
    params.kp_v_pitch = kp_v_pitch;
    params.kp_v_throttle = kp_v_throttle;
    params.kd_v_roll = kd_v_roll;
    params.kd_v_pitch = kd_v_pitch;
    params.kd_v_throttle = kd_v_throttle;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream(control_parameters_rfn);
    outfile << xmlData ;
    outfile.close();
}

void DroneController::close() {
    if (initialized) {
        LED(true);
        beep(false);
        std::cout << "Closing controller." << std::endl;
        if (pparams.control_tuning)
            serialize_settings();
        initialized = false;
    }
}
