#include "dronecontroller.h"
#include "rc.h"
#include "flightplan.h"
#include "quaternion.h"
#include "linalg.h"

#include <experimental/filesystem>

using namespace cv;
using namespace tracking;

// Create an instance of Joystick
static Joystick joystick("/dev/input/js0");
static JoystickEvent event;

bool DroneController::joystick_ready() {
    return joystick.isFound();
}

void DroneController::init(RC *rc, tracking::DroneTracker *dtrk, FlightArea *flight_area) {
    _rc = rc;
    _dtrk = dtrk;
    _flight_area = flight_area;
    control_history_max_size = pparams.fps;
    control_mode_hold_filter.init(.25);

    std::cout << "Initialising control." << std::endl;

    load_calibration();
    calibration.serialize(data_output_dir + "/initial_drone_calibration.xml");

    initialized = true;
}

void DroneController::init_flight(std::ofstream *logger, int flight_id) {
    _logger = logger;
    (*_logger) << "flight_mode_str;" <<
               "tracking_valid;" <<
               "posX_target;posY_target;posZ_target;" <<
               "accX_target;accY_target;accZ_target;" <<
               "accX_commanded;accY_commanded;accZ_commanded;" <<
               "auto_throttle;auto_roll;auto_pitch;auto_yaw;" <<
               "joy_throttle;joy_roll;joy_pitch;joy_yaw; " <<
               "joy_arm_switch;joy_mode_switch;joy_takeoff_switch;" <<
               "mm_arm_switch;mm_mode_switch;" <<
               "kiv;" <<
               "max_thrust; integrator_x;integrator_y;integrator_z;" <<
               "batt_cell_v;rssi;";


    save_calibration_before_flight(flight_id);

    d_vel_err_x.init(1.f / pparams.fps);
    d_vel_err_y.init(1.f / pparams.fps);
    d_vel_err_z.init(1.f / pparams.fps);
    pos_err_i = {0};
    pos_modelx.init(1.f / pparams.fps, 1, 0.2, 0.38);
    pos_modely.init(1.f / pparams.fps, 1, 0.2, 0.38);
    pos_modelz.init(1.f / pparams.fps, 1, 0.2, 0.38);
    calibration.max_thrust = std::clamp(calibration.max_thrust, dparams.max_thrust / 2.f, dparams.max_thrust * 2.f);

    _landed = false;
    _hover_mode = false;

    take_off_start_time = 0;
    state_start_time = 0;
    remaining_spinup_duration_t0 = 0;
    interception_start_time = 0;
    in_flight_start_time = -1;
    ff_land_start_time = 0;
    ff_auto_throttle_start = RC_BOUND_MIN;

    _dist_to_setpoint = 999;
    time_waypoint_moved = 0;
    kill_cnt_down = 0;
    spin_up_start_time = 0;
    start_takeoff_burn_time = 0;
    _shake_finished = false;
    auto_burn_duration = 0;
    _manual_override_take_off_now = false;

    initial_hover_throttle_guess_non3d = GRAVITY / dparams.max_thrust * RC_BOUND_RANGE + dparams.min_throttle;
}

void DroneController::init_full_log_replay(std::string replay_dir) {
    log_replay_mode = true;
    calib_rfn = replay_dir + "/" +  std::experimental::filesystem::path(calib_fn).replace_filename("initial_drone_calibration.xml").string();
    load_calibration();
    calibration.serialize(data_output_dir + "/initial_drone_calibration.xml");
}
void DroneController::init_flight_replay(std::string replay_dir, int flight_id) {
    log_replay_mode = true;
    calib_rfn = replay_dir + "/" +  std::experimental::filesystem::path(calib_fn).replace_filename("drone_calibration_flight" + to_string(flight_id) + ".xml").string();
    load_calibration();
    calibration.serialize(data_output_dir + "/initial_drone_calibration.xml");
}

void DroneController::led_strength(float light_level) {
    if (dparams.led_type == led_fiber_uv || dparams.led_type == led_top_uv)
        dparams.drone_led_strength = 100;
    else {
        const float max_light_level = 0.7f;
        const float mid_light_level = 0.4f;

        if (light_level > max_light_level)
            dparams.drone_led_strength = 100;
        else if (light_level > mid_light_level)
            dparams.drone_led_strength = 50 + (light_level - mid_light_level) / (max_light_level - mid_light_level) * 50.f;
        else
            dparams.drone_led_strength =  std::clamp(light_level  / mid_light_level * 50.f, 5.f, 50.f);
    }
    std::cout << "Led strength set to: " << dparams.drone_led_strength << " with light level: " << light_level << std::endl;
}

void DroneController::control(TrackData data_drone, TrackData data_target, control_modes control_mode, cv::Point3f target_acceleration, double time, bool enable_logging) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_control = std::chrono::high_resolution_clock::now();
#endif
    _time = time;
    control_mode_hold_filter.update(control_mode, time);

    if (!log_replay_mode && pparams.joystick != rc_none)
        read_joystick();
    process_joystick();

    if (control_mode == acceleration_feedforward)
        _dtrk->update_target(data_drone.pos() + target_acceleration);
    else
        _dtrk->update_target(data_target.pos());

    _dist_to_setpoint = normf(data_drone.state.pos - data_target.state.pos);

    int throttle = 0, roll = 0, pitch = 0, yaw = 0, mode = RC_BOUND_MIN;
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
        } case fm_init_spinup: {
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
                remaining_spinup_duration_t0 = max(0.f, dparams.full_bat_and_throttle_spinup_duration - time_spent_spinning_up(time));
                _flight_mode = fm_remaining_spinup;
                auto_throttle = spinup_throttle();
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;

                if (dparams.mode3d) {
                    _rc->arm(bf_armed);
                    break;
                }
                [[fallthrough]];
        } case fm_remaining_spinup: {
                mode += bf_airmode;
                auto_throttle = spinup_throttle();
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;

                float spinup_time = static_cast<float>(time - take_off_start_time);

                if (spinup_time > remaining_spinup_duration_t0) {
                    auto_burn_duration = dparams.target_takeoff_velocity / calibration.max_thrust;
                    spin_up_start_time = 0;
                    _flight_mode = fm_max_burn;
                    start_takeoff_burn_time = time;
                }
                break;
        } case fm_max_burn: {
                mode += bf_airmode;
                auto_throttle = RC_BOUND_MAX;
                if (data_drone.vel_valid && data_drone.pos().y > _dtrk->pad_location().y + land_ctrl.trusted_tracking_height_above_pad()) {
                    _flight_mode = fm_flying_pid_init;
                }
                else if (static_cast<float>(time - start_takeoff_burn_time) >  auto_burn_duration + transmission_delay_duration) {
                    std::tie(auto_roll, auto_pitch, auto_throttle) = drone_commands({0, takeoff_aim_acceleration_factor * GRAVITY, 0});
                    _flight_mode = fm_1g;
                } else {
                    auto_roll = RC_MIDDLE;
                    auto_pitch = RC_MIDDLE;
                    auto_yaw = RC_MIDDLE;
                    auto_throttle = calibration.max_thrust;
                }
                break;
        } case fm_max_burn_spin_down: {
                mode += bf_airmode;
                std::tie(auto_roll, auto_pitch, auto_throttle) = drone_commands({0, 0, 0});
                if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration)
                    _flight_mode = fm_1g;
                break;
        }  case fm_1g: {
                mode += bf_airmode;
                cv::Point3f aim_acceleration = takeoff_acceleration(data_target, GRAVITY);
                std::tie(auto_roll, auto_pitch, auto_throttle) = drone_commands(aim_acceleration);
                if (data_drone.vel_valid && data_drone.pos().y > _dtrk->pad_location().y + land_ctrl.trusted_tracking_height_above_pad())
                    _flight_mode = fm_flying_pid_init;
                break;
        } case fm_flying_pid_init: {
                _flight_mode = fm_flying_pid;

                d_vel_err_x.preset(data_target.vel().x - data_drone.state.vel.x);
                d_vel_err_y.preset(data_target.vel().y - data_drone.state.vel.y);
                d_vel_err_z.preset(data_target.vel().z - data_drone.state.vel.z);

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
                if (control_mode == acceleration_feedforward)
                    control_drone_acceleration_based(data_drone, data_target.pos(), target_acceleration);
                else
                    control_drone_position_based(data_drone, data_target.pos());

                break;
        } case fm_long_range_forth: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_drone_position_based(data_drone, data_target.pos());
                auto_pitch = RC_MIDDLE + RC_BOUND_RANGE / 2 / 8;
                break;
        } case fm_long_range_back: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_drone_position_based(data_drone, data_target.pos());
                auto_pitch = RC_MIDDLE - RC_BOUND_RANGE / 2 / 8;
                break;
        } case fm_calib_thrust:
        case fm_prep_to_land:
        case fm_reset_headless_yaw:
        case fm_headed: {
                mode += bf_headless_disabled;
                auto_yaw = RC_MIDDLE;
                control_drone_position_based(data_drone, data_target.pos());
                break;
        } case fm_reset_yaw_on_pad: {
                mode += bf_headless_disabled;
                auto_throttle = max(static_cast<int>(thrust_to_throttle(2.f)), dparams.spinup_throttle_non3d);
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
                break;
        } case fm_correct_yaw: {
                mode += bf_headless_disabled;
                control_drone_position_based(data_drone, data_target.pos());
                if (data_drone.yaw_deviation_valid)
                    correct_yaw(data_drone.yaw_deviation);
                break;
        } case fm_ff_landing_start: {
                _flight_mode = fm_ff_landing;
                landing_att_calibration_msg_printed = false;
                ff_land_start_time = time;
                ff_auto_throttle_start = auto_throttle;
                auto_yaw = RC_MIDDLE;
                [[fallthrough]];
        } case fm_ff_landing: {
                mode += bf_headless_disabled;
                control_drone_position_based(data_drone, data_drone.pos());
                float dt = static_cast<float>(time - ff_land_start_time);
                auto_throttle = land_ctrl.ff_auto_throttle(ff_auto_throttle_start, dt);

                if (dt > land_ctrl.time_ff_landing()) {
                    auto_throttle = RC_BOUND_MIN;
                    _landed = true;
                    if (dparams.static_shakeit_thrust > 0)
                        _flight_mode = fm_wait;
                    else
                        _flight_mode = fm_inactive;
                }
                break;
        } case fm_start_shake: {
                _shake_finished = false;
                state_start_time = time;
                _flight_mode = fm_shake_it_baby;
                [[fallthrough]];
        } case fm_shake_it_baby: {
                _rc->arm(bf_disarmed);

                const double shake_period = 0.2;
                const double shake_pause_period = 0.2;
                const int spin_value_static = max(static_cast<int>(thrust_to_throttle(dparams.static_shakeit_thrust)), dparams.spinup_throttle_non3d);
                const int spin_value_shake = max(static_cast<int>(thrust_to_throttle(dparams.static_shakeit_thrust * 1.8)), dparams.spinup_throttle_non3d);
                const int spin_value_shake_reversed = max(static_cast<int>(thrust_to_throttle(dparams.static_shakeit_thrust * 2.3f)), dparams.spinup_throttle_non3d);

                //defaults:
                mode += bf_spin_motor;
                auto_roll = RC_BOUND_MIN;
                auto_pitch = RC_BOUND_MIN;
                auto_throttle = RC_BOUND_MIN;
                auto_yaw = RC_BOUND_MIN;

                static int shake_state = 0;
                switch (shake_state) {
                    case 0: // roll
                        auto_roll = spin_value_shake;
                        auto_pitch = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 2:
                        auto_throttle = spin_value_shake;
                        auto_yaw = spin_value_shake;
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
                    case 12: // yaw
                        auto_pitch = spin_value_shake;
                        auto_throttle = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 14:
                        auto_pitch = spin_value_shake;
                        auto_throttle = spin_value_shake;
                        if (time - state_start_time > shake_period) {
                            state_start_time = time;
                            shake_state++;
                        }
                        break;
                    case 16:
                        _shake_finished = true;
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
                break;
        } case fm_trim_accelerometer: {
                auto accel_trim_data = accelerometer_trim.trim_accelerometer(time);
                auto_throttle = accel_trim_data.throttle;
                auto_roll = accel_trim_data.roll;
                auto_pitch = accel_trim_data.pitch;
                auto_yaw = accel_trim_data.yaw;

                if (accel_trim_data.arm)
                    _rc->arm(bf_armed);
                else
                    _rc->arm(bf_disarmed);

                if (accel_trim_data.finished) {
                    _flight_mode = fm_wait;
                }

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
                break;
        } case fm_wait: {
                auto_throttle = RC_BOUND_MIN;
                auto_roll = RC_MIDDLE;
                auto_pitch = RC_MIDDLE;
                auto_yaw = RC_MIDDLE;
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
        } case fm_sleep: {
                mode += bf_sleep;
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
    _rc->queue_commands(throttle, roll, pitch, yaw, mode);

    ControlData c(Roll(), Throttle(), Pitch(), time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    if (enable_logging)
        (*_logger) <<
                   flight_mode_names[_flight_mode] << ";" <<
                   static_cast<int>(data_drone.pos_valid)  << ";" <<
                   data_target.pos().x << ";" << data_target.pos().y  << ";" << data_target.pos().z << ";" <<
                   _target_acceleration.x << ";" << _target_acceleration.y << ";" << _target_acceleration.z << ";" <<
                   commanded_acceleration.x << ";" << commanded_acceleration.y << ";"  << commanded_acceleration.z << ";" <<
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
                   kiv_ctrl.active << ";" <<
                   calibration.max_thrust << ";" <<
                   pos_err_i.x << ";" << pos_err_i.y << ";" << pos_err_i.z << ";" <<
                   _rc->telemetry.batt_cell_v  << ";" <<
                   static_cast<int>(_rc->telemetry.rssi)  << ";";
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_control = std::chrono::high_resolution_clock::now();
    std::cout << "timing (drone_control): " << (t_end_control - t_start_control).count() * 1e-6 << "ms" << std::endl;
#endif
}



bool check_att_bounds(cv::Point2f att, cv::Point2f att_min, cv::Point2f att_max) {
    bool att_check = att.x > att_min.x && att.x < att_max.x &&
                     att.y > att_min.y && att.y < att_max.y;
    return att_check;
}

void DroneController::reset_attitude_pad_filter() {
    att_somewhere_on_pad_cnt = 0;
    att_precisely_on_pad_cnt = 0;
    att_pad_calibration_ok_cnt = 0;
    _att_telemetry_samples_cnt = 0;
}
void DroneController::update_attitude_pad_state() {

    static uint32_t prev_roll_pitch_package_id = 0;
    if (_rc->telemetry.roll_pitch_package_id == prev_roll_pitch_package_id)
        return;

    _att_telemetry_samples_cnt++;
    prev_roll_pitch_package_id = _rc->telemetry.roll_pitch_package_id;

    cv::Point2f current_att(_rc->telemetry.roll, _rc->telemetry.pitch);
    cv::Point2f pad_att_calibration(calibration.pad_roll, calibration.pad_pitch);

    if (check_att_bounds(current_att, pad_att_calibration - att_somewhere_on_pad_range, pad_att_calibration + att_somewhere_on_pad_range)) {
        if (att_somewhere_on_pad_cnt < 6)
            att_somewhere_on_pad_cnt++;
    } else if (att_somewhere_on_pad_cnt)
        att_somewhere_on_pad_cnt--;

    if (check_att_bounds(current_att, pad_att_calibration - att_precisely_on_pad_range, pad_att_calibration + att_precisely_on_pad_range)) {
        if (att_precisely_on_pad_cnt < 6)
            att_precisely_on_pad_cnt++;
    } else if (att_precisely_on_pad_cnt)
        att_precisely_on_pad_cnt--;

    if (check_att_bounds(current_att, -att_pad_calibration_range, att_pad_calibration_range)) {
        if (att_pad_calibration_ok_cnt < 6)
            att_pad_calibration_ok_cnt++;
    } else if (att_pad_calibration_ok_cnt)
        att_pad_calibration_ok_cnt--;

}


void DroneController::invalidize_blink() {
    calibration.pad_calib_date = "2000/01/01 00:00:00";
    save_calibration();
}
void DroneController::save_pad_pos_and_att_calibration() {
    calibration.pad_pos_x = _dtrk->pad_location().x;
    calibration.pad_pos_y = _dtrk->pad_location().y;
    calibration.pad_pos_z = _dtrk->pad_location().z;
    calibration.pad_roll = _rc->telemetry.roll;
    calibration.pad_pitch = _rc->telemetry.pitch;
    auto date = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&date), "%Y/%m/%d %T");
    calibration.pad_calib_date = ss.str();
    save_calibration();
    _flight_area->update_bottom_plane_based_on_blink(calibration.pad_pos_y);
    kiv_ctrl.init(_flight_area, &calibration); //See comment at alternative initialization call
}

void DroneController::init_thrust_calibration() {
    calibration.max_thrust -= dparams.ki_thrust_hover * 0.001f * pos_err_i.y;
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


float DroneController::thrust_to_throttle(float thrust) {
    const float thrust_ratio = thrust / calibration.max_thrust;
    const float p1 = 1985.43232918;
    const float p2 = -2029.22780781;
    const float p3 = 48.2837516242;
    const float p4 = 1501.36671279;
    const float p5 = 292.80869993;
    return p1 * powf(thrust_ratio, 4) + p2 * powf(thrust_ratio, 3) + p3 * powf(thrust_ratio, 2) + p4 * thrust_ratio + p5;
}

cv::Point3f DroneController::takeoff_acceleration(tracking::TrackData data_target, float target_acceleration_y) {
    if (target_acceleration_y >= calibration.max_thrust)
        return cv::Point3f(0, calibration.max_thrust, 0);

    StateData state_drone_takeoff;
    state_drone_takeoff.pos = _dtrk->pad_location() + cv::Point3f(0, lift_off_dist_take_off_aim, 0);
    state_drone_takeoff.vel = {0};
    cv::Point3f burn_direction = data_target.pos() - state_drone_takeoff.pos;
    burn_direction = lowest_direction_to_horizontal(burn_direction, min_takeoff_angle);
    cv::Point3f takeoff_accel = target_acceleration_y / burn_direction.y * burn_direction;

    if (normf(takeoff_accel) > calibration.max_thrust) {
        float modified_takeoff_angle = asinf(target_acceleration_y / calibration.max_thrust);
        burn_direction = lowest_direction_to_horizontal(burn_direction, modified_takeoff_angle);
        takeoff_accel = burn_direction * calibration.max_thrust;
    }

    _target_acceleration = takeoff_accel;
    return takeoff_accel;
}


std::tuple<int, int, int> DroneController::drone_commands(cv::Point3f desired_acc) {

    if (normf(desired_acc) < 0.001f) //if no drone acceleration required, put the drone in an upright orientation
        desired_acc = {0, 0.001, 0};

    if (normf(desired_acc) > calibration.max_thrust)
        desired_acc *= calibration.max_thrust / normf(desired_acc);
    commanded_acceleration = desired_acc;

    cv::Point3f direction = desired_acc / normf(desired_acc);
    int throttle_cmd =  static_cast<uint16_t>(roundf(thrust_to_throttle(normf(desired_acc))));

    if (throttle_cmd < dparams.spinup_throttle_non3d)
        throttle_cmd = dparams.spinup_throttle_non3d;

    auto [roll_quat, pitch_quat] = acc_to_quaternion(direction);
    int roll_cmd =  roundf((roll_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE); // convert to RC commands range
    int pitch_cmd = roundf((-pitch_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE);
    return std::make_tuple(roll_cmd, pitch_cmd, throttle_cmd);
}


cv::Point3f DroneController::combine_drone_accelerations_with_priority(cv::Point3f prio_acc, cv::Point3f trivil_acc) {
    float thrust_limit = 0.95f * calibration.max_thrust;
    if (normf(prio_acc) >= thrust_limit)
        return prio_acc / normf(prio_acc) * thrust_limit;

    cv::Point3f combined_acc = prio_acc + trivil_acc;
    if (normf(combined_acc) <= thrust_limit)
        return combined_acc;

    float a = trivil_acc.dot(trivil_acc);
    float b = 2 * prio_acc.dot(trivil_acc);
    float c = prio_acc.dot(prio_acc) - powf(thrust_limit, 2);
    float a1, a2, valid;
    std::tie(a1, a2, valid) = solve_quadratic_solution(a, b, c);

    if (valid < 0) {
        std::cout << "WARNING: combine drone acceleration failed with prio_acc: " << prio_acc << " and trivial acc: " << trivil_acc << ". Return only prio_acc." << std::endl;
        return prio_acc;
    }

    if (a1 > 1.f && a1 < 1.05f)
        a1 = 1.;
    if (a2 > 1.f && a2 < 1.05f)
        a2 = 1.;

    bool a1_valid = (0 <= a1) && (a1 <= 1);
    bool a2_valid = (0 <= a2) && (a2 <= 1);

    if (a1_valid && !a2_valid) {
        return prio_acc + a1 * trivil_acc;
    } else if (!a1_valid && a2_valid) {
        return prio_acc + a2 * trivil_acc;
    } else if (a1_valid && a2_valid) {
        if (a1 >= a2) {
            return prio_acc + a1 * trivil_acc;
        } else {
            return prio_acc + a2 * trivil_acc;
        }
    } else {
        std::cout << "WARNING: combine drone acceleration failed with prio_acc: " << prio_acc << " and trivial acc: " << trivil_acc << ". Return only prio_acc." << std::endl;
        return prio_acc;
    }

}

void DroneController::mix_drone_accelerations(TrackData data_drone, cv::Point3f target_acc) {
    _target_acceleration = target_acc;
    if (_dtrk->image_predict_item().out_of_image) {
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        auto_throttle = spinup_throttle();
        return;
    }

    cv::Point3f gravity_compensation = cv::Point3f(0, GRAVITY, 0);
    kiv_acc = kiv_update(data_drone);
    cv::Point3f acceleration_mix = {0, 0, 0};
    if (normf(kiv_acc) > 0) {
        acceleration_mix = combine_drone_accelerations_with_priority(gravity_compensation, kiv_acc);
        acceleration_mix = combine_drone_accelerations_with_priority(acceleration_mix, component_a_perpendicular_to_b(target_acc, kiv_acc));
        if (target_acc.dot(kiv_acc) > 0)
            acceleration_mix = combine_drone_accelerations_with_priority(acceleration_mix, component_a_parallel_to_b(target_acc, kiv_acc));
    } else {
        acceleration_mix = combine_drone_accelerations_with_priority(gravity_compensation, target_acc);
    }

    std::tie(auto_roll, auto_pitch, auto_throttle) = drone_commands(acceleration_mix);
}

void DroneController::control_drone_acceleration_based(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_acc) {
    update_pid_controller(data_drone, setpoint_pos, false); //pid controller must be called to update filters (to be ready for switching back to position control)
    mix_drone_accelerations(data_drone, setpoint_acc);
}

void DroneController::control_drone_position_based(TrackData data_drone, cv::Point3f setpoint_pos) {
    cv::Point3f pid_acc = update_pid_controller(data_drone, setpoint_pos, false);
    mix_drone_accelerations(data_drone, pid_acc);
}

cv::Point3f DroneController::kiv_update(TrackData data_drone) {
    kiv_ctrl.update(data_drone, transmission_delay_duration, _time);

    bool flight_mode_with_kiv = _flight_mode == fm_flying_pid || _flight_mode == fm_reset_headless_yaw || _flight_mode == fm_correct_yaw;

    if (data_drone.pos_valid && data_drone.vel_valid && flight_mode_with_kiv && !(_time - start_takeoff_burn_time < 0.45)) {
        if (control_mode_hold_filter.output()) //feedforward was active in the close past
            return kiv_ctrl.correction_acceleration(strict, data_drone, acceleration_feedforward);
        else
            return kiv_ctrl.correction_acceleration(relaxed, data_drone, position_control);
    }
    else
        return {0, 0, 0};
}


cv::Point3f DroneController::update_pid_controller(TrackData data_drone, cv::Point3f setpoint_pos, bool dry_run) {
    //WARNING: this function is not allowed to store any information (or apply filters that store),
    //because it is also being used for dummy calculations in the interceptor! See also the dry_run variable
    integrator_state enable_horizontal_integrators = hold;

    if (!dry_run) {
        pos_modelx.new_sample(setpoint_pos.x);
        pos_modely.new_sample(setpoint_pos.y);
        pos_modelz.new_sample(setpoint_pos.z);
        enable_horizontal_integrators = horizontal_integrators(data_drone.time);
    }


    auto [kp_pos, ki_pos, kd_pos] = adjust_control_gains(data_drone, enable_horizontal_integrators);
    auto [pos_err_p, pos_err_d] = pid_error(data_drone, setpoint_pos, enable_horizontal_integrators, dry_run);

    return multf(kp_pos, pos_err_p) + multf(ki_pos, pos_err_i) + multf(kd_pos, pos_err_d);
}


DroneController::integrator_state DroneController::horizontal_integrators(double time) {
    float duration_waypoint_update = duration_since_waypoint_moved(time);

    if (thrust_calibration
            || (_flight_mode == fm_headed && duration_waypoint_update > 1)
            ||  duration_waypoint_update > 2)
        return running;
    else if (_flight_mode == fm_headed && duration_waypoint_update <= 1)
        return hold;
    else
        return reset;
}

std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::adjust_control_gains(TrackData data_drone, bool enable_horizontal_integrators) {
    float kp_pos_roll_scaled, kp_pos_throttle_scaled, kp_pos_pitch_scaled, ki_pos_roll_scaled, ki_thrust_scaled, ki_pos_pitch_scaled, kd_pos_roll_scaled, kd_pos_throttle_scaled, kd_pos_pitch_scaled;

    cv::Point3f scale_pos_p = {1.f, 1.f, 1.f};
    cv::Point3f scale_pos_i = {1.f, 1.f, 1.f};
    cv::Point3f scale_pos_d = {1.f, 1.f, 1.f};

    if (!enable_horizontal_integrators) {
        scale_pos_i.x = 0;
        scale_pos_i.z = 0;
    }

    if (_hover_mode) {
        kp_pos_roll_scaled = scale_pos_p.x * dparams.kp_pos_roll_hover;
        kp_pos_throttle_scaled = scale_pos_p.y * dparams.kp_pos_throttle_hover;
        kp_pos_pitch_scaled = scale_pos_p.z * dparams.kp_pos_pitch_hover;
        ki_pos_roll_scaled = scale_pos_i.x * dparams.ki_pos_roll_hover;
        ki_thrust_scaled = scale_pos_i.y * dparams.ki_thrust_hover;
        ki_pos_pitch_scaled = scale_pos_i.z * dparams.ki_pos_pitch_hover;
        kd_pos_roll_scaled = scale_pos_d.x * dparams.kd_pos_roll_hover;
        kd_pos_throttle_scaled = scale_pos_d.y * dparams.kd_pos_throttle_hover;
        kd_pos_pitch_scaled = scale_pos_d.z * dparams.kd_pos_pitch_hover;
    } else {
        if (!data_drone.pos_valid) {
            scale_pos_p *= 0.8;
        } else if (norm(data_drone.vel()) > 2) { //Our tuning is such that the drone is aggressiv which includes overshoots. To limit the overshoot after high speed increase D at high speeds.
            scale_pos_d *= 1.2;
        }
        kp_pos_roll_scaled = scale_pos_p.x * dparams.kp_pos_roll;
        kp_pos_throttle_scaled = scale_pos_p.y * dparams.kp_pos_throttle;
        kp_pos_pitch_scaled = scale_pos_p.z * dparams.kp_pos_pitch;
        ki_pos_roll_scaled = scale_pos_i.x * dparams.ki_pos_roll;
        ki_thrust_scaled = scale_pos_i.y * dparams.ki_thrust;
        ki_pos_pitch_scaled = scale_pos_i.z * dparams.ki_pos_pitch;
        kd_pos_roll_scaled = scale_pos_d.x * dparams.kd_pos_roll;
        kd_pos_throttle_scaled = scale_pos_d.y * dparams.kd_pos_throttle;
        kd_pos_pitch_scaled = scale_pos_d.z * dparams.kd_pos_pitch;
    }

    // Arrange gains (needed because may be updated from trackbars)
    cv::Point3f kp_pos(kp_pos_roll_scaled, kp_pos_throttle_scaled, kp_pos_pitch_scaled);
    cv::Point3f kd_pos(kd_pos_roll_scaled, kd_pos_throttle_scaled, kd_pos_pitch_scaled);
    cv::Point3f ki_pos(ki_pos_roll_scaled, ki_thrust_scaled, ki_pos_pitch_scaled);

    return std::tuple(kp_pos, ki_pos, kd_pos);
}

std::tuple<cv::Point3f, cv::Point3f> DroneController::pid_error(TrackData data_drone, cv::Point3f setpoint_pos, integrator_state enable_horizontal_integrators, bool dry_run) {

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
            calibration.max_thrust -= 0.1f * ((err_y_filtered - setpoint_pos.y + pos_modely.current_output()));
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

    if (deviation_angle < - min_yaw_deviation)
        target_theta_velocity = -3;
    else if (deviation_angle > min_yaw_deviation)
        target_theta_velocity = 3;

    auto_yaw = RC_MIDDLE + target_theta_velocity;
}

void DroneController::fine_tune_thrust(float integration_error) {
    float thrust_error = dparams.ki_thrust_hover * integration_error;
    calibration.max_thrust -= 0.3f * thrust_error;
    save_calibration();
}

bool DroneController::abort_take_off() {
    //check if the take off is not yet too far progressed to abort, if not go to spin up else return true
    if (_flight_mode == fm_init_spinup) {
        _flight_mode = fm_init_spinup;
        return true;
    } else
        return false;
}

void DroneController::read_joystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (pparams.joystick == rc_xlite) {
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

void DroneController::load_calibration() {
    if (file_exist(calib_rfn)) {
        xmls::DroneCalibration tmp;
        try {
            tmp.deserialize(calib_rfn);
        } catch (std::runtime_error const &err) {
            std::cout << "Error, corrupted drone calibration xml:" << err.what() << std::endl;
        }
        if ((tmp.drone_id != _rc->drone_id() || tmp.drone_name.compare(dparams.name)) && !log_replay_mode) {
            std::cout << _rc->drone_id() << " vs " << tmp.drone_id << std::endl;
            std::cout << "Drone mismatch. Using default values, but calibration is needed!" << std::endl;
            calibration.max_thrust = dparams.max_thrust;
            calibration.drone_id = _rc->drone_id();
            calibration.drone_name = dparams.name;
        } else
            calibration.deserialize(calib_rfn);
    } else {
        std::cout << "Drone calibration missing. Using default values, but calibration is needed!" << std::endl;
        calibration.max_thrust = dparams.max_thrust;
        calibration.drone_id = _rc->drone_id();
        calibration.drone_name = dparams.name;
    }

    _dtrk->set_pad_location(calibration.pad_pos());
    _flight_area->update_bottom_plane_based_on_blink(calibration.pad_pos_y);
    _flight_area->set_vertical_camera_plane(calibration.pad_pos_z);
    kiv_ctrl.init(_flight_area, &calibration);// kiv can only be initialized after the latest (potential) adding of a plane (bottom_plane) since filter initialization is dependent on the number of planes in the flight area
}

void DroneController::save_calibration() {
    if (!log_replay_mode)
        calibration.serialize(pats_folder + "xml/" + calib_fn);
}
void DroneController::save_calibration_before_flight(int flight_id) {
    calibration.serialize(data_output_dir + "/drone_calibration_flight" + to_string(flight_id) + ".xml");
}

bool DroneController::pad_calib_valid() {
    if (!calibration.pad_calib_date.length())
        return false;
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

void DroneController::close() {
    if (initialized) {
        LED(true);
        std::cout << "Closing controller." << std::endl;
        initialized = false;
    }
}
