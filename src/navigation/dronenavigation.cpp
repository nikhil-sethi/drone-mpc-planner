#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

namespace navigation {


void DroneNavigation::init(std::ofstream *logger, tracking::TrackerManager * trackers, DroneController * dctrl, VisionData *visdat, CameraView *camview,std::string replay_dir) {
    _logger = logger;
    _trackers = trackers;
    _dctrl = dctrl;
    _visdat = visdat;
    _camview = camview;

    _iceptor.init(_trackers, visdat, camview, logger);

    deserialize_settings();
    deserialize_flightplan(replay_dir);

    if (pparams.navigation_tuning) {
        namedWindow("Nav", WINDOW_NORMAL);
        createTrackbar("X [cm", "Nav", &setpoint_slider_X, 500);
        createTrackbar("Y off", "Nav", &setpoint_slider_Y, 500);
        createTrackbar("Z center]", "Nav", &setpoint_slider_Z, 500);

        createTrackbar("v_crcl1", "Nav", &v_crcl1, 1000);
        createTrackbar("v_crcl2", "Nav", &v_crcl2, 1000);
        createTrackbar("r_crcl1", "Nav", &r_crcl1, 1000);
        createTrackbar("r_crcl2", "Nav", &r_crcl2, 1000);

        createTrackbar ("w_sqr", "Nav", &w_sqr, 2500);
        createTrackbar ("v_sqr", "Nav", &v_sqr, 500);
    }

    if (pparams.op_mode == op_mode_monitoring_only)
        _navigation_status = ns_wait_for_insect;

    (*_logger) << "nav_state;";
    initialized = true;
}

void DroneNavigation::deserialize_flightplan(std::string replay_dir) {
    //Waypoints are relative to the camera position. The camera is 0,0,0.
    //X image vs world is reversed! Negative world x is right in the image.
    //Everything below the camera is negative Y, heigher than the camera is positive
    //Farther away from the camera is negative Z, positive Z should be impossible because the camera can't see that.
    //The flight plan will be repeated indefinetely, unless there is a landing waypoint somewhere in the list.
    navigation::XML_FlightPlan fp;
    if (replay_dir == "") {
        fp.deserialize(pparams.flightplan);
        fp.serialize("./logging/flightplan.xml"); // write a copy of the currently used flightplan to the logging dir
    } else {
        fp.deserialize(replay_dir + "/flightplan.xml");
    }
    waypoints = fp.waypoints();
}

void DroneNavigation::update(double time) {

    if (_dctrl->Joy_State() != DroneController::js_none) {
        if (_dctrl->Joy_State() == DroneController::js_checking ||
                _dctrl->Joy_State() == DroneController::js_none ||
                _dctrl->Joy_State() == DroneController::js_disarmed )
            _nav_flight_mode = nfm_none;
        else if(_dctrl->Joy_State() == DroneController::js_hunt)
            _nav_flight_mode = nfm_hunt;
        else if(_dctrl->Joy_State() == DroneController::js_waypoint)
            _nav_flight_mode = nfm_waypoint;
        else if(_dctrl->Joy_State() == DroneController::js_manual)
            _nav_flight_mode = nfm_manual;
    } else if (_navigation_status == ns_init) {
        _nav_flight_mode = nfm_hunt;
    }

    if (_dctrl->flight_aborted() || _dctrl->in_flight_duration(time) > 180)
        _navigation_status = ns_drone_problem;

    _iceptor.update(_navigation_status != ns_chasing_insect, time);
    bool repeat = true;
    while (repeat) {
        repeat  = false;
        switch (_navigation_status) {
        case ns_init: {
            _dctrl->LED(true);
            locate_drone_attempts = 0;
            _visdat->reset_motion_integration();
            if (time > 1.5) { // skip first second or so due to auto exposure
                _navigation_status = ns_locate_drone_init;
            }
            _trackers->mode(tracking::TrackerManager::mode_idle);
            break;
        } case ns_locate_drone_init: {
            _dctrl->LED(true);
            _dctrl->flight_mode(DroneController::fm_disarmed);
            locate_drone_start_time = time;
            _navigation_status = ns_locate_drone_wait_led_on;
            [[fallthrough]];
        } case ns_locate_drone_wait_led_on: {
            if (time - locate_drone_start_time > 0.1) {
                _visdat->reset_motion_integration();
                _visdat->disable_fading = true;
                _dctrl->flight_mode(DroneController::fm_blink);
                _trackers->mode(tracking::TrackerManager::mode_locate_drone);
                _navigation_status = ns_wait_locate_drone;
            }
            break;
        } case ns_wait_locate_drone: {
            static double __attribute__((unused)) prev_time = time;
            if (static_cast<float>(time - prev_time) > dparams.blink_period)
                _dctrl->flight_mode(DroneController::fm_blink);
            if (_trackers->mode() != tracking::TrackerManager::mode_locate_drone) {
                _dctrl->flight_mode(DroneController::fm_disarmed);
                time_located_drone = time;
                locate_drone_attempts = 0;
                if (pparams.op_mode == op_mode_crippled) {
                    _navigation_status = ns_crippled;
                } else
                    _navigation_status = ns_located_drone;
            }
            if (time - locate_drone_start_time/(locate_drone_attempts+1) > 15 )
                _navigation_status = ns_locate_drone_init;
            if (time - locate_drone_start_time > 45 && pparams.op_mode != op_mode_crippled)
                _navigation_status = ns_drone_problem;
            break;
        } case ns_crippled: {
            _dctrl->flight_mode(DroneController::fm_disarmed);
            int itime = time;
            _dctrl->LED(true,itime*5 % 50);
            break;
        } case ns_located_drone: {
            _dctrl->LED(true);
            _trackers->mode(tracking::TrackerManager::mode_idle);
            _visdat->disable_fading = false;
            _visdat->enable_collect_no_drone_frames = true;
            if (time-time_located_drone>1.0 && (_dctrl->drone_state_inactive() || pparams.joystick != rc_none)) { // delay until blinking stopped
                _visdat->enable_background_motion_map_calibration(2);
                _visdat->create_overexposed_removal_mask(_trackers->dronetracker()->drone_takeoff_im_location(),_trackers->dronetracker()->drone_takeoff_im_size());
                if (_nav_flight_mode == nfm_hunt)
                    _navigation_status = ns_wait_for_insect;
                else if (_nav_flight_mode == nfm_manual)
                    _navigation_status = ns_manual;
                else
                    _navigation_status = ns_wait_for_takeoff_command;
            }
            break;
        } case ns_wait_for_takeoff_command: {
            _trackers->mode(tracking::TrackerManager::mode_idle);
            _dctrl->flight_mode(DroneController::fm_inactive);
            if (_nav_flight_mode == nfm_hunt)
                _navigation_status = ns_wait_for_insect;
            else if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else if ( _dctrl->manual_override_take_off_now()) {
                wpid = 0;
                next_waypoint(waypoints[wpid],time);
                _navigation_status = ns_takeoff;
                _visdat->enable_collect_no_drone_frames = false;
                repeat = true;
            }
            break;
        } case ns_wait_for_insect: {
            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else if (_nav_flight_mode == nfm_hunt) {
                _trackers->mode(tracking::TrackerManager::mode_wait_for_insect);

                auto itrkr = _trackers->insecttracker_best();

                if(_iceptor.insect_in_range_takeoff(itrkr)) {
                    _navigation_status = ns_takeoff;
                    repeat = true;
                } else if(itrkr->tracking () && !itrkr->false_positive()) {
                    _dctrl->flight_mode (DroneController::fm_spinup);
                } else {
                    _dctrl->flight_mode(DroneController::fm_inactive);
                }
            } else if (_nav_flight_mode == nfm_none) {
                _dctrl->flight_mode(DroneController::fm_inactive);
            } else if (_nav_flight_mode == nfm_waypoint)
                _navigation_status = ns_wait_for_takeoff_command;
            break;
        } case ns_takeoff: {
            _dctrl->reset_manual_override_take_off_now();
            _dctrl->flight_mode(DroneController::fm_start_takeoff);
            _dctrl->hover_mode(false);
            _trackers->dronetracker()->hover_mode(false);
            time_take_off = time;
            if (_nav_flight_mode == nfm_hunt)
                _trackers->mode(tracking::TrackerManager::mode_hunt);
            else
                _trackers->mode(tracking::TrackerManager::mode_drone_only);
            _navigation_status=ns_taking_off;
            break;
        } case ns_taking_off: {
            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
                break;
            }
            if (_trackers->dronetracker()->take_off_detection_failed()) {
                std::cout << "Drone was not detected during max burn take off manoeuvre, aborting." << std::endl;
                _dctrl->flight_mode(DroneController::fm_abort_flight);
                _dctrl->flight_submode_name = "fm_abort_takeoff";
                _navigation_status = ns_drone_problem;
                break;
            }
            if (_dctrl->spinup()) {
                _navigation_status = ns_wait_for_insect;
                break;
            }

            if (_iceptor.insect_in_range_takeoff(_trackers->insecttracker_best()) && _nav_flight_mode == nfm_hunt) {
                setpoint_pos_world = _iceptor.target_position();
                setpoint_vel_world = _iceptor.target_speed();
                setpoint_acc_world = _iceptor.target_accelleration();
            } else if (_nav_flight_mode == nfm_hunt && _dctrl->abort_take_off()) {
                _navigation_status = ns_wait_for_insect;
            }

            if (!_trackers->dronetracker()->taking_off() && time - time_take_off > dparams.max_burn_time)
                _navigation_status = ns_take_off_completed;
            else
                break;
            [[fallthrough]];
        } case ns_take_off_completed: {

            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status = ns_start_the_chase;
                repeat = true;
            } else if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else {
                wpid++;
                _navigation_status = ns_set_waypoint;
            }
            break;
        } case ns_start_the_chase: {
            _iceptor.reset_insect_cleared();
            _navigation_status = ns_chasing_insect_ff;
            [[fallthrough]];
        } case ns_chasing_insect_ff: {
            if (_dctrl->ff_completed())
                _navigation_status = ns_chasing_insect;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            break;
        } case ns_chasing_insect: {
            //update target chasing waypoint and speed
            if (_iceptor.insect_in_range()) {
                setpoint_pos_world = _iceptor.target_position();
                setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
                setpoint_vel_world = _iceptor.target_speed();
                setpoint_acc_world = _iceptor.target_accelleration();
            }

            if(drone_is_blocked(0.1))
                _navigation_status = ns_drone_problem;

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            else if (_iceptor.insect_cleared())
                _navigation_status = ns_goto_landing_waypoint;
            break;
        } case ns_goto_landing_waypoint: {
            next_waypoint(Waypoint_Landing(),time);
            _navigation_status = ns_approach_waypoint;
            break;
        } case ns_set_waypoint: {
            next_waypoint(waypoints[wpid],time);
            if (current_waypoint->mode == wfm_flower)
                _navigation_status = ns_flower_waypoint;
            else if(current_waypoint->mode == wfm_brick)
                _navigation_status = ns_brick_waypoint;
            else
                _navigation_status = ns_approach_waypoint;
            time_wp_reached = -1;
            break;
        } case ns_approach_waypoint: {
            if (pparams.navigation_tuning && current_waypoint->mode != wfm_landing && current_waypoint->mode != wfm_takeoff ) {
                setpoint_pos_world.x = waypoints[wpid].xyz.x + (250-setpoint_slider_X)/100.f;
                setpoint_pos_world.y = waypoints[wpid].xyz.y + (250-setpoint_slider_Y)/100.f;
                setpoint_pos_world.z = waypoints[wpid].xyz.z + (setpoint_slider_Z-250)/100.f;
                setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
            }

            if(drone_is_blocked(0.005))
                _navigation_status = ns_drone_problem;

            if (_dctrl->dist_to_setpoint() *1000 < current_waypoint->threshold_mm * distance_threshold_f
                    && normf(_trackers->dronetracker()->Last_track_data().state.vel) < current_waypoint->threshold_v
                    && _trackers->dronetracker()->n_frames_tracking>5)
            {
                if (current_waypoint->mode == wfm_landing) {
                    _dctrl->flight_mode(DroneController::fm_initial_reset_yaw);
                }
                if ((static_cast<float>(time - time_wp_reached) > current_waypoint->hover_pause && time_wp_reached > 0) || current_waypoint->hover_pause <= 0) {
                    if (current_waypoint->mode == wfm_landing)
                        _navigation_status = ns_land;
                    else if (current_waypoint->mode == wfm_yaw_reset)
                        _navigation_status = ns_initial_reset_yaw;
                    else if (wpid < waypoints.size()) { // next waypoint in flight plan
                        wpid++;
                        _navigation_status = ns_set_waypoint;
                    } else if (wpid == waypoints.size()) {
                        wpid = 0; // another round
                        _navigation_status = ns_set_waypoint;
                    }
                } else if (time_wp_reached<0) {
                    time_wp_reached = time;
                }
            }

            if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range())
                _navigation_status = ns_start_the_chase;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_flower_waypoint: {

            float timef = static_cast<float>(time);

            cv::Point3f new_pos_setpoint;
            cv::Point3f new_vel_setpoint;
            new_pos_setpoint.x = current_waypoint->xyz.x + (r_crcl1/100.f) * sinf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * cosf((v_crcl2/100.f)*timef);
            new_pos_setpoint.y = current_waypoint->xyz.y;
            new_pos_setpoint.z = current_waypoint->xyz.z + (r_crcl1/100.f) * cosf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * sinf((v_crcl2/100.f)*timef);
            new_vel_setpoint = (new_pos_setpoint - setpoint_pos_world)*static_cast<float>(pparams.fps);
            setpoint_acc_world = (new_vel_setpoint - setpoint_vel_world)*static_cast<float>(pparams.fps);
            setpoint_vel_world = new_vel_setpoint;
            setpoint_pos_world = new_pos_setpoint;

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_brick_waypoint: {
            float timef = static_cast<float>(time);

            cv::Point3f new_pos_setpoint;
            cv::Point3f new_vel_setpoint;
            new_pos_setpoint = square_point (current_waypoint->xyz, static_cast<float>(w_sqr)/1000.f, static_cast<float>(v_sqr)/100.f*timef);
            std::cout << "brick_setpoint: " << new_pos_setpoint << std::endl;
            new_vel_setpoint = (new_pos_setpoint - setpoint_pos_world)*static_cast<float>(pparams.fps);
            setpoint_acc_world = (new_vel_setpoint - setpoint_vel_world)*static_cast<float>(pparams.fps);
            setpoint_vel_world = new_vel_setpoint;
            setpoint_pos_world = new_pos_setpoint;

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_initial_reset_yaw: {
            _dctrl->flight_mode(DroneController::fm_initial_reset_yaw);

            float pos_error = normf( setpoint_pos_world - _trackers->dronetracker()->Last_track_data().pos());
            if(time-time_initial_reset_yaw>0.5 && pos_error<0.3f) {
                _trackers->dronetracker()->detect_yaw();
                _navigation_status = ns_wait_reset_yaw;
            }
            break;
        } case ns_wait_reset_yaw: {
            _dctrl->flight_mode(DroneController::fm_reset_yaw);
            if(_trackers->dronetracker()->check_yaw()
                    && _trackers->dronetracker()-> check_smooth_yaw()) {
                if(_nav_flight_mode == nfm_waypoint) {
                    _navigation_status = ns_set_waypoint;
                    wpid++;
                } else {
                    _navigation_status = ns_goto_landing_waypoint;
                }
            }
            break;
        } case ns_land: {
            landing_start_time = time;
            _dctrl->hover_mode(true);
            _trackers->dronetracker()->hover_mode(true);
            _trackers->dronetracker()->land();
            _navigation_status = ns_landing;
            [[fallthrough]];
        } case ns_landing: {
            cv::Point3f new_pos_setpoint;
            cv::Point3f new_vel_setpoint;
            cv::Point3f pad_pos = _trackers->dronetracker()->drone_landing_location();
            setpoint_pos_world =  pad_pos + current_waypoint->xyz;
            if (setpoint_pos_world.y > -0.5f) // keep some margin to the top of the view, because atm we have an overshoot problem.
                setpoint_pos_world.y = -0.5f;

            float dt_land = static_cast<float>(time-landing_start_time);
            float v_descend = -0.5f;
            float target_time = -2.f * current_waypoint->xyz.y/v_descend;
            if (dt_land < target_time)
                v_descend = v_descend*(dt_land/target_time);
            new_pos_setpoint.x = setpoint_pos_world.x;
            new_pos_setpoint.y = setpoint_pos_world.y + dt_land* v_descend;
            new_pos_setpoint.z = setpoint_pos_world.z;
            setpoint_vel_world = {0};
            setpoint_acc_world = {0};

            if (new_pos_setpoint.y < pad_pos.y) {
                // new_pos_setpoint.y = pad_pos.y;
                setpoint_vel_world = {0};
                if ((!_dctrl->landing()) && ((_trackers->dronetracker()->Last_track_data().spos().y < pad_pos.y+0.2f) || (!_trackers->dronetracker()->Last_track_data().spos_valid)))
                    _dctrl->flight_mode(DroneController::fm_ff_landing_start);
            }
            setpoint_pos_world = new_pos_setpoint;
            if (!_dctrl->drone_is_active())
                _navigation_status = ns_landed;
            if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range() && !_dctrl->landing())
                _navigation_status = ns_start_the_chase;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_landed: {
            wpid = 0;
            _navigation_status = ns_wait_after_landing;
            landed_time = time;
            _trackers->dronetracker()->delete_landing_motion(time_out_after_landing);
            _dctrl->hover_mode(false);
            _trackers->dronetracker()->hover_mode(false);
            [[fallthrough]];
        } case ns_wait_after_landing: {
            if (static_cast<float>(time - landed_time) > time_out_after_landing )
                _navigation_status = ns_locate_drone_init;
            break;
        } case ns_manual: { // also used for disarmed
            wpid = 0;
            _trackers->mode(tracking::TrackerManager::mode_drone_only);
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status=ns_wait_for_insect;
            } else if (_nav_flight_mode != nfm_manual) { // waypoint mode
                _navigation_status=ns_wait_for_takeoff_command;
            }
            break;
        } case ns_drone_problem: {
            if (time_drone_problem < 0)
                time_drone_problem = time;
            _dctrl->flight_mode(DroneController::fm_abort_flight);
            _dctrl->LED(static_cast<int>((time - time_drone_problem) * 10.0) % 10 > 5,100); // blink every half second
            _dctrl->beep(true);
            break;
        }
        }
    }
    (*_logger) << static_cast<int16_t>(_navigation_status) << ";";
}

void DroneNavigation::next_waypoint(Waypoint wp, double time) {
    current_waypoint = new Waypoint(wp);
    if (wp.mode == wfm_takeoff) {
        cv::Point3f p = _trackers->dronetracker()->drone_takeoff_location();
        setpoint_pos_world =  p + wp.xyz;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
    } else if (wp.mode == wfm_landing) {
        cv::Point3f p = _trackers->dronetracker()->drone_landing_location();
        setpoint_pos_world =  p + wp.xyz;
        if (setpoint_pos_world.y > -0.5f) // keep some margin to the top of the view, because atm we have an overshoot problem.
            setpoint_pos_world.y = -0.5f;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
    } else {
        setpoint_pos_world =  wp.xyz;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
    }
    _dctrl->nav_waypoint_changed(time);

    setpoint_vel_world = {0,0,0};
    setpoint_acc_world = {0,0,0};
}

void DroneNavigation::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    navigationParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        navigationParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
    } else {
        throw my_exit("File not found: " + settings_file);
    }

    distance_threshold_f = params.distance_threshold_f.value();
    time_out_after_landing = params.time_out_after_landing.value();
}

void DroneNavigation::serialize_settings() {
    navigationParameters params;
    params.distance_threshold_f = distance_threshold_f;
    params.time_out_after_landing = time_out_after_landing;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void DroneNavigation::close() {
    if (initialized) {
        std::cout << "Closing drone navigation" << std::endl;
        if (pparams.navigation_tuning)
            serialize_settings();
        initialized = false;
    }
}

cv::Point3f DroneNavigation::square_point(cv::Point3f center, float width, float s) {
    s = fmodf(s, 4*width);
    float si = fmodf(s, width);

    float x_offset, y_offset, z_offset;

    if(s>3*width) {
        x_offset = -width/2;
        z_offset = width/2 - si;
    } else if(s>2*width) {
        x_offset = width/2 - si;
        z_offset = width/2;
    } else if(s>width) {
        x_offset = width/2;
        z_offset = -width/2 + si;
    } else {
        x_offset = -width/2 + si;
        z_offset = -width/2;
    }
    y_offset = z_offset;
    z_offset = 0;
    // Miss use this function to tune the pid controller:
    //x_offset = 0; z_offset = 0;
    //float y_offset = width/2 * sin(si/width*2*M_PIf32);

    return {center.x+x_offset, center.y+y_offset, center.z+z_offset};
}

bool DroneNavigation::drone_is_blocked(float speed_threshold) {
    float speed = norm(_trackers->dronetracker()->Last_track_data().vel());
    bool speed_valid = _trackers->dronetracker()->Last_track_data().vel_valid;
    float error = 0; // TODO re implement locally : _dctrl->position_error();

    if(_dctrl->auto_throttle>1700 && speed<speed_threshold && error>0.3f && speed_valid)
        return true;
    return false;
}

void DroneNavigation::demo_flight(std::string flightplan_fn) {
    navigation::XML_FlightPlan fp;
    fp.deserialize(flightplan_fn);
    fp.serialize("./logging/flightplan.xml"); // write a copy of the currently used flightplan to the logging dir
    waypoints = fp.waypoints();
    wpid = 0;
    _nav_flight_mode = nfm_waypoint;
    next_waypoint(waypoints[wpid],_trackers->dronetracker()->Last_track_data().time);
    _navigation_status = ns_takeoff;
}
}

