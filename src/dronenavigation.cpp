#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

void DroneNavigation::init(std::ofstream *logger, TrackerManager * trackers, DroneController * dctrl, VisionData *visdat, CameraVolume *camvol) {
    _logger = logger;
    _trackers = trackers;
    _dctrl = dctrl;
    _visdat = visdat;

    _iceptor.init(_trackers, visdat, camvol, logger);

    // Load saved navigation paremeters
    deserialize_settings();

    //Waypoints are relative to the camera position. The camera is 0,0,0.
    //X goes from negative left, to positive right.
    //Everything below the camera is negative Y, heigher than the camera is positive
    //Farther away from the camera is negative Z, positive Z should be impossible because the camera can't see that.

    //The flight plan will be repeated indefinetely, unless there is a landing waypoint somewhere in the list.

    //    setpoints.push_back(waypoint(cv::Point3f(-2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(-2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(2,-1.0f,-3.5f),100));



//    setpoints.push_back(stay_waypoint(cv::Point3f(0,-0.7f,-2.0f)));
//    setpoints.push_back(stay_waypoint(cv::Point3f(-1,-0.7f,-2.0f)));
//    setpoints.push_back(stay_waypoint(cv::Point3f(1,-0.7f,-2.0f)));

//    setpoints.push_back(stay_waypoint(cv::Point3f(0,-0.7f,-2.0f)));
//    setpoints.push_back(stay_waypoint(cv::Point3f(0,-0.7f,-1.3f)));
//    setpoints.push_back(stay_waypoint(cv::Point3f(0,-0.7f,-2.3f)));

    setpoints.push_back(waypoint(cv::Point3f(0,-1.0f,-1.3f),100));

    //    setpoints.push_back(flower_waypoint(cv::Point3f(0,-1.5f,-2.0f)));
//    setpoints.push_back(brick_waypoint(cv::Point3f(0,-1.f,-2.0f)));

    setpoints.push_back(landing_waypoint());

    if (pparams.navigation_tuning) {
        namedWindow("Nav", WINDOW_NORMAL);
        createTrackbar("X [cm", "Nav", &setpoint_slider_X, 500);
        createTrackbar("Y off", "Nav", &setpoint_slider_Y, 500);
        createTrackbar("Z center]", "Nav", &setpoint_slider_Z, 500);
        //        createTrackbar("WP id", "Nav", reinterpret_cast<int*>(wpid), setpoints.size()-1);
        //        createTrackbar("d threshold factor", "Nav", &distance_threshold_f, 10);

        createTrackbar("v_crcl1", "Nav", &v_crcl1, 1000);
        createTrackbar("v_crcl2", "Nav", &v_crcl2, 1000);
        createTrackbar("r_crcl1", "Nav", &r_crcl1, 1000);
        createTrackbar("r_crcl2", "Nav", &r_crcl2, 1000);

        createTrackbar ("w_sqr", "Nav", &w_sqr, 2500);
        createTrackbar ("v_sqr", "Nav", &v_sqr, 500);
    }

    if (pparams.insect_logging_mode)
        _navigation_status = ns_wait_for_insect;

    (*_logger) << "nav_state;";
    initialized = true;
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
    } else {
        _nav_flight_mode = nfm_hunt;
    }
    _iceptor.update(_navigation_status != ns_chasing_insect, time);
    bool repeat = true;
    while (repeat) {
        repeat  = false;
        switch (_navigation_status) {
        case ns_init: {
            if (pparams.drone == drone_trashcan)
                _dctrl->LED(true);
            if (time > 1) // skip the first second or so, e.g. auto exposure may give heavily changing images
                _navigation_status = ns_locate_drone;
            _trackers->mode(TrackerManager::mode_idle);
            break;
        } case ns_locate_drone: {
            if(pparams.drone != drone_trashcan)
                _dctrl->blink_by_binding(true);
            _trackers->mode(TrackerManager::mode_locate_drone);
            _visdat->reset_motion_integration();
            _visdat->disable_fading = true;
            _navigation_status = ns_wait_locate_drone;
            break;
        } case ns_wait_locate_drone: {
            static double __attribute__((unused)) prev_time = time;
            if (pparams.drone != drone_trashcan) {
                if (time - prev_time > 7 && time - prev_time < 8) {
                    _dctrl->blink_by_binding(false); // refresh the blinking
                } else if (abs(time - prev_time) > 8.5) {
                    prev_time = time;
                    _dctrl->blink_by_binding(true);
                }
            } else {
                if (static_cast<float>(time - prev_time) > dparams.blink_period)
                    _dctrl->blink(time);
            }
            if (_trackers->mode() != TrackerManager::mode_locate_drone) {
                _navigation_status = ns_located_drone;
                time_located_drone = time;
            }

            break;
        } case ns_located_drone: {
            if (pparams.drone != drone_trashcan)
                _dctrl->blink_by_binding(false);
            else
                _dctrl->LED(true);
            _trackers->mode(TrackerManager::mode_idle);
            _visdat->disable_fading = false;
            if (time-time_located_drone>1.0 && (_dctrl->drone_state_inactive() || pparams.joystick != rc_none)) { // delay until blinking stopped
                if (_nav_flight_mode == nfm_hunt)
                    _navigation_status = ns_wait_for_insect;
                else if (_nav_flight_mode == nfm_manual)
                    _navigation_status = ns_manual;
                else
                    _navigation_status = ns_wait_for_takeoff_command;
            }
            break;
        } case ns_wait_for_takeoff_command: {
            _trackers->mode(TrackerManager::mode_idle);
            _dctrl->flight_mode(DroneController::fm_inactive);
            if (_nav_flight_mode == nfm_hunt)
                _navigation_status = ns_wait_for_insect;
            else if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else if (_dctrl->manual_override_take_off_now() ){
                next_waypoint(setpoints[wpid]);
                _navigation_status = ns_takeoff;
                repeat = true;
            }
            break;
        } case ns_wait_for_insect: {
            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else if (_nav_flight_mode == nfm_hunt){
                _trackers->mode(TrackerManager::mode_wait_for_insect);
                if(_iceptor.insect_in_range_takeoff()) {
                    _navigation_status = ns_takeoff;
                    repeat = true;
                } else if(_trackers->insecttracker_best ()->tracking ()){
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
            time_taken_off = time;
            if (_nav_flight_mode == nfm_hunt)
                _trackers->mode(TrackerManager::mode_hunt);
            else
                _trackers->mode(TrackerManager::mode_drone_only);
            _navigation_status=ns_taking_off;
            break;
        } case ns_taking_off: {
            if (_nav_flight_mode == nfm_manual){
                _navigation_status = ns_manual;
                break;
            }
            if (_dctrl->duration_spent_taking_off(time)>0.5f && _trackers->dronetracker()->taking_off()){ //TODO: make parameter
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

            if (_iceptor.insect_in_range_takeoff()) {
                setpoint_pos_world = _iceptor.target_position();
                setpoint_vel_world = _iceptor.target_speed();
                setpoint_acc_world = _iceptor.target_accelleration();
            } else if (_nav_flight_mode == nfm_hunt && _dctrl->abort_take_off()) {
                _navigation_status = ns_wait_for_insect;
            }

            if (!_trackers->dronetracker()->taking_off() && time - time_taken_off > dparams.max_burn_time)
                _navigation_status = ns_take_off_completed;
            else
                break;
            [[fallthrough]];
        } case ns_take_off_completed: {
            //_dctrl->flight_mode(DroneController::fm_flying);
            _dctrl->hoverthrottle = _trackers->dronetracker()->hover_throttle_estimation;

            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status = ns_start_the_chase;
                repeat = true;
            } else if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else {
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
                setpoint_vel_world = _iceptor.target_speed();
                setpoint_acc_world = _iceptor.target_accelleration();
            }

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            else if (_iceptor.insect_cleared())
                _navigation_status = ns_goto_landing_waypoint;
            break;
        } case ns_goto_landing_waypoint: {
            next_waypoint(landing_waypoint());
            _navigation_status = ns_approach_waypoint;
            break;
        } case ns_set_waypoint: {
            next_waypoint(setpoints[wpid]);
            if (current_setpoint->mode == fm_flower)
                _navigation_status = ns_flower_waypoint;
            else if(current_setpoint->mode == fm_brick)
                _navigation_status = ns_brick_waypoint;
            else
                _navigation_status = ns_approach_waypoint;
            break;
        } case ns_approach_waypoint: {

            if (pparams.navigation_tuning && current_setpoint->mode != fm_landing && current_setpoint->mode != fm_takeoff ){
                setpoint_pos_world.x = setpoints[wpid].xyz.x + (250-setpoint_slider_X)/100.f;
                setpoint_pos_world.y = setpoints[wpid].xyz.y + (250-setpoint_slider_Y)/100.f;
                setpoint_pos_world.z = setpoints[wpid].xyz.z + (setpoint_slider_Z-250)/100.f;
            }

            if (_dctrl->dist_to_setpoint() *1000 < current_setpoint->threshold_mm * distance_threshold_f
                && normf(_trackers->dronetracker()->Last_track_data().state.vel) < 1.6f
                && _trackers->dronetracker()->n_frames_tracking>5)
            {
                if (current_setpoint->mode == fm_landing) {
                    _navigation_status = ns_initial_reset_heading;
                    time_initial_reset_heading = time;
                } else if (wpid < setpoints.size()) { // next waypoint in flight plan
                    wpid++;
                    _navigation_status = ns_set_waypoint;
                } else if (wpid == setpoints.size()){
                    wpid = 0; // another round
                    _navigation_status = ns_set_waypoint;
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
            new_pos_setpoint.x = current_setpoint->xyz.x + (r_crcl1/100.f) * sinf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * cosf((v_crcl2/100.f)*timef);
            new_pos_setpoint.y = current_setpoint->xyz.y;
            new_pos_setpoint.z = current_setpoint->xyz.z + (r_crcl1/100.f) * cosf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * sinf((v_crcl2/100.f)*timef);
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
            new_pos_setpoint = square_point (current_setpoint->xyz, static_cast<float>(w_sqr)/1000.f, static_cast<float>(v_sqr)/100.f*timef);
            std::cout << "brick_setpoint: " << new_pos_setpoint << std::endl;
            new_vel_setpoint = (new_pos_setpoint - setpoint_pos_world)*static_cast<float>(pparams.fps);
            setpoint_acc_world = (new_vel_setpoint - setpoint_vel_world)*static_cast<float>(pparams.fps);
            setpoint_vel_world = new_vel_setpoint;
            setpoint_pos_world = new_pos_setpoint;

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_initial_reset_heading: {
            _dctrl->flight_mode(DroneController::fm_initial_reset_heading);
            if(time-time_initial_reset_heading>0.5){
                _navigation_status = ns_reset_heading;
            }
            break;
        } case ns_reset_heading: {
            _dctrl->flight_mode(DroneController::fm_reset_heading);
            _trackers->dronetracker()->reset_heading();
            if(_trackers->dronetracker()->check_heading() == true && norm(_trackers->dronetracker()->Last_track_data().vel()) < 0.05){
                _navigation_status = ns_land;
            }
            break;
        } case ns_land: {
            _dctrl->flight_mode(DroneController::fm_landing_start);
            _trackers->dronetracker()->land();
            _navigation_status = ns_landing;
            [[fallthrough]];
        } case ns_landing: {
            track_data data = _trackers->dronetracker ()->Last_track_data ();
            if (data.pos ().y <= _trackers->dronetracker()->drone_landing_location ().y + 0.05f || !data.pos_valid)
                _navigation_status = ns_landed;

            if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range())
                _navigation_status = ns_start_the_chase;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_landed: {
            wpid = 0;
            _dctrl->flight_mode(DroneController::fm_inactive);
            _navigation_status = ns_wait_after_landing;
            landed_time = time;
            [[fallthrough]];
        } case ns_wait_after_landing: {
            _visdat->delete_from_motion_map(_trackers->dronetracker()->drone_startup_im_location()*pparams.imscalef,_trackers->dronetracker()->drone_startup_im_disparity(),_trackers->dronetracker()->drone_startup_im_size()*pparams.imscalef,1);
            if (static_cast<float>(time - landed_time) > time_out_after_landing )
                _navigation_status = ns_locate_drone;
            break;
        } case ns_manual: { // also used for disarmed
            wpid = 0;
            _trackers->mode(TrackerManager::mode_drone_only);
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status=ns_wait_for_insect;
            } else if (_nav_flight_mode != nfm_manual) { // waypoint mode
                _navigation_status=ns_wait_for_takeoff_command;
            }
            break;
        } case ns_drone_problem: {
            break;
        }
        }
    }
    (*_logger) << static_cast<int16_t>(_navigation_status) << ";";
}

void DroneNavigation::next_waypoint(waypoint wp) {
    current_setpoint = new waypoint(wp);
    if (wp.mode == fm_takeoff) {
        cv::Point3f p = _trackers->dronetracker()->drone_startup_location();
        setpoint_pos_world =  p + wp.xyz;
    }else if (wp.mode == fm_landing ) {
        cv::Point3f p = _trackers->dronetracker()->drone_landing_location();
        setpoint_pos_world =  p + wp.xyz;
    } else {
        setpoint_pos_world =  wp.xyz;
    }

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
        { // Deserialization not successful
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

cv::Point3f DroneNavigation::square_point(cv::Point3f center, float width, float s){
    s = fmodf(s, 4*width);
    float si = fmodf(s, width);

    float x_offset, y_offset, z_offset;

    if(s>3*width){
        x_offset = -width/2;
        z_offset = width/2 - si;
    } else if(s>2*width){
        x_offset = width/2 - si;
        z_offset = width/2;
    } else if(s>width){
        x_offset = width/2;
        z_offset = -width/2 + si;
    } else{
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
