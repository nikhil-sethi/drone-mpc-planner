#include "pats.h"

void Patser::init(std::ofstream *logger, int rc_id, RC *rc, std::string replay_dir, Cam *cam, VisionData *visdat,  BaseboardLink *baseboard_link) {
    _logger = logger;
    (*_logger) << "pats_state_str;pats_state;";
    _visdat = visdat;
    _baseboard_link = baseboard_link;
    _rc = rc;

    trackers.init(logger, replay_dir, visdat, &interceptor);
    if (pparams.op_mode == op_mode_x) {
        flight_area.init(replay_dir, cam);
        drone.init(logger, rc_id, rc, &trackers, visdat, &flight_area, &interceptor, baseboard_link);
        FlightAreaConfig *_config = flight_area.flight_area_config(bare);
        interceptor.init(&trackers, visdat, &flight_area, &drone, _config);
    }
}

void Patser::init_insect_replay() {
    _pats_state = pats_c;
    trackers.mode(tracking::TrackerManager::t_c);
}
void Patser::init_flight_replay(std::string replay_dir, int flight_id) {
    drone.init_flight_replay(replay_dir, flight_id);
    _pats_state = pats_x;
    trackers.mode(tracking::TrackerManager::t_x);
}

void Patser::update(double time) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start = std::chrono::high_resolution_clock::now();
#endif
    (*_logger) << state_str() << ";" << static_cast<uint16_t>(_pats_state) << ";";
    switch (_pats_state) {
        case pats_init: {
                if (time_first_frame < 0)
                    time_first_frame = time;
                _visdat->reset_motion_integration();
                trackers.mode(tracking::TrackerManager::t_idle);
                trackers.update(time);
                if (time - time_first_frame > 1.5) { // skip first second or so due to auto exposure settling
                    _visdat->disable_cloud_rejection = false;
                    _visdat->enable_noise_map_calibration(duration_motion_calibration);
                    drone.control.led_strength(_visdat->light_level());
                    time_start_motion_calibration = time;
                    _pats_state = pats_calibrating_motion;
                }
                if (pparams.op_mode == op_mode_x)
                    drone.dummy_log();
                break;
        } case pats_calibrating_motion: {
                maintain_motion_map(time);
                trackers.update(time);
                if (static_cast<float>(time - time_start_motion_calibration) > duration_motion_calibration && _visdat->motion_filtered_noise_initialized()) {
                    _visdat->save_maps(data_output_dir);
                    if (pparams.op_mode == op_mode_c) {
                        trackers.mode(tracking::TrackerManager::t_c);
                        _pats_state = pats_c;
                        communicate_state(es_pats_c);
                    } else if (pparams.op_mode == op_mode_x) {
                        communicate_state(es_pats_x);
                        _pats_state = pats_x;
                    }
                }
                if (pparams.op_mode == op_mode_x)
                    drone.dummy_log();
                break;
        } case pats_c: {
                maintain_motion_map(time);
                trackers.update(time);
                break;
        } case pats_x: {
                if (!drone.in_flight())
                    maintain_motion_map(time);

                trackers.update(time);
                interceptor.update(drone.control.at_base(), time);
                drone.update(time);
                break;
            }
    }
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    std::cout << "timing (update_patser): " << (t_end - t_start).count() * 1e-6 << "ms" << std::endl;
#endif
}

void Patser::maintain_motion_map(double time) {
    float time_since_tracking_nothing = trackers.tracking_anything_duration(time);
    if (time_since_tracking_nothing > 20 || time_since_tracking_nothing == 0 || !_visdat->motion_filtered_noise_initialized())
        _visdat->maintain_noise_maps();
}

void Patser::close() {
    interceptor.close();
    trackers.close();
    drone.close();
}
