#include "pats.h"

void Patser::init(std::ofstream *logger, int rc_id, RC *rc, std::string replay_dir, Cam *cam, VisionData *visdat,  Baseboard *baseboard) {
    _logger = logger;
    (*_logger) << "pats_state;";
    _visdat = visdat;
    _baseboard = baseboard;
    _rc = rc;

    trackers.init(logger, replay_dir, visdat, &interceptor);
    if (pparams.op_mode == op_mode_x) {
        flight_area.init(replay_dir, cam);
        drone.init(logger, rc_id, rc, &trackers, visdat, &flight_area, &interceptor, baseboard);
        interceptor.init(&trackers, visdat, &flight_area, &drone);
    }
}

void Patser::update(double time) {
    (*_logger) << state_str() << ";";
    switch (_pats_state) {
        case pats_init: {
                if (time_first_frame < 0)
                    time_first_frame = time;
                _visdat->reset_motion_integration();
                trackers.mode(tracking::TrackerManager::t_idle);
                trackers.update(time);
                if (time - time_first_frame > 1.5) { // skip first second or so due to auto exposure settling
                    _visdat->enable_noise_map_calibration(duration_motion_calibration);
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
                    if (pparams.op_mode == op_mode_c) {
                        trackers.mode(tracking::TrackerManager::t_c);
                        _pats_state = pats_c;
                    } else if (pparams.op_mode == op_mode_x)
                        _pats_state = pats_x;
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
}

void Patser::maintain_motion_map(double time) {
    float time_since_tracking_nothing = trackers.tracking_anything_duration(time);
    if (time_since_tracking_nothing > 20 || time_since_tracking_nothing == 0 || !_visdat->motion_filtered_noise_initialized())
        _visdat->maintain_noise_maps();
}

void Patser::close() {
    trackers.close();
    drone.close();
}