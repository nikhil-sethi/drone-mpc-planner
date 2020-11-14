#include "blinktracker.h"

namespace tracking {

bool BlinkTracker::init(int id, VisionData *visdat, int motion_thresh, int16_t viz_id) {
    _blink_trkr_id = id;
    disparity_filter_rate = 0.5;
    pos_smth_width = pparams.fps/15;
    vel_smth_width = pparams.fps/10;
    acc_smth_width = pparams.fps/5;
    skip_wait_smth_spos = false;

    ItemTracker::init(visdat,motion_thresh,"blink",viz_id);

    max_size = dparams.radius*3;
    expected_radius = dparams.radius;
    _n_frames_lost = 1;
    n_frames_lost_threshold = 120;

    return false;
}

void BlinkTracker::init_logger() {
    std::string logger_fn;
    logger_fn = data_output_dir  + "log_btrk" + to_string(_blink_trkr_id) + ".csv";
    blinklogger.open(logger_fn,std::ofstream::out);
    blinklogger << "RS_ID;time;";
    ItemTracker::init_logger(&blinklogger);
    blinklogger << std::endl;
}

void BlinkTracker::update(double time) {
    switch (_blinking_drone_status) {
    case bds_start: {
        _tracking = true;
        _blinking_drone_status = bds_searching;
        [[fallthrough]];
    } case bds_restart_search: {
        attempts++;
        _blinking_drone_status = bds_searching;
        [[fallthrough]];
    } case bds_searching: {
        if (attempts > 4) {
            _blinking_drone_status = bds_failed;
            fail_time_start = time;
        }
        ItemTracker::update(time);
        if (_n_frames_lost == 0) {
            _blinking_drone_status = bds_1_blink_off;
            blink_time_start = time;
            _score_threshold = 100; // increase score threshold after first sightings, so that the tracker rejects moving blobs
        }
        break;
    } case bds_1_blink_off: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_1_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_2_blink_off: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_2_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_3_blink_off: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_3_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_4_blink_off: {
        attempts = 0;
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_4_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_5_blink_off: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_5_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_6_blink_off_calib: {
        _blinking_drone_status = bds_6_blink_off;
        [[fallthrough]];
    } case bds_6_blink_off: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_tracking == 0);
        break;
    } case bds_6_blink_on: {
        ItemTracker::update(time);
        _blinking_drone_status = detect_blink(time, _n_frames_lost == 0);
        break;
    } case bds_found: {
        append_log(); // no tracking needed in this stage
        break;
    } case bds_failed: {
        //just keep tracking it until it the item is lost, or time out after 2 seconds
        if (time - fail_time_start > 2)
            _blinking_drone_status = bds_failed_delete_me;
        append_log(); // no tracking needed in this stage
        break;
    } case bds_failed_delete_me: {
        append_log(); // no tracking needed in this stage
        break;
    }
    }

    clean_ignore_blobs(time);
    if (_world_item.valid) {
        last_known_valid_pos = _world_item.pt;
        last_known_valid_pos_valid = true;
    }
}

BlinkTracker::blinking_drone_states BlinkTracker::detect_blink(double time, bool found) {
    const float margin = 0.75f * dparams.blink_period; // the blinking is not a hard on/off, but rather a dimming operation so we need a big margin and blink more often
    float blink_period = static_cast<float>(time - blink_time_start);
    if (found) {
        if ( blink_period > dparams.blink_period - margin && blink_period < dparams.blink_period+margin) {
            blink_time_start = time;
            int tmp  =static_cast<int>(_blinking_drone_status)+1;
            return static_cast<blinking_drone_states>(tmp);
        } else {
            return bds_restart_search;
        }
    } else if (!found && blink_period > dparams.blink_period + margin) {
        return bds_restart_search;
    }
    return _blinking_drone_status;
}

void BlinkTracker::calc_world_item(BlobProps * props, double time [[maybe_unused]]) {

    const float max_world_dist = 0.05f; // pre-sift blobs using the image coordinates, to prevent having to calculate the stereo_match
    int max_im_dist = world2im_dist(last_known_valid_pos,max_world_dist,_visdat->Qfi,_visdat->camera_angle);
    if (last_known_valid_pos_valid && normf(cv::Point2f(props->x,props->y) - _image_item.pt()) > max_im_dist) {
        props->world_props.im_pos_ok = false;
        props->world_props.valid = false;
    } else {
        calc_world_props_blob_generic(props,false);
        props->world_props.im_pos_ok = true;
        props->world_props.valid = props->world_props.disparity_in_range && props->world_props.radius_in_range;
    }
}
bool BlinkTracker::check_ignore_blobs(BlobProps * props) {

    std::vector<IgnoreBlob> filtered_ignores_for_me;
    for (auto ign : ignores_for_me) {
        if (ign.ignore_type != tracking::IgnoreBlob::landing_spot && ign.ignore_type != tracking::IgnoreBlob::takeoff_spot) {
            filtered_ignores_for_me.push_back(ign);
        }
    }
    ignores_for_me = filtered_ignores_for_me;

    return this->check_ignore_blobs_generic(props);
}

//Removes all ignore points which timed out
void BlinkTracker::clean_ignore_blobs(double time) {
    std::vector<IgnoreBlob> new_ignores_for_insect_tracker;
    for (uint i = 0; i < ignores_for_other_trkrs.size(); i++) {
        if (ignores_for_other_trkrs.at(i).was_used && ignores_for_other_trkrs.at(i).invalid_after>=0)
            ignores_for_other_trkrs.at(i).invalid_after += 1./pparams.fps;
        ignores_for_other_trkrs.at(i).was_used = false;
        if (ignores_for_other_trkrs.at(i).invalid_after > time || ignores_for_other_trkrs.at(i).invalid_after<0)
            new_ignores_for_insect_tracker.push_back(ignores_for_other_trkrs.at(i));
    }
    ignores_for_other_trkrs= new_ignores_for_insect_tracker;
}

}
