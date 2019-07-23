#include "blinktracker.h"

std::ofstream dummy;

bool BlinkTracker::init(VisionData *visdat) {

    // disable logging for the blink tracker. #67
    dummy.open("/dev/null",std::ofstream::out);
    dummy.close();
    ItemTracker::init(&dummy,visdat,"blink");

    n_frames_lost = 1;
    n_frames_lost_threshold = 120;
    return false;
}
void BlinkTracker::init_settings() {
    settings.score_threshold = 0.1;

    settings.min_disparity = 1;
    settings.background_subtract_zone_factor = 97;
}

void BlinkTracker::track(double time) {
    switch (_blinking_drone_status) {
    case bds_start: {
        path.clear();
        predicted_image_path.clear();
        _blinking_drone_status = bds_reset_bkg;
        ItemTracker::append_log(); // write a dummy entry
        break;
    } case bds_reset_bkg: {
        _blinking_drone_status = bds_searching; // -> wait 1 frame
        ItemTracker::append_log(); // write a dummy entry
        break;
    } case bds_searching: {
#ifdef MANUAL_DRONE_LOCATE
        append_log();
        _blinking_drone_status = bds_found;
        break;
#endif
        ItemTracker::track(time);
        if (n_frames_lost == 0) {
            _blinking_drone_status = bds_1_blink_off;
            blink_time_start = time;
        }
        break;
    } case bds_1_blink_off: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_1_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_2_blink_off: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_2_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_3_blink_off_calib: {
        _visdat->enable_background_motion_map_calibration(bind_blink_time*0.8);  //0.8 to prevent picking up the upcoming blink in the background calib
        _blinking_drone_status = bds_3_blink_off;
    } FALLTHROUGH_INTENDED; case bds_3_blink_off: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_3_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_dedicated_calib: {
        append_log();
        ignores_for_other_trkrs.push_back(IgnoreBlob(_image_item.pt(),time+2, IgnoreBlob::blink_spot)); //prevent any residual blinkiness being picked up
#ifndef MANUAL_DRONE_LOCATE
        _blinking_drone_status = bds_found;
        break;
#endif
        _visdat->enable_background_motion_map_calibration(5.f);
        manual_calib_time_start = time;
        _blinking_drone_status = bds_calib_wait;
        break;
    } case bds_calib_wait: {
        append_log();
        if (time  - manual_calib_time_start > 5.1)
            _blinking_drone_status = bds_found;
        break;
    } case bds_found: {
        append_log(); // no tracking needed in this stage
#ifdef MANUAL_DRONE_LOCATE
        _enable_roi = true;
        //TMP solution:
        _drone_blink_world_location.x = 0.190292642;
        _drone_blink_world_location.y = -1.64084888;
        _drone_blink_world_location.z = -1.32899487;
        //write to xml
        serialize_calib();
        break;
#endif
        break;
    }
    }
    clean_ignore_blobs(time);
}

BlinkTracker::blinking_drone_states BlinkTracker::detect_blink(double time, bool found) {
    double blink_period = time - blink_time_start;
    if (found) {
        if ( blink_period > bind_blink_time - 0.1 && blink_period < bind_blink_time+0.1) {
            blink_time_start = time;
            int tmp  =static_cast<int>(_blinking_drone_status)+1;
            return static_cast<blinking_drone_states>(tmp);
        } else {
            return bds_searching;
        }
    } else if (!found && blink_period > bind_blink_time +0.1) {
        return bds_searching;
    }
    return _blinking_drone_status;
}

ItemTracker::BlobWorldProps BlinkTracker::calc_tmp_world_item(BlobProps * pbs){
    ItemTracker::BlobWorldProps wbp = calc_world_props_blob_generic(pbs);
    wbp.valid = wbp.disparity_in_range;
    return wbp;
}
bool BlinkTracker::check_ignore_blobs(BlobProps * pbs, uint id __attribute__((unused))) {
    return this->check_ignore_blobs_generic(pbs);
}

//Removes all ignore points which timed out
void BlinkTracker::clean_ignore_blobs(double time){
    std::vector<IgnoreBlob> new_ignores_for_insect_tracker;
    for (uint i = 0; i < ignores_for_other_trkrs.size(); i++) {
        if (ignores_for_other_trkrs.at(i).was_used && ignores_for_other_trkrs.at(i).invalid_after>=0)
            ignores_for_other_trkrs.at(i).invalid_after += 1./VIDEOFPS;
        ignores_for_other_trkrs.at(i).was_used = false;
        if (ignores_for_other_trkrs.at(i).invalid_after > time || ignores_for_other_trkrs.at(i).invalid_after<0)
            new_ignores_for_insect_tracker.push_back(ignores_for_other_trkrs.at(i));
    }
    ignores_for_other_trkrs= new_ignores_for_insect_tracker;
}
