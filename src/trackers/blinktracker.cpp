#include "blinktracker.h"

namespace tracking {

static std::ofstream dummy;

bool BlinkTracker::init(VisionData *visdat, int16_t viz_id) {

    // disable logging for the blink tracker. #67
    dummy.open("/dev/null",std::ofstream::out);
    dummy.close();
    ItemTracker::init(&dummy,visdat,"blink",viz_id);

    n_frames_lost = 1;
    n_frames_lost_threshold = 120;
    return false;
}

void BlinkTracker::track(double time) {
    switch (_blinking_drone_status) {
    case bds_start: {
        _tracking = true;
        path.clear();
        predicted_image_path.clear();
        _blinking_drone_status = bds_searching;
        [[fallthrough]];
    } case bds_restart_search: {
        attempts++;
        _blinking_drone_status = bds_searching;
        [[fallthrough]];
    } case bds_searching: {
        if (attempts > 4)
            _blinking_drone_status = bds_failed;
        ItemTracker::track(time);
        if (n_frames_lost == 0) {
            _blinking_drone_status = bds_1_blink_off;
            blink_time_start = time;
        }
        break;
    } case bds_1_blink_off: {
        _score_threshold = 250; // increase score threshold after first sightings, so that the tracker rejects moving blobs
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_1_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_2_blink_off: {
        attempts = 0;
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_2_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_3_blink_off_calib: {
        _visdat->enable_background_motion_map_calibration(dparams.blink_period*0.8f);  //0.8 to prevent picking up the upcoming blink in the background calib
        _blinking_drone_status = bds_3_blink_off;
        [[fallthrough]];
    } case bds_3_blink_off: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
        break;
    } case bds_3_blink_on: {
        ItemTracker::track(time);
        _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
        break;
    } case bds_found: {
        append_log(); // no tracking needed in this stage
        break;
    } case bds_failed: {
        //just keep tracking it until it the item is lost;
        ItemTracker::track(time);
        append_log(); // no tracking needed in this stage
    }
    }

    if (_image_item.valid && _blinking_drone_status > bds_searching ){
        _visdat->exclude_drone_from_motion_fading(_image_item.pt()*pparams.imscalef,static_cast<int>(roundf(_image_item.size*1.2f*pparams.imscalef))); // TODO: does not work properly
    }

    clean_ignore_blobs(time);
}

BlinkTracker::blinking_drone_states BlinkTracker::detect_blink(double time, bool found) {
    const float margin = dparams.blink_period/3.f;
    if (Last_track_data().vel_valid && norm(Last_track_data().vel()) > 0.3)
        return bds_restart_search;
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

void BlinkTracker::calc_world_item(BlobProps * pbs, double time [[maybe_unused]]){
    calc_world_props_blob_generic(pbs);
    pbs->world_props.valid = pbs->world_props.disparity_in_range && pbs->world_props.radius_in_range;
}
bool BlinkTracker::check_ignore_blobs(BlobProps * pbs) {
    return this->check_ignore_blobs_generic(pbs);
}

//Removes all ignore points which timed out
void BlinkTracker::clean_ignore_blobs(double time){
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
