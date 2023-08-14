#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void InsectTracker::init(int id, VisionData *visdat, int motion_thresh, int16_t viz_id, bool enable_stereo_viz) {
    _insect_trkr_id = id;
    ItemTracker::init(visdat, motion_thresh, "insect", viz_id);
    expected_radius = 0.01;
    max_radius = 0.03;
    _n_frames_lost = 0;
    enable_draw_stereo_viz = enable_stereo_viz;
}
void InsectTracker::init_logger() {
    logger_fn = data_output_dir  + "log_itrk" + to_string(_insect_trkr_id) + ".csv";
    insectlogger.open(logger_fn, std::ofstream::out);
    insectlogger << "rs_id;elapsed;";
    ItemTracker::init_logger(&insectlogger);
    insectlogger << "fp;hunt_id;light_level;";
    insectlogger << std::endl;
}
void InsectTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}
void InsectTracker::close_log_line() {
    (*_logger) << false_positive_names[false_positive()] << ";";
    (*_logger) << _hunt_id << ";";
    (*_logger) << _visdat->light_level() << ";";
    (*_logger) << '\n';
    _hunt_id = -1; // reset hunt_id after logging, in case the interceptor changes target.

}
void InsectTracker::append_log(double time, unsigned long long frame_number) {
    start_new_log_line(time, frame_number);
    _n_frames_lost++;
    ItemTracker::append_log();
    close_log_line();
}

void InsectTracker::check_false_positive() {
    //there are several known fp's:
    // 1. a 'permanent motion pair', a spot in the motion map that was permanentely changed after the motion map was reset.
    //(this could well have been a moving insect, splitting the blob in a permanent speck where it started at the point of the reset,
    //and the actual flying insect )
    // 2. A blinking / reflective object, possibly caused by 50hz external light sources
    // 3. Camera noise pixels, this noise seems to be stronger with overexposed areas
    // 4. Static objects with actual motion (e.g. a ventilator)
    // 5. A moving plant, e.g. caused by down wash from the drone
    // 6. Reflections from the drone led on reflective surfaces (i.e. the new charging pad)
    // 7. Flickering of and slowly moving vertical wires used to grow certain crops
    // 8. Monsters. Which can be anything big and moving. Like humans, or sweeping lights, or cats, etc...

    //A check whether an object is actually moving through the image seems to be quite robust to filter out 1,4 and possibly 5 and 7:
    //A check whether a detection was tracked for more then a few frames filters out 2 and 3
    if (_track.size() < track_history_max_size) {
        auto wti_0 = _track.at(0).world_item;
        assert(wti_0.valid); // the first ever point tracked should always be valid
        if (_track.size() > 1) {
            if (_world_item.valid) {
                dist_integrator_fp += normf(_world_item.pt - wti_0.pt);
                float tot = dist_integrator_fp / _n_frames_tracked ;
                if (tot < 0.01f && _n_frames_tracked)
                    _fp_static_cnt++;
                else
                    _fp_static_cnt = 0;
            }
        } else {
            _fp_static_cnt = 0;
        }
    } else if (_fp_static_cnt > 3) { // stop tracking this one so that it will be added to the false_positive list in the trackermanager
        _tracking = false;
        _n_frames_lost = n_frames_lost_threshold + 1;
    }

    // a check on big things, or things too far away to be a visible insect handles 8
    if (_world_item.valid) {
        if (_world_item.radius > 3 * max_radius) {
            _fp_too_big_cnt++;
        } else if (_fp_too_big_cnt && _world_item.radius < max_radius && _fp_too_big_cnt <= 30)
            _fp_too_big_cnt--;

        if (_world_item.distance > 6)
            _fp_too_far_cnt++;
        else if (_fp_too_far_cnt && _world_item.distance < 6)
            _fp_too_far_cnt--;
    }
}
tracking::false_positive_type InsectTracker::false_positive() {

    if (!_world_item.valid && _fp_static_cnt > 3)
        return tracking::false_positive_type::fp_static_location;
    else if (_world_item.valid) {
        if (_fp_too_big_cnt > 10 ||  _world_item.radius > 3 * max_radius)
            return tracking::false_positive_type::fp_too_big;
        if (_fp_too_far_cnt > 10 ||  _world_item.distance > 6)
            return tracking::false_positive_type::fp_too_far;
    } else {
        if (_fp_too_big_cnt >= n_frames_lost_threshold || (_fp_too_big_cnt * 2 >= _n_frames_tracked && _n_frames_tracked * 2 >= n_frames_lost_threshold))
            return tracking::false_positive_type::fp_too_big;
        if (_fp_too_far_cnt >= n_frames_lost_threshold || (_fp_too_far_cnt * 2 >= _n_frames_tracked && _n_frames_tracked * 2 >= n_frames_lost_threshold))
            return tracking::false_positive_type::fp_too_far;
    }

    if (!_world_item.valid && _n_frames_tracked < n_frames_lost_threshold && _n_frames)
        return tracking::false_positive_type::fp_short_detection;
    else
        return tracking::false_positive_type::fp_not_a_fp;
}

int terminate = 0;
bool InsectTracker::go_for_terminate() {
    if (_world_item.valid && _world_item.radius * 2.f > pparams.min_hunt_size && _world_item.radius * 2.f < pparams.max_hunt_size) {
        terminate++;
        return true;
    }
    return terminate > 10;
}

void InsectTracker::update(double time) {
    start_new_log_line(time, _visdat->frame_id);
    ItemTracker::update(time);
    check_false_positive();

    if (_tracking) {
        update_prediction(time);
        if (_image_item.valid && _image_item.disparity == _image_item.disparity) {
            min_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
            max_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());
        }
    }

    close_log_line();
}

void InsectTracker::calc_world_item(BlobProps *props, double time [[maybe_unused]]) {
    calc_world_props_blob_generic(props);
    props->world_props.im_pos_ok = true;
    props->world_props.valid = props->world_props.disparity_in_range ;
    if (!props->world_props.bkg_check_ok && props->world_props.distance > 1.2f * props->world_props.distance_bkg)
        //We need to allow for marging for moth flying through the crops, but at some point the chance of
        //some tracking problem too great. Not really sure when though, so this cut-off number 1.2 may need further tuning.
        //The problem that used to be solved using the bkg_check_ok was tracking of shadows. But current set up that does not seem
        // to be a concern anymore.
        props->world_props.valid = false;

    if (_blobs_are_fused_cnt > pparams.fps) // if the insect and drone are fused, the drone is accelerating through it and should become seperate again within a limited time
        props->world_props.valid = false;
}

bool InsectTracker::check_ignore_blobs(BlobProps *props) { return this->check_ignore_blobs_generic(props);}

bool InsectTracker::delete_me() {
    if (_blobs_are_fused_cnt && !_image_item.blob_is_fused) {
        _n_frames_lost = 0;
        _blobs_are_fused_cnt = 0;
    }

    if ((_n_frames_lost > n_frames_lost_threshold && !_image_item.blob_is_fused)) {
        if (initialized_logger) {
            (*_logger) << std::flush;
            _logger->close();
        }
        initialized = false;
        initialized_logger = false;
        auto fp = false_positive();
        if (fp == fp_short_detection || fp == fp_static_location)
            remove(logger_fn.c_str());
        return true;
    } else
        return false;
}

float InsectTracker::score_threshold() {
    if (_blobs_are_fused_cnt)
        return _score_threshold * 2;
    return std::clamp(_score_threshold + _n_frames_lost * 0.3f * _score_threshold, 0.f, 1.5f * _score_threshold);
}

}
