#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void InsectTracker::init(int id, VisionData *visdat, int motion_thresh, int16_t viz_id, bool enable_stereo_viz) {
    _insect_trkr_id = id;
    ItemTracker::init(visdat,motion_thresh,"insect",viz_id);
    expected_radius = 0.01;
    max_size = 0.03;
    _n_frames_lost = 0;
    enable_draw_stereo_viz = enable_stereo_viz;
}
void InsectTracker::init_logger() {
    logger_fn = data_output_dir  + "log_itrk" + to_string(_insect_trkr_id) + ".csv";
    insectlogger.open(logger_fn,std::ofstream::out);
    insectlogger << "RS_ID;time;";
    ItemTracker::init_logger(&insectlogger);
    insectlogger << "fp;";
    insectlogger << std::endl;
}
void InsectTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}
void InsectTracker::close_log_line() {
    (*_logger) << false_positive_names[false_positive()] << ";";
    (*_logger) << '\n';
}
void InsectTracker::append_log(double time, unsigned long long frame_number) {
    start_new_log_line(time,frame_number);
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

    //A check whether an object is actually moving through the image seems to be quite robust to filter out 1,4 and possibly 5 and 7:
    //A check whether a detection was tracked for more then a few frames filters out 2 and 3

    if (_track.size() < track_history_max_size) {
        auto wti_0 = _track.at(0).world_item;
        assert(wti_0.valid); // the first ever point tracked should always be valid
        if (_track.size()>1) {
            if (_world_item.valid)
                dist_integrator_fp += normf(_world_item.pt - wti_0.pt);
            float tot = dist_integrator_fp/(_n_frames_tracked-1);
            if (tot < 0.01f && _n_frames_tracked > 1)
                _fp_cnt++;
            else
                _fp_cnt = 0;
        } else {
            _fp_cnt = 0;
        }
    } else if (_fp_cnt> 3) {
        _tracking = false; //bye
        _n_frames_lost = n_frames_lost_threshold;
    }
}
tracking::false_positive_type InsectTracker::false_positive() {
    if (_fp_cnt>3)
        return tracking::false_positive_type::fp_static_location;
    else if ( properly_tracking() && _n_frames_tracked < n_frames_lost_threshold/2 )
        return tracking::false_positive_type::fp_short_detection;
    else if ( properly_tracking() && _n_frames_tracked < _n_frames/4 )
        return tracking::false_positive_type::fp_short_detection;
    else if ( !_tracking && _n_frames_tracked < n_frames_lost_threshold && _n_frames_tracked)
        return tracking::false_positive_type::fp_short_detection;
    else
        return tracking::false_positive_type::fp_not_a_fp;
}

void InsectTracker::update(double time) {
    start_new_log_line(time,_visdat->frame_id);
    ItemTracker::update(time);
    check_false_positive();

    if (_tracking) {
        update_prediction(time);
        if (_image_item.valid) {
            min_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity))-5,params.min_disparity.value(),params.max_disparity.value());
            max_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity))+5,params.min_disparity.value(),params.max_disparity.value());
        }
    }

    close_log_line();
}

void InsectTracker::calc_world_item(BlobProps * props, double time [[maybe_unused]]) {
    calc_world_props_blob_generic(props);
    props->world_props.im_pos_ok = true;
    props->world_props.valid = props->world_props.disparity_in_range & props->world_props.radius_in_range;
    if (!props->world_props.bkg_check_ok && props->world_props.distance>1.2f*props->world_props.distance_bkg)
        //We need to allow for marging for moth flying through the crops, but at some point the chance of
        //some tracking problem too great. Not really sure when though, so this cut-off number 1.2 may need further tuning.
        //The problem that used to be solved using the bkg_check_ok was tracking of shadows. But current set up that does not seem
        // to be a concern anymore.
        props->world_props.valid = false;

    if (_blobs_are_fused_cnt > pparams.fps) // if the insect and drone are fused, the drone is accelerating through it and should become seperate again within a limited time
        props->world_props.valid = false;
}

bool InsectTracker::check_ignore_blobs(BlobProps * props) { return this->check_ignore_blobs_generic(props);}

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
        if (false_positive())
            remove(logger_fn.c_str());
        return true;
    } else
        return false;
}

}
