#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

static std::ofstream insectlogger;

void InsectTracker::init(int id, VisionData *visdat, int16_t viz_id) {
    _insect_trkr_id = id;
    std::string logger_fn = data_output_dir  + "log_itrk" + to_string(id) + ".csv";
    insectlogger.open(logger_fn,std::ofstream::out);
    insectlogger << "RS_ID;time;";
    ItemTracker::init(&insectlogger,visdat,"insect",viz_id);
    insectlogger << std::endl;
    n_frames_lost = 0;
}
void InsectTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void InsectTracker::append_log(double time, unsigned long long frame_number) {
    start_new_log_line(time,frame_number);
    ItemTracker::append_log();
    (*_logger) << '\n';
}

void InsectTracker::check_false_positive() {

    //check for a 'permanent motion pair', a spot in the motion map that was permanentely changed after the motion map was reset.
    //(this could well have been a moving insect, splitting the blob in a permanent speck where it started at the point of the reset,
    //and the actual flying insect )
    if (path.size()>1) {
        float tot = 0;
        uint cnt = 0;
        for (auto wi : path) {
            if (wi.valid) {
                tot += normf(wi.pt - _world_item.pt);
                cnt++;
            }
        }
        tot/=cnt;
        if (tot < 0.03f)
            _fp++;
        else
            _fp = 0;
    } else {
        _fp = 0;
    }

}

void InsectTracker::update(double time) {

    start_new_log_line(time,_visdat->frame_id);

    check_false_positive();
    ItemTracker::update(time);

    if (!_tracking) {
        predicted_image_path.clear();
        path.clear();
    } else {
        update_prediction(time);
        if (_image_item.valid) {
            min_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity))-5,params.min_disparity.value(),params.max_disparity.value());
            max_disparity = std::clamp(static_cast<int>(roundf(_image_item.disparity))+5,params.min_disparity.value(),params.max_disparity.value());
        }
    }
    (*_logger) << '\n';
}

void InsectTracker::calc_world_item(BlobProps * pbs, double time [[maybe_unused]]) {
    calc_world_props_blob_generic(pbs,false);
    pbs->world_props.valid = pbs->world_props.bkg_check_ok && pbs->world_props.disparity_in_range & pbs->world_props.radius_in_range;

    if (_blobs_are_fused_cnt > 1 * pparams.fps) // if the insect and drone are fused, the drone is accelerating through it and should become seperate again within a limited time
        pbs->world_props.valid = false;
}

bool InsectTracker::check_ignore_blobs(BlobProps * pbs) {
    return this->check_ignore_blobs_generic(pbs);
}

}
