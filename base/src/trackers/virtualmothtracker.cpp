#include "virtualmothtracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void VirtualMothTracker::init(int id, mothbehavior mothbehavior_type, VisionData *visdat, DroneController *dctrl) {
    _id = id;
    _visdat = visdat;
    _dctrl = dctrl;
    _n_frames_lost = 0;
    _name = "replay"; //This is also an prefix in the log.csv. For replay-support the log is saved as replay-moth
    behavior_type = mothbehavior_type;
    track_history_max_size = pparams.fps;
    init_logger();
    initialized = true;
    insect_pos = {2., -0.4, -2};
    insect_vel = {-1., 0, 0};
}

void VirtualMothTracker::init_logger() {
    //writing of new log:
    _logger = new std::ofstream(); // FIXME: use std::shared_ptr?
    logger_fn = data_output_dir  + "log_rtrkr" + to_string(_id) + ".csv";
    (*_logger).open(logger_fn, std::ofstream::out);
    (*_logger) << "RS_ID;time;";
    ItemTracker::init_logger(_logger);
    (*_logger) << std::endl;
}

void VirtualMothTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void VirtualMothTracker::update(double time) {
    start_new_log_line(time, _visdat->frame_id);
    update_behavior_based(_visdat->frame_id, time);
    _n_frames++;
    _n_frames_tracking++;
    _n_frames_tracked++;
    ItemTracker::append_log();
    (*_logger) << '\n';
}

void VirtualMothTracker::update_behavior_based(unsigned long long frame_number, double time) {
    if (start_time <= 0)
        start_time = time;

    if (_dctrl->auto_throttle > 400) {
        escape_triggered = true;
    }
    if (escape_triggered) {
        insect_vel += cv::Point3f(0., -2, 0.) / static_cast<float>(pparams.fps);
    }

    _n_frames_lost = 0;
    _n_frames_tracking++;
    _tracking = true;

    insect_pos += 1. / pparams.fps * insect_vel;
    cv::Point3f insect_im = world2im_3d(insect_pos, _visdat->Qfi, _visdat->camera_roll, _visdat->camera_pitch);
    _image_item = ImageItem(insect_im.x, insect_im.y, insect_im.z, frame_number);
    _image_item.valid = true;
    _image_predict_item = ImagePredictItem(cv::Point3f(insect_im.x, insect_im.y, insect_im.z), 1, 255, frame_number);
    _image_predict_item.valid = _image_predict_item.pt.x > 0;

    WorldItem w;
    w.image_item = _image_item;
    w.valid = true;
    w.pt.x = insect_pos.x;
    w.pt.y = insect_pos.y;
    w.pt.z = insect_pos.z;
    w.distance = norm(w.pt);
    w.image_item.x = std::clamp(static_cast<int>(w.image_item.x), 0, IMG_W);
    w.image_item.y = std::clamp(static_cast<int>(w.image_item.y), 0, IMG_H);
    w.distance_bkg = _visdat->depth_background_mm.at<float>(w.image_item.y, w.image_item.x);
    _world_item = w;

    TrackData data;
    data.pos_valid = true;
    data.state.pos.x = insect_pos.x;
    data.state.pos.y = insect_pos.y;
    data.state.pos.z = insect_pos.z;
    data.state.spos.x = insect_pos.x;
    data.state.spos.y = insect_pos.y;
    data.state.spos.z = insect_pos.z;
    data.state.vel.x = insect_vel.x;
    data.state.vel.y = insect_vel.y;
    data.state.vel.z = insect_vel.z;
    data.state.acc.x = 0;
    data.state.acc.y = 0;
    data.state.acc.z = 0;
    data.time = time;
    _track.push_back(data);
    data.world_item = w;
    data.predicted_image_item = _image_predict_item;

    if (time - start_time > 5)
        _delete_me = true;

    cleanup_history();
}

bool VirtualMothTracker::delete_me() {
    if (_delete_me) {
        _logger->close();
        return true;
    }
    return false;
}

}
