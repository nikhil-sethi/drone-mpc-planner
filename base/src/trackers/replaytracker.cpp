#include "replaytracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void ReplayTracker::init(int id, std::string file, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    _n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    logreader.init(file);
    max_radius = 0.03;
    init_logger();
    initialized = true;
}

void ReplayTracker::init(int id, logging::InsectReader log, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    _n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    max_radius = 0.03;
    init_logger();
    logreader = log;
    initialized = true;
}

void ReplayTracker::init_logger() {
    _logger = new std::ofstream();
    logger_fn = data_output_dir  + "log_rtrkr" + to_string(_id) + ".csv";
    (*_logger).open(logger_fn, std::ofstream::out);
    (*_logger) << "rs_id;elapsed;";
    ItemTracker::init_logger(_logger);
    (*_logger) << std::endl;
}

void ReplayTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void ReplayTracker::inject_log(unsigned long long frame_number, double time) {
    auto log = logreader.current_entry;
    logreader.increase_frame_number();

    _n_frames++;
    _n_frames_lost = log.n_frames_lost;
    _n_frames_tracking = log.n_frames_tracking;
    if (_n_frames_tracking > 0)
        _n_frames_tracked++;
    _tracking = log.foundL;
    bool valid = (log.im_x >= 0 && log.im_y >= 0);

    _image_item = ImageItem(log.im_x, log.im_y, log.disparity, frame_number);
    _image_item.valid = valid;
    _image_predict_item = ImagePredictItem(cv::Point3f(log.pred_im_x, log.pred_im_y, log.disparity), 1, 255, frame_number);
    _image_predict_item.valid = _image_predict_item.pt.x > 0 && _tracking;

    WorldItem w;
    w.image_item = _image_item;
    w.valid = valid;
    w.pt.x = log.pos_x;
    w.pt.y = log.pos_y;
    w.pt.z = log.pos_z;
    w.distance = norm(w.pt);
    w.radius = 0.015;
    w.image_item.x = std::clamp(static_cast<int>(w.image_item.x), 0, IMG_W);
    w.image_item.y = std::clamp(static_cast<int>(w.image_item.y), 0, IMG_H);
    w.distance_bkg = _visdat->depth_background_mm.at<float>(w.image_item.y, w.image_item.x);
    _world_item = w;

    TrackData data;
    data.pos_valid = valid;
    data.state.pos.x = log.pos_x;
    data.state.pos.y = log.pos_y;
    data.state.pos.z = log.pos_z;
    data.state.spos.x = log.spos_x;
    data.state.spos.y = log.spos_y;
    data.state.spos.z = log.spos_z;
    data.state.vel.x = log.svel_x;
    data.state.vel.y = log.svel_y;
    data.state.vel.z = log.svel_z;
    data.state.acc.x = log.sacc_x;
    data.state.acc.y = log.sacc_y;
    data.state.acc.z = log.sacc_z;
    data.time = time;
    data.predicted_image_item = _image_predict_item;
    data.world_item = w;
    _track.push_back(data);

    cleanup_history();
}

void ReplayTracker::update(double time) {
    start_new_log_line(time, _visdat->frame_id);
    inject_log(_visdat->frame_id, time);
    ItemTracker::append_log();
    (*_logger) << '\n';
}

bool ReplayTracker::delete_me() {
    if (logreader.done()) {
        _logger->close();
        return true;
    }
    return false;
}


}
