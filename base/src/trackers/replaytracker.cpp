#include "replaytracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void ReplayTracker::init(int id,std::string file, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    _n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    logreader.init(file);
    init_logger();
    initialized = true;
}

void ReplayTracker::init(int id,logging::InsectReader log, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    _n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    init_logger();
    logreader = log;
    initialized = true;
}

void ReplayTracker::init_logger() {
    _logger = new std::ofstream();
    logger_fn = data_output_dir  + "log_rtrkr" + to_string(_id) + ".csv";
    (*_logger).open(logger_fn,std::ofstream::out);
    (*_logger) << "RS_ID;time;";
    ItemTracker::init_logger(_logger);
    (*_logger) << std::endl;
}

void ReplayTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void ReplayTracker::update_from_log(unsigned long long frame_number,double time) {
    auto log = logreader.current_entry;
    logreader.increase_frame_number();

    _n_frames++;
    _n_frames_lost = log.ins_n_frames_lost;
    _n_frames_tracking = log.ins_n_frames_tracking;
    if(_n_frames_tracking>0)
        _n_frames_tracked++;
    _tracking = log.ins_foundL;
    bool valid = (log.ins_im_x >= 0 && log.ins_im_y >= 0);

    _image_item = ImageItem (log.ins_im_x,log.ins_im_y,log.ins_disparity,frame_number);
    _image_item.valid = valid;
    _image_predict_item = ImagePredictItem(cv::Point3f(log.ins_pred_im_x,log.ins_pred_im_y,log.ins_disparity),1,255,frame_number);
    _image_predict_item.valid = _image_predict_item.pt.x > 0 && _tracking;

    WorldItem w;
    w.image_item = _image_item;
    w.valid = valid;
    w.pt.x = log.ins_pos_x;
    w.pt.y = log.ins_pos_y;
    w.pt.z = log.ins_pos_z;
    w.distance = norm(w.pt);
    w.image_item.x = std::clamp(static_cast<int>(w.image_item.x),0,IMG_W);
    w.image_item.y = std::clamp(static_cast<int>(w.image_item.y),0,IMG_H);
    w.distance_bkg = _visdat->depth_background_mm.at<float>(w.image_item.y,w.image_item.x);
    _world_item = w;

    TrackData data;
    data.pos_valid = valid;
    data.state.pos.x = log.ins_pos_x;
    data.state.pos.y = log.ins_pos_y;
    data.state.pos.z = log.ins_pos_z;
    data.state.spos.x = log.ins_spos_x;
    data.state.spos.y = log.ins_spos_y;
    data.state.spos.z = log.ins_spos_z;
    data.state.vel.x = log.ins_svel_x;
    data.state.vel.y = log.ins_svel_y;
    data.state.vel.z = log.ins_svel_z;
    data.state.acc.x = log.ins_sacc_x;
    data.state.acc.y = log.ins_sacc_y;
    data.state.acc.z = log.ins_sacc_z;
    data.time = time;
    data.predicted_image_item = _image_predict_item;
    data.world_item = w;
    _track.push_back(data);

    cleanup_history();
}

void ReplayTracker::update(double time) {
    start_new_log_line(time,_visdat->frame_id);
    update_from_log(_visdat->frame_id,time);
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
