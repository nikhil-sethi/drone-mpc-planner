#include "replaytracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void ReplayTracker::init(int id,std::string file, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    logreader.init(file);
    init_logger();
    initialized = true;
}

void ReplayTracker::init(int id,logging::InsectReader log, VisionData *visdat) {
    _id = id;
    _visdat = visdat;
    n_frames_lost = 0;
    _name = "replay";
    track_history_max_size = pparams.fps;
    init_logger();
    logreader = log;
    initialized = true;
}

void ReplayTracker::init_logger() {
    //writing of new log:
    _logger = new std::ofstream(); // FIXME: use std::shared_ptr?
    std::string logger_fn;
    logger_fn = data_output_dir  + "log_rtrkr" + to_string(_id) + ".csv";
    (*_logger).open(logger_fn,std::ofstream::out);
    (*_logger) << "RS_ID;time;";
    ItemTracker::init_logger();
    (*_logger) << std::endl;
}

void ReplayTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void ReplayTracker::update_from_log(unsigned long long frame_number,double time) {
    auto log = logreader.current_entry;
    logreader.increase_frame_number();

    bool valid = (log.ins_im_x >= 0 && log.ins_im_y >= 0);
    _image_item = ImageItem (log.ins_im_x/pparams.imscalef,log.ins_im_y/pparams.imscalef,log.ins_disparity,frame_number);
    _image_predict_item = ImagePredictItem(cv::Point2f(log.ins_pred_im_x/pparams.imscalef,log.ins_pred_im_y/pparams.imscalef),1,1,255,frame_number);
    _image_item.valid = valid;

    track_data data;
    data.pos_valid = valid;
    data.state.pos.x = log.ins_pos_x;
    data.state.pos.y = log.ins_pos_y;
    data.state.pos.z = log.ins_pos_z;
    data.posX_smooth = log.ins_spos_x;
    data.posY_smooth = log.ins_spos_y;
    data.posZ_smooth = log.ins_spos_z;
    data.state.vel.x = log.ins_svel_x;
    data.state.vel.y = log.ins_svel_y;
    data.state.vel.z = log.ins_svel_z;
    data.state.acc.x = log.ins_sacc_x;
    data.state.acc.y = log.ins_sacc_y;
    data.state.acc.z = log.ins_sacc_z;
    data.time = time;
    track_history.push_back(data);

    cv::Point3f recalc_world = im2world(cv::Point2f(log.ins_im_x,log.ins_im_y), _image_item.disparity,_visdat->Qf,_visdat->camera_angle);
    if (norm(recalc_world -data.pos()) > 0.01) {
        //it seems the camera angle was changed since this log, or someone has hacked something into this log. Use the world coordinates to match the image coordinates
        //(UN)HACK:
        cv::Point3f diff = recalc_world -data.pos();
        cv::Point3f recalc_world_pred =  im2world(cv::Point2f(log.ins_pred_im_x,log.ins_pred_im_y), _image_item.disparity,_visdat->Qf,_visdat->camera_angle);
        recalc_world_pred -= diff;
        cv::Point3f recalc_im_coor = world2im_3d(data.pos(),_visdat->Qfi,_visdat->camera_angle);
        cv::Point3f recalc_im_pred_coor = world2im_3d(recalc_world_pred,_visdat->Qfi,_visdat->camera_angle);

        _image_item = ImageItem (recalc_im_coor.x/pparams.imscalef,recalc_im_coor.y/pparams.imscalef,recalc_im_coor.z,frame_number);
        _image_item.valid = valid;
        _image_predict_item = ImagePredictItem(cv::Point2f(recalc_im_pred_coor.x/pparams.imscalef,recalc_im_pred_coor.y/pparams.imscalef),1,1,255,frame_number);
    }

    _image_predict_item.valid = _image_predict_item.x > 0 ;
    predicted_image_path.push_back(_image_predict_item);

    WorldItem w;
    w.iti = _image_item;
    w.valid = valid;
    w.pt.x = log.ins_pos_x;
    w.pt.y = log.ins_pos_y;
    w.pt.z = log.ins_pos_z;
    w.distance = norm(w.pt);
    w.iti.x = std::clamp(static_cast<int>(w.iti.x),0,_visdat->depth_background_mm.cols);
    w.iti.y = std::clamp(static_cast<int>(w.iti.y),0,_visdat->depth_background_mm.rows);
    w.distance_bkg = _visdat->depth_background_mm.at<float>(w.iti.y,w.iti.x);
    path.push_back(w);
    _world_item = w;

    n_frames_lost = log.ins_n_frames_lost;
    n_frames_tracking = log.ins_n_frames_tracking;
    _tracking = log.ins_foundL;

    if (!_tracking) {
        predicted_image_path.clear();
        _image_predict_item.valid = false;
    }
    cleanup_paths();
}

void ReplayTracker::track(double time) {
    start_new_log_line(time,_visdat->frame_id);
    update_from_log(_visdat->frame_id,time);
    ItemTracker::append_log();
    (*_logger) << '\n';
}

}
