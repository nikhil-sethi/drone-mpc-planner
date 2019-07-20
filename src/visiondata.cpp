#include "visiondata.h"

#ifdef HASSCREEN
//#define TUNING
#endif

using namespace cv;
using namespace std;

void VisionData::init(bool fromfile, cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, float new_camera_gain, cv::Mat new_depth_background_mm){
    Qf = new_Qf;
    frameL = new_frameL.clone();
    frameR = new_frameR.clone();
    frameL_prev = frameL;
    frameR_prev = frameR;
    depth_background_mm = new_depth_background_mm;

    enable_viz_diff = false; // Note: the enable_diff_vizs in the insect/drone trackers may be more interesting instead of this one.

    camera_angle = new_camera_angle;
    camera_gain = new_camera_gain;

    if (fromfile)
        motion_noise_map_wfn = "./logging/replay_" + motion_noise_map_wfn;
    else
        motion_noise_map_wfn = "./logging/" + motion_noise_map_wfn;

    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(settings);
        }catch (cereal::Exception e) {
            std::stringstream serr;
            serr << "cannot read visiondata settings file: " << e.what() << ". Maybe delete the file: " << settingsFile;
            throw my_exit(serr.str());
        }
        BaseVisionSettings tmp;
        if (tmp.version-settings.version > 0.001f){
            std::stringstream serr;
            serr << "visiondata settings version too low! Maybe delete the file: " << settingsFile;
            throw my_exit(serr.str());
        }
    }

    smallsize =cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);
    diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
    diffR16 = cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);
    motion_noise_map = cv::Mat::zeros(smallsize,CV_8UC1);
    diffL16_back = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);

#ifdef TUNING
    namedWindow("Background", WINDOW_NORMAL);
    createTrackbar("motion_update_iterator_max", "Background", &settings.motion_update_iterator_max, 255);
#endif

    initialized = true;
}

void VisionData::update(cv::Mat new_frameL,cv::Mat new_frameR,double time, unsigned long long new_frame_id) {
    lock_data.lock();
    frameL_prev = frameL;
    frameR_prev = frameR;
    frameL = new_frameL;
    frameR = new_frameR;
    frame_id = new_frame_id;
    _current_frame_time = time;

    cv::Mat frameL_prev16 = frameL16;
    cv::Mat frameR_prev16 = frameR16;
    cv::Mat tmpL;
    frameL.convertTo(tmpL, CV_16SC1);
    frameL16 = tmpL;
    cv::Mat tmpR;
    frameR.convertTo(tmpR, CV_16SC1);
    frameR16 = tmpR;

    track_avg_brightness(frameL16,time); //todo: check if this still works!
    if (_reset_motion_integration) {
        std::cout << "Resetting motion" << std::endl;
        frameL_prev16 = frameL16.clone();
        frameR_prev16 = frameR16.clone();
        diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
        diffR16 = cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);
        _reset_motion_integration = false;
        motion_update_iterator = 0;
    }

    //calcuate the motion difference, through the integral over time (over each pixel)
    cv::Mat dL = frameL16 - frameL_prev16;
    diffL16 += dL;
    cv::Mat dR = frameR16 - frameR_prev16;
    diffR16 += dR;

    if (!(motion_update_iterator++ % (settings.motion_update_iterator_max+1)) && !disable_fading) { // +1 -> prevent 0
        //split negative and positive motion
        fade(diffL16, _exclude_drone_from_motion_fading);
        _exclude_drone_from_motion_fading = false;
        fade(diffR16,false);
    }

    if (delete_motion_frame_cnt_duration>0){
        cv::circle(diffL16,delete_motion_spot,delete_motion_r,0,CV_FILLED);
        cv::circle(diffR16,delete_motion_spot,delete_motion_r,0,CV_FILLED); //todo: add disparity
        delete_motion_frame_cnt_duration--;
    }

    diffL = abs(diffL16);
    diffL.convertTo(diffL, CV_8UC1);
    cv::resize(diffL,diffL_small,smallsize);

    diffR = abs(diffR16);
    diffR.convertTo(diffR, CV_8UC1);
    cv::resize(diffR,diffR_small,smallsize);

    if (enable_viz_diff)
        viz_frame = diffL*10;

    if (_calibrating_background )
        collect_no_drone_frames(dL); // calibration of background uncertainty map

    lock_data.unlock();
}

void VisionData::fade(cv::Mat diff16, bool exclude_drone) {
    cv::Mat drone_roi;
    cv::Rect rec ;
    if (exclude_drone) {
        cv::Point r(exclude_drone_from_motion_fading_r,exclude_drone_from_motion_fading_r);
        cv::Point p = exclude_drone_from_motion_fading_spot;

        rec = cv::Rect(p-r,p+r);
        if (rec.x < 0)
            rec.x = 0;
        if (rec.y < 0)
            rec.y = 0;
        if (rec.width + rec.x >= diff16.cols)
            rec.width = diff16.cols - rec.x;
        if (rec.height + rec.y >= diff16.rows)
            rec.height = diff16.rows - rec.y;
        drone_roi = diff16(rec).clone() ;
    }

    cv::Mat diffL16_neg,diffL16_pos;
    diffL16_pos = min(diff16 >= 1,1);
    diffL16_neg = min(diff16 <= -1,1);
    diffL16_pos.convertTo(diffL16_pos,CV_16SC1);
    diffL16_neg.convertTo(diffL16_neg,CV_16SC1);
    diff16 -= diffL16_pos;
    diff16 += diffL16_neg;

    if (exclude_drone) {
        drone_roi.copyTo(diff16(rec));
    }
}

void VisionData::collect_no_drone_frames(cv::Mat dL) {

    if (skip_background_frames > 0){ // during the first frame or two, there is still residual blink
        skip_background_frames--;
        return;
    }

    diffL16_back += dL;
    cv::Mat diffL_back = abs(diffL16_back);
    diffL_back.convertTo(diffL_back, CV_8UC1);
    cv::resize(diffL_back,diffL_back,smallsize);

    cv::Mat mask = diffL_back > motion_noise_map;
    diffL_back.copyTo(motion_noise_map,mask);

    double minv,maxv;
    cv::minMaxLoc(motion_noise_map,&minv,&maxv);
    if (maxv > 100)
        std::cout << "Warning: max background motion during calibration too high: " << maxv << std::endl;

    if (_current_frame_time > calibrating_background_end_time) {
        _calibrating_background = false;
        GaussianBlur(motion_noise_map,motion_noise_map,Size(9,9),0);
        imwrite(motion_noise_map_wfn,motion_noise_map);
    }

}

void VisionData::enable_background_motion_map_calibration(double duration){
    motion_noise_map = cv::Mat::zeros(smallsize,CV_8UC1);
    diffL16_back = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
    calibrating_background_end_time = _current_frame_time+duration;
    _calibrating_background = true;
    skip_background_frames = 2;
}

//Keep track of the average brightness, and reset the motion integration frame when it changes to much. (e.g. when someone turns on the lights or something)
void VisionData::track_avg_brightness(cv::Mat frame,double time) {
    if (time - prev_time_brightness_check > settings.brightness_check_period){ // only check once in a while
        prev_time_brightness_check = time;
        cv::Mat frame_small;
        cv::resize(frame,frame_small,cv::Size(frame.cols/8,frame.rows/8));
        float brightness = mean( frame_small )[0];
        if (fabs(brightness - prev_brightness) > settings.brightness_event_tresh ) {
            std::cout << "Warning, large brightness change: " << prev_brightness << " -> " << brightness  << std::endl;
            _reset_motion_integration = true;
        }
        prev_brightness = brightness;
    }
}

void VisionData::delete_from_motion_map(cv::Point p, int radius, int duration) {
    delete_motion_spot = p;
    delete_motion_r = radius;
    delete_motion_frame_cnt_duration = duration;
}

void VisionData::exclude_drone_from_motion_fading(cv::Point p, int radius) {
    _exclude_drone_from_motion_fading = true;
    exclude_drone_from_motion_fading_spot = p;
    exclude_drone_from_motion_fading_r = radius;
}
