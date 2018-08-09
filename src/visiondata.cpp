#include "visiondata.h"
#include <opencv2/highgui/highgui.hpp>

#ifdef HASSCREEN
//#define TUNING
#endif

using namespace cv;
using namespace std;


void VisionData::init(cv::Mat Qf, cv::Mat frameL,cv::Mat frameR){
    _Qf = Qf;
    _frameL = frameL;
    _frameR = frameR;
    _frameL_prev = frameL;
    _frameR_prev = frameR;

    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }

    smallsize =cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    cv::resize(frameL,_frameL_small,smallsize);
    _frameL_s_prev = _frameL_small.clone();

    init_avg_prev_frame();

#ifdef TUNING
    namedWindow("Background", WINDOW_NORMAL);
    createTrackbar("Uncertain mult", "Background", &settings.uncertainty_multiplier, 255);
    createTrackbar("Uncertain pow", "Background", &settings.uncertainty_power, 255);
    createTrackbar("Uncertain back", "Background", &settings.uncertainty_background, 255);
#endif

}


void VisionData::update(cv::Mat frameL,cv::Mat frameR,float time, int frame_id) {
    _frameL_prev = this->_frameL.clone();
    _frameR_prev = this->_frameR.clone();
    _frameL = frameL;
    _frameR = frameR;
    _frame_id = frame_id;

    _frameL_s_prev = _frameL_small.clone();
    cv::resize(frameL,_frameL_small,smallsize);


    cv::absdiff( _frameL_small ,_frameL_s_prev,diffL);
    if (!background_calibrated )
        collect_no_drone_frames(diffL); // calibration of background uncertainty map

#ifdef BEEP
    if (!background_calibrated && time > settings.background_calib_time) {
        system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Mallet.ogg &");
    }
#endif
    if (!background_calibrated && time > settings.background_calib_time)
        background_calibrated= true;

#if CAMMODE == CAMMODE_GENERATOR
    background_calibrated = true;
#endif

}

void VisionData::collect_no_drone_frames(cv::Mat diff) {
    static cv::Mat max_uncertainty_map = diff.clone();
    cv::Mat mask = diff > max_uncertainty_map;
    diff.copyTo(max_uncertainty_map,mask);

    uncertainty_map = 255 - max_uncertainty_map * settings.uncertainty_multiplier;
    uncertainty_map.convertTo(uncertainty_map ,CV_32F);
    uncertainty_map /=255.0f;
    cv::pow(uncertainty_map,settings.uncertainty_power,uncertainty_map);

}

void VisionData::init_avg_prev_frame(void) {
    avg_prev_frame = cv::Mat::zeros(_frameL_s_prev.rows,_frameL_s_prev.cols,CV_32SC1);
    n_avg_prev_frames = 0;
}

void VisionData::collect_avg_prev_frame(cv::Mat frame) {
    cv::Mat frame32;
    frame.convertTo(frame32,CV_32SC1);
    n_avg_prev_frames+=1;
    avg_prev_frame +=frame32;
}
