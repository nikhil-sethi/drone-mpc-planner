#include "visiondata.h"
#include <opencv2/highgui/highgui.hpp>

#ifdef HASSCREEN
//#define TUNING
#endif

using namespace cv;
using namespace std;


void VisionData::init(cv::Mat new_Qf, cv::Mat new_frameL,cv::Mat new_frameR){
    Qf = new_Qf;
    frameL = new_frameL;
    frameR = new_frameR;
    frameL_prev = frameL;
    frameR_prev = frameR;

    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }

    smallsize =cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    frameL.convertTo(frameL16, CV_16SC1);
    frameL_prev16_OK = frameL16.clone();
    diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
    diffL16_prevOK = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);

    init_avg_prev_frame();

#ifdef TUNING
    namedWindow("Background", WINDOW_NORMAL);
    createTrackbar("Uncertain mult", "Background", &settings.uncertainty_multiplier, 255);
    createTrackbar("Uncertain pow", "Background", &settings.uncertainty_power, 255);
    createTrackbar("Uncertain back", "Background", &settings.uncertainty_background, 255);
#endif

}

void VisionData::update(cv::Mat new_frameL,cv::Mat new_frameR,float time, int new_frame_id) {
    frameL_prev = frameL.clone();
    frameR_prev = frameR.clone();
    frameL = new_frameL;
    frameR = new_frameR;
    frame_id = new_frame_id;

    frameL_prev16 = frameL16.clone();
    frameL.convertTo(frameL16, CV_16SC1);

    //calcuate the motion difference, through the integral over time (over each pixel)
    cv::Mat d = frameL16 - frameL_prev16;
    diffL16 += d;
    diffL16 = (diffL16 * 595) / 600;
    diffL16.convertTo(diffL, CV_8UC1);
    cv::resize(diffL,diffL_small,smallsize);

    if (!background_calibrated )
        collect_no_drone_frames(diffL_small); // calibration of background uncertainty map

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

/*calcuate motion differences, same code as above except performed on the last frame that was good*/
void VisionData::update_prevOK() {
    cv::Mat d = frameL16 - frameL_prev16_OK;
    diffL16_prevOK += d;
    diffL16_prevOK.convertTo(diffL_prevOK, CV_8UC1);
    cv::resize(diffL_prevOK,diffL_prevOK_small,smallsize);
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
    avg_prev_frame = cv::Mat::zeros(frameL.rows,frameL.cols,CV_32SC1);
    n_avg_prev_frames = 0;
}

void VisionData::collect_avg_prev_frame(cv::Mat frame) {
    cv::Mat frame32;
    frame.convertTo(frame32,CV_32SC1);
    n_avg_prev_frames+=1;
    avg_prev_frame +=frame32;
}
