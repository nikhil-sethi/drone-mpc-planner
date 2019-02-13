#include "visiondata.h"

#ifdef HASSCREEN
//#define TUNING
#endif

using namespace cv;
using namespace std;

void VisionData::init(bool fromfile, std::string log_in_dir,cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, cv::Mat new_depth_background_mm){
    Qf = new_Qf;
    frameL = new_frameL.clone();
    frameR = new_frameR.clone();
    frameL_prev = frameL;
    frameR_prev = frameR;
    depth_background_mm = new_depth_background_mm;

    camera_angle = new_camera_angle;

    if (fromfile)
        motion_noise_map_fn = log_in_dir + "/" +  motion_noise_map_fn;
    else
        motion_noise_map_fn = "./logging/" + motion_noise_map_fn;
    max_uncertainty_map = imread(motion_noise_map_fn,false);

    if (getSecondsSinceFileCreation(motion_noise_map_fn) < 60*60 || fromfile) {
        _background_calibrated = true;
        //todo: remove the old uncertainty map stuff, I don't think it works any more anyway
    }

    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(settings);
        }catch (cereal::Exception e) {
            std::cout << "Visiondata settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << settingsFile << std::endl;
            exit (1);
        }
        BaseVisionSettings tmp;
        if (settings.version < tmp.version){
            std::cout << "Visiondata settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << settingsFile << std::endl;
            throw my_exit(1);
        }
    }

    smallsize =cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);
    diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
    diffR16 = cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);

#ifdef TUNING
    namedWindow("Background", WINDOW_NORMAL);
    createTrackbar("Uncertain mult", "Background", &settings.uncertainty_multiplier, 255);
    createTrackbar("Uncertain pow", "Background", &settings.uncertainty_power, 255);
    createTrackbar("Uncertain back", "Background", &settings.uncertainty_background, 255);
#endif

}

void VisionData::update(cv::Mat new_frameL,cv::Mat new_frameR,float time, int new_frame_id) {
    lock_data.lock();

    frameL_prev = frameL;
    frameR_prev = frameR;
    frameL = new_frameL;
    frameR = new_frameR;
    frame_id = new_frame_id;

    frameL_prev16 = frameL16;
    frameR_prev16 = frameR16;
    cv::Mat tmpL;
    frameL.convertTo(tmpL, CV_16SC1);
    frameL16 = tmpL;
    cv::Mat tmpR;
    frameR.convertTo(tmpR, CV_16SC1);
    frameR16 = tmpR;

    track_avg_brightness(frameL16,time); //todo: check if this still works!
    if (_reset_motion_integration) {
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

    if (!(motion_update_iterator++ % settings.motion_update_iterator_max)) {
        //split negative and positive motion
        fade(diffL16);
        fade(diffR16);
    }

    diffL = abs(diffL16);
    diffL.convertTo(diffL, CV_8UC1);
    cv::resize(diffL,diffL_small,smallsize);

    diffR = abs(diffR16);
    diffR.convertTo(diffR, CV_8UC1);
    cv::resize(diffR,diffR_small,smallsize);

//    showRowImage({diffL*10,diffR*10},"motion",CV_8UC1,1.f);

    if (!_background_calibrated )
        collect_no_drone_frames(diffL_small); // calibration of background uncertainty map

    if (!_background_calibrated && time > settings.background_calib_time) {
        _background_calibrated= true;
        imwrite(motion_noise_map_fn,max_uncertainty_map);
    }

#if CAMMODE == CAMMODE_GENERATOR
    _background_calibrated = true;
#endif

    lock_data.unlock();
}

void VisionData::fade(cv::Mat diff16) {
    cv::Mat diffL16_neg,diffL16_pos;
    diffL16_pos = min(diff16 >= 1,1);
    diffL16_neg = min(diff16 <= -1,1);
    diffL16_pos.convertTo(diffL16_pos,CV_16SC1);
    diffL16_neg.convertTo(diffL16_neg,CV_16SC1);
    diff16 -= diffL16_pos;
    diff16 += diffL16_neg;
}

void VisionData::collect_no_drone_frames(cv::Mat diff) {

    if (max_uncertainty_map.cols == 0)
        max_uncertainty_map = diff.clone();
    cv::Mat mask = diff > max_uncertainty_map;
    diff.copyTo(max_uncertainty_map,mask);
}

//Keep track of the average brightness, and reset the motion integration frame when it changes to much. (e.g. when someone turns on the lights or something)
void VisionData::track_avg_brightness(cv::Mat frame,float time) {
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
