#include "visiondata.h"

using namespace cv;
using namespace std;

void VisionData::init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, float new_camera_gain, float new_camera_exposure, cv::Mat new_depth_background_mm) {
    Qf = new_Qf;
    cv::invert(Qf,Qfi);
    frameL = new_frameL.clone();
    frameR = new_frameR.clone();
    depth_background_mm = new_depth_background_mm;
    frameL_background = new_frameL.clone();

    enable_viz_diff = false; // Note: the enable_diff_vizs in the insect/drone trackers may be more interesting instead of this one.

    camera_angle = new_camera_angle;
    camera_gain = new_camera_gain;
    camera_exposure = new_camera_exposure;

    motion_noise_mapL_wfn = data_output_dir + motion_noise_mapL_wfn;
    motion_noise_mapR_wfn = data_output_dir + motion_noise_mapR_wfn;
    overexposed_map_wfn = data_output_dir + overexposed_map_wfn;

    deserialize_settings();

    smallsize =cv::Size(frameL.cols/pparams.imscalef,frameL.rows/pparams.imscalef);
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);
    diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
    diffR16 = cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);

    int dilation_size = 5;
    cv::Mat element_mat = getStructuringElement( cv::MORPH_RECT,cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),cv::Point( dilation_size, dilation_size ) );
    element_mat.copyTo(dilate_element);

    if (pparams.vision_tuning) {
        namedWindow("Background", WINDOW_NORMAL);
        createTrackbar("motion_update_iterator_max", "Background", &motion_update_iterator_max, 255);
    }

    initialized = true;
}

void VisionData::read_motion_and_overexposed_from_image(std::string replay_dir) {
    std::string motion_noise_mapL_rfn = replay_dir + '/' + motion_noise_mapL_wfn;
    std::string motion_noise_mapR_rfn = replay_dir + '/' + motion_noise_mapR_wfn;
    std::string overexposed_map_rfn = replay_dir + '/' + overexposed_map_wfn;

    if (!file_exist(motion_noise_mapR_rfn)) { //for older logs that don't have seperate max_motion_noise.pngs yet. (probably can be removed in the near future)
        motion_noise_mapL_rfn = replay_dir + "/max_motion_noise.png";
        motion_noise_mapR_rfn = replay_dir + "/max_motion_noise.png";
    }

    motion_noise_mapL = cv::imread(motion_noise_mapL_rfn,cv::IMREAD_ANYDEPTH);
    motion_noise_mapR = cv::imread(motion_noise_mapR_rfn,cv::IMREAD_ANYDEPTH);
    overexposed_map = cv::imread(overexposed_map_rfn,cv::IMREAD_ANYDEPTH);
    smallsize =cv::Size(motion_noise_mapL.cols/pparams.imscalef,motion_noise_mapL.rows/pparams.imscalef);
    cv::resize(motion_noise_mapL,motion_noise_mapL_small,smallsize);
    cv::resize(motion_noise_mapR,motion_noise_mapR_small,smallsize);

    _reset_motion_integration = true;
    enable_collect_no_drone_frames = false;
    _calibrating_background = false;
}

void VisionData::update(cv::Mat new_frameL,cv::Mat new_frameR,double time, unsigned long long new_frame_id) {
    frameL = new_frameL;
    frameR = new_frameR;
    frame_id = new_frame_id;
    _current_frame_time = time;

    cv::Mat frameL_prev16 = frameL16.clone();
    cv::Mat frameR_prev16 = frameR16.clone();
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);

    track_avg_brightness(frameL,time);
    static bool reset_motion_integration_prev = false; // used to zero diffL16 and R only once
    if (_reset_motion_integration) {
        // std::cout << "Resetting motion" << std::endl;
        frameL_prev16 = frameL16.clone();
        frameR_prev16 = frameR16.clone();
        if (!reset_motion_integration_prev) {
            diffL16 = cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);
            diffR16 = cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);
        }
        reset_motion_integration_prev = true;
        _reset_motion_integration = false;
        motion_update_iterator = 0;
    } else {
        reset_motion_integration_prev = false;
        if (motion_spot_to_be_reset.cnt_active) {
            cv::circle(diffL16,motion_spot_to_be_reset.pt,motion_spot_to_be_reset.r,0,cv::FILLED);
            cv::circle(diffR16,motion_spot_to_be_reset.pt+ cv::Point(motion_spot_to_be_reset.disparity,0),motion_spot_to_be_reset.r,0,cv::FILLED);
            motion_spot_to_be_reset.cnt_active--;
        }

        //calcuate the motion difference, through the integral over time (over each pixel)
        cv::Mat dL = frameL16 - frameL_prev16;
        diffL16 += dL;
        cv::Mat dR = frameR16 - frameR_prev16;
        diffR16 += dR;

        if (!(motion_update_iterator++ % (motion_update_iterator_max+1)) && !disable_fading) { // +1 -> prevent 0
            fade(diffL16,exclude_drone_from_motion_fading_spot_L);
            fade(diffR16,exclude_drone_from_motion_fading_spot_R);
        }

        if (motion_spot_to_be_deleted.cnt_active) {
            cv::circle(diffL16,motion_spot_to_be_deleted.pt,motion_spot_to_be_deleted.r,0,cv::FILLED);
            cv::circle(diffR16,motion_spot_to_be_deleted.pt + cv::Point(motion_spot_to_be_deleted.disparity,0),motion_spot_to_be_deleted.r,0,cv::FILLED);
            motion_spot_to_be_deleted.cnt_active--;
        }
    }

    cv::Mat diffL16_abs = abs(diffL16);
    diffL16_abs.convertTo(diffL, CV_8UC1);
    cv::resize(diffL,diffL_small,smallsize);

    cv::Mat diffR16_abs = abs(diffR16);
    diffR16_abs.convertTo(diffR, CV_8UC1);
    cv::resize(diffR,diffR_small,smallsize);

    if (enable_viz_diff)
        viz_frame = diffL*10;

    maintain_motion_noise_map();
}

void VisionData::fade(cv::Mat diff16, cv::Point exclude_drone_spot) {
    cv::Mat drone_roi;
    cv::Rect rec ;
    if (exclude_drone_spot.x>0) { // this serves as an initialisation flag, as well as an disparity out of image check!
        cv::Point r(exclude_drone_from_motion_fading_radius,exclude_drone_from_motion_fading_radius);
        cv::Point p = exclude_drone_spot;

        rec = cv::Rect(p-r,p+r);
        if (rec.x < 0)
            rec.x = 0;
        if (rec.y < 0)
            rec.y = 0;
        if (rec.width + rec.x >= diff16.cols)
            rec.width = diff16.cols - rec.x - 1;
        if (rec.height + rec.y >= diff16.rows)
            rec.height = diff16.rows - rec.y - 1;

        if (rec.width && rec.height) // check if it is not out of the image
            drone_roi = diff16(rec).clone() ;
    }

    cv::Mat diff16_neg,diff16_pos;
    diff16_pos = min(diff16 >= 1,1);
    diff16_neg = min(diff16 <= -1,1);
    diff16_pos.convertTo(diff16_pos,CV_16SC1);
    diff16_neg.convertTo(diff16_neg,CV_16SC1);
    diff16 -= diff16_pos;
    diff16 += diff16_neg;

    if (exclude_drone_spot.x>0 && rec.width && rec.height) {
        drone_roi.copyTo(diff16(rec));
    }
}

void VisionData::maintain_motion_noise_map() {
    if (frame_id % pparams.fps == 0 && enable_collect_no_drone_frames) {
        static uint cnt = 0;
        motion_noise_bufferL.push_back(diffL);
        motion_noise_bufferL.erase(motion_noise_bufferL.begin());
        motion_noise_bufferR.push_back(diffR);
        motion_noise_bufferR.erase(motion_noise_bufferR.begin());
        cnt++;
        if (cnt > motion_noise_bufferL.size() && motion_noise_bufferL.size() > 1) {
            _calibrating_background = true;
            frameL_background = frameL/2 + frameL_background/2;
            cnt = 0;
        }
    }

    if (enable_collect_no_drone_frames && _calibrating_background ) {
        motion_noise_bufferL.push_back(diffL);
        motion_noise_bufferR.push_back(diffR);
        if (_current_frame_time > calibrating_background_end_time) {
            std::cout << "Writing motion noise map" << std::endl;
            _calibrating_background = false;

            cv::Mat bkgL = motion_noise_bufferL.at(0);
            cv::Mat bkgR = motion_noise_bufferR.at(0);
            for (cv::Mat im : motion_noise_bufferL) {
                cv::Mat mask = im > bkgL;
                im.copyTo(bkgL,mask);
            }
            for (cv::Mat im : motion_noise_bufferR) {
                cv::Mat mask = im > bkgR;
                im.copyTo(bkgR,mask);
            }

            cv::dilate(bkgL,bkgL,dilate_element);
            cv::dilate(bkgR,bkgR,dilate_element);
            GaussianBlur(bkgL,motion_noise_mapL,Size(9,9),0);
            GaussianBlur(bkgR,motion_noise_mapR,Size(9,9),0);
            cv::resize(bkgL,motion_noise_mapL_small,smallsize);
            cv::resize(bkgR,motion_noise_mapR_small,smallsize);
            imwrite(motion_noise_mapL_wfn,motion_noise_mapL);
            imwrite(motion_noise_mapR_wfn,motion_noise_mapR);
        }
    }
}

void VisionData::create_overexposed_removal_mask(cv::Point2f drone_im_location, float drone_im_size) {
    cv::Mat maskL = frameL < 254;
    cv::erode(maskL,overexposed_map,dilate_element);
    cv::circle(overexposed_map,drone_im_location,drone_im_size,255, cv::FILLED);
    imwrite(overexposed_map_wfn,overexposed_map);
    _reset_motion_integration = true;
}

void VisionData::enable_background_motion_map_calibration(float duration) {
    calibrating_background_end_time = _current_frame_time+static_cast<double>(duration);
    _calibrating_background = true;
}

//Keep track of the average brightness, and reset the motion integration frame when it changes to much. (e.g. when someone turns on the lights or something)
void VisionData::track_avg_brightness(cv::Mat frame,double time) {
    cv::Mat frame_top = frame(cv::Rect(frame.cols/3,0,frame.cols/3*2,frame.rows/3)); // only check the middle & top third of the image, to save CPU
    float brightness = static_cast<float>(mean( frame_top )[0]);
    if (fabs(brightness - prev_brightness) > brightness_event_tresh  && prev_brightness >= 0) {
        frameL_background = frame;
        std::cout << "Warning, large brightness change: " << prev_brightness << " -> " << brightness  << std::endl;
        _reset_motion_integration = true;
        _large_brightness_change_event_time = time;
    }
    prev_brightness = brightness;

}

void VisionData::delete_from_motion_map(cv::Point p, int disparity,int radius, int duration) {
    motion_spot_to_be_deleted.cnt_active = duration;
    motion_spot_to_be_deleted.pt = p;
    motion_spot_to_be_deleted.disparity = disparity;
    motion_spot_to_be_deleted.r = radius;
}

void VisionData::reset_spot_on_motion_map(cv::Point p, int disparity,int radius, int duration) {
    motion_spot_to_be_reset.cnt_active = duration;
    motion_spot_to_be_reset.pt = p;
    motion_spot_to_be_reset.disparity = disparity;
    motion_spot_to_be_reset.r = radius;
}

void VisionData::exclude_drone_from_motion_fading(cv::Point3f p, int radius) {
    exclude_drone_from_motion_fading_spot_L = cv::Point(p.x,p.y);
    exclude_drone_from_motion_fading_spot_R = cv::Point(p.x-p.z,p.y);
    exclude_drone_from_motion_fading_radius = radius;
}

void VisionData::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    VisionParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        VisionParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw my_exit("File not found: " + settings_file);
    }

    motion_update_iterator_max = params.motion_update_iterator_max.value();
    brightness_event_tresh = params.brightness_event_tresh.value();
}

void VisionData::serialize_settings() {
    VisionParameters params;
    params.motion_update_iterator_max = motion_update_iterator_max;
    params.brightness_event_tresh = brightness_event_tresh;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void VisionData::close() {
    if (initialized) {
        std::cout << "Closing visdat" << std::endl;
        if (pparams.vision_tuning)
            serialize_settings();
        initialized = false;
    }
}
