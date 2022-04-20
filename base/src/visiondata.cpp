#include "visiondata.h"

using namespace cv;
using namespace std;

void VisionData::init(Cam *cam) {
    _cam = cam;
    Qf = cam->Qf;
    cv::invert(Qf, Qfi);
    auto frame = cam->current();
    frameL = frame->left;
    frameR = frame->right;
    depth_background_mm = cam->depth_background_mm;

    enable_viz_motion = false; // Note: the enable_diff_vizs in the insect/drone trackers may be more interesting instead of this one.

    camera_pitch = _cam->camera_pitch();
    camera_roll = _cam->camera_roll();
    camera_gain = _cam->measured_gain();
    camera_exposure = _cam->measured_exposure();

    deserialize_settings();

    smallsize = cv::Size(frameL.cols / pparams.imscalef, frameL.rows / pparams.imscalef);
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);
    diffL16 = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_16SC1);
    diffR16 = cv::Mat::zeros(cv::Size(frameR.cols, frameR.rows), CV_16SC1);
    overexposed_bufferL16 = frameL16.clone(); //cv::Mat::zeros(cv::Size(frameL.cols,frameL.rows),CV_16SC1);;
    overexposed_bufferR16 = frameR16.clone(); // cv::Mat::zeros(cv::Size(frameR.cols,frameR.rows),CV_16SC1);;
    overexposed_mapL = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_8UC1);
    overexposed_mapR = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_8UC1);
    motion_max_noise_mapL = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_8UC1);
    motion_filtered_noise_mapL = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_8UC1);
    motion_filtered_noise_mapR = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_8UC1);
    motion_filtered_noise_mapL_small = cv::Mat::zeros(smallsize, CV_8UC1);

    int dilation_size = 5;
    cv::Mat element_mat = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
    element_mat.copyTo(dilate_element);

    initialized = true;
}

void VisionData::update(StereoPair *data) {
    track_avg_brightness(data->left, frameL, _current_frame_time);
    frameL = data->left;
    frameR = data->right;
    frame_id = data->rs_id;
    _current_frame_time = data->time;

    cv::Mat frameL_prev16 = frameL16;
    cv::Mat frameR_prev16 = frameR16;
    frameL16 = cv::Mat();
    frameR16 = cv::Mat();
    frameL.convertTo(frameL16, CV_16SC1);
    frameR.convertTo(frameR16, CV_16SC1);


    if (brightness_change_event(_current_frame_time)) {
        diffL16 = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_16SC1);
        diffR16 = cv::Mat::zeros(cv::Size(frameR.cols, frameR.rows), CV_16SC1);
    }
    static bool reset_motion_integration_prev = false; // used to zero diffL16 and R only once
    if (_reset_motion_integration) {
        frameL_prev16 = frameL16;
        frameR_prev16 = frameR16;
        if (!reset_motion_integration_prev) {
            diffL16 = cv::Mat::zeros(cv::Size(frameL.cols, frameL.rows), CV_16SC1);
            diffR16 = cv::Mat::zeros(cv::Size(frameR.cols, frameR.rows), CV_16SC1);
        }
        reset_motion_integration_prev = true;
        _reset_motion_integration = false;
        motion_update_iterator = 0;
    } else {
        reset_motion_integration_prev = false;
        if (motion_spot_to_be_reset.cnt_active) {
            cv::circle(diffL16, motion_spot_to_be_reset.pt, motion_spot_to_be_reset.r, 0, cv::FILLED);
            cv::circle(diffR16, motion_spot_to_be_reset.pt + cv::Point(motion_spot_to_be_reset.disparity, 0), motion_spot_to_be_reset.r, 0, cv::FILLED);
            motion_spot_to_be_reset.cnt_active--;
        }

        //calcuate the motion difference, through the integral over time (over each pixel)
        cv::Mat dL = frameL16 - frameL_prev16;
        diffL16 += dL;
        cv::Mat dR = frameR16 - frameR_prev16;
        diffR16 += dR;

        if (!(motion_update_iterator++ % (motion_update_iterator_max + 1)) && !disable_fading) { // +1 -> prevent 0
            fade(diffL16, exclude_drone_from_motion_fading_spot_L);
            fade(diffR16, exclude_drone_from_motion_fading_spot_R);
        }

        if (motion_spot_to_be_deleted.cnt_active) {
            cv::circle(diffL16, motion_spot_to_be_deleted.pt, motion_spot_to_be_deleted.r, 0, cv::FILLED);
            cv::circle(diffR16, motion_spot_to_be_deleted.pt + cv::Point(motion_spot_to_be_deleted.disparity, 0), motion_spot_to_be_deleted.r, 0, cv::FILLED);
            motion_spot_to_be_deleted.cnt_active--;
        }
    }

    cv::Mat diffL16_abs = abs(diffL16);
    diffL16_abs.convertTo(diffL, CV_8UC1);
    cv::resize(diffL, diffL_small, smallsize);
    cv::Mat diffR16_abs = abs(diffR16);
    diffR16_abs.convertTo(diffR, CV_8UC1);
    cv::resize(diffR, diffR_small, smallsize);

    if (enable_viz_motion)
        viz_frame = diffL * 10;

}

void VisionData::fade(cv::Mat diff16, cv::Point exclude_drone_spot) {
    cv::Mat drone_roi;
    cv::Rect rec ;
    if (exclude_drone_spot.x > 0) { // this serves as an initialisation flag, as well as an disparity out of image check!
        cv::Point r(exclude_drone_from_motion_fading_radius, exclude_drone_from_motion_fading_radius);
        cv::Point p = exclude_drone_spot;

        rec = cv::Rect(p - r, p + r);
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

    cv::Mat diff16_neg, diff16_pos;
    diff16_pos = min(diff16 >= 1, 1);
    diff16_neg = min(diff16 <= -1, 1);
    diff16_pos.convertTo(diff16_pos, CV_16SC1);
    diff16_neg.convertTo(diff16_neg, CV_16SC1);
    diff16 -= diff16_pos;
    diff16 += diff16_neg;

    if (exclude_drone_spot.x > 0 && rec.width && rec.height) {
        drone_roi.copyTo(diff16(rec));
    }
}

void VisionData::maintain_noise_maps() {
    if (frame_id % pparams.fps == 0) {
        motion_noise_bufferL.push_back(diffL.clone());
        motion_noise_bufferR.push_back(diffR.clone());
        cv::Mat mask = diffL > motion_max_noise_mapL;
        diffL.copyTo(motion_max_noise_mapL, mask);
        if (motion_noise_bufferL.size() > motion_buf_size_target) {
            _calibrating_motion_noise_map = true;
            save_every_nth_frame_during_motion_calib = 1;
        }

        overexposed_bufferL16 = (overexposed_bufferL16 * 7 + frameL16) / 8;
        overexposed_bufferR16 = (overexposed_bufferR16 * 7 + frameR16) / 8;
        overexposed_mapL = overexposed_bufferL16 > 200;
        overexposed_mapR = overexposed_bufferL16 > 200;
    }

    if (_calibrating_motion_noise_map) {
        if (!(frame_id % save_every_nth_frame_during_motion_calib)) {
            motion_noise_bufferL.push_back(diffL.clone());
            motion_noise_bufferR.push_back(diffR.clone());
        }
        if (_current_frame_time > calibrating_noise_map_end_time && motion_noise_bufferL.size() > motion_buf_size_target) {
            std::cout << "Rebuilding motion noise map" << std::endl;
            _calibrating_motion_noise_map = false;

            cv::Mat bkgL = motion_noise_bufferL.at(0).clone();
            cv::Mat bkgR = motion_noise_bufferR.at(0).clone();
            for (cv::Mat im : motion_noise_bufferL) {
                cv::Mat mask = im > bkgL;
                im.copyTo(bkgL, mask);
            }
            for (cv::Mat im : motion_noise_bufferR) {
                cv::Mat mask = im > bkgR;
                im.copyTo(bkgR, mask);
            }

            motion_max_noise_mapL = bkgL.clone();
            cv::dilate(bkgL, bkgL, dilate_element);
            cv::dilate(bkgR, bkgR, dilate_element);
            GaussianBlur(bkgL, motion_filtered_noise_mapL, Size(9, 9), 0);
            GaussianBlur(bkgR, motion_filtered_noise_mapR, Size(9, 9), 0);
            cv::resize(bkgL, motion_filtered_noise_mapL_small, smallsize);
            _motion_filtered_noise_initialized = true;

            motion_noise_bufferL.clear();
            motion_noise_bufferR.clear();

        }
    }
}

int VisionData::max_noise(cv::Point blob_pt) {
    //get the max noise in a small area around the blob point, meant to be used to detect new insects
    const int size = 10;
    cv::Point2i p(std::clamp(static_cast<int>(roundf(blob_pt.x * pparams.imscalef)) - size / 2, 0, IMG_W - size), std::clamp(static_cast<int>(roundf(blob_pt.y * pparams.imscalef)) - size / 2, 0, IMG_H - size));
    cv::Rect roi(p, cv::Size(size, size));
    double max;
    cv::minMaxIdx(motion_max_noise_mapL(roi), NULL, &max, NULL, NULL);
    return static_cast<int>(round(max));
}

bool VisionData::overexposed(cv::Point blob_pt) {
    //check in a small area around the blob point if there are any overexposed pixels (which are 0 in the overexposed map)
    const int size = 20;
    cv::Point2i p(std::clamp(static_cast<int>(roundf(blob_pt.x * pparams.imscalef)) - size / 2, 0, IMG_W - size), std::clamp(static_cast<int>(roundf(blob_pt.y * pparams.imscalef)) - size / 2, 0, IMG_H - size));
    cv::Rect roi(p, cv::Size(size, size));
    int s = cv::sum(overexposed_mapL(roi))[0];
    return s > 0;
}

void VisionData::enable_noise_map_calibration() {
    enable_noise_map_calibration(motion_buf_size_target / pparams.fps);
}
void VisionData::enable_noise_map_calibration(float duration) {
    calibrating_noise_map_end_time = _current_frame_time + static_cast<double>(duration);
    save_every_nth_frame_during_motion_calib = static_cast<int>(round((pparams.fps * duration) / motion_buf_size_target));
    if (save_every_nth_frame_during_motion_calib < 1)
        save_every_nth_frame_during_motion_calib = 1;
    _calibrating_motion_noise_map = true;
}

//Keep track of the average brightness, and reset the motion integration frame when it changes to much. (e.g. when someone turns on the lights or something)
void VisionData::track_avg_brightness(cv::Mat frameL_new, cv::Mat frameL_prev, double time) {
    cv::Mat frame_top = frameL_new(cv::Rect(frameL_new.cols / 3, 0, frameL_new.cols / 3 * 2, frameL_new.rows / 3)); // only check the middle & top third of the image, to save CPU
    float brightness_new = static_cast<float>(mean(frame_top)[0]);
    float brightness_diff = fabs(brightness_new - brightness_prev);
    if (time > 1.5) {
        if (brightness_diff > brightness_event_tresh) {
            std::cout << "Warning, large brightness change: " << brightness_prev << " -> " << brightness_new  << std::endl;
            large_brightness_event_time = time;
        } else if (brightness_diff > 0.3f)  { // capture subtle autoexposure changes
            float brightness_ff_new = static_cast<float>(mean(frameL_new)[0]);
            float brightness_ff_prev = static_cast<float>(mean(frameL_prev)[0]);
            float brightness_ff_diff = fabs(brightness_ff_new - brightness_ff_prev);

            if (brightness_ff_diff > 0.1f) {
                small_brightness_event_time = time;
                std::cout << "Warning, small brightness change: " << brightness_prev << " -> " << brightness_new << "=" <<  brightness_diff << ". And ff brightness " << brightness_ff_prev << " -> " << brightness_ff_new << " = " << brightness_ff_diff << std::endl;
            }
        }
    }
    brightness_prev = brightness_new;
}


void VisionData::delete_from_motion_map(cv::Point p, int disparity, int radius, int duration) {
    motion_spot_to_be_deleted.cnt_active = duration;
    motion_spot_to_be_deleted.pt = p;
    motion_spot_to_be_deleted.disparity = disparity;
    motion_spot_to_be_deleted.r = radius;
}

void VisionData::reset_spot_on_motion_map(cv::Point p, int disparity, int radius, int duration) {
    motion_spot_to_be_reset.cnt_active = duration;
    motion_spot_to_be_reset.pt = p;
    motion_spot_to_be_reset.disparity = disparity;
    motion_spot_to_be_reset.r = radius;
}

void VisionData::exclude_drone_from_motion_fading(cv::Point3f p, int radius) {
    exclude_drone_from_motion_fading_spot_L = cv::Point(p.x, p.y);
    exclude_drone_from_motion_fading_spot_R = cv::Point(p.x - p.z, p.y);
    exclude_drone_from_motion_fading_radius = radius;
}


void VisionData::init_insect_replay(std::string folder) {
    overexposed_mapL = cv::imread(folder + "/overexposed_mapL.png", cv::IMREAD_ANYDEPTH);
    overexposed_mapR = cv::imread(folder + "/overexposed_mapR.png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapL = cv::imread(folder + "/motion_filtered_noise_mapL.png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapR = cv::imread(folder + "/motion_filtered_noise_mapR.png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapL_small = cv::imread(folder + "/motion_filtered_noise_mapL_small.png", cv::IMREAD_ANYDEPTH);
    motion_max_noise_mapL = cv::imread(folder + "/motion_max_noise_mapL.png", cv::IMREAD_ANYDEPTH);
}
void VisionData::init_flight_replay(std::string folder, int id) {
    overexposed_mapL = cv::imread(folder + "/overexposed_mapL_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
    overexposed_mapR = cv::imread(folder + "/overexposed_mapR_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapL = cv::imread(folder + "/motion_filtered_noise_mapL_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapR = cv::imread(folder + "/motion_filtered_noise_mapR_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
    motion_filtered_noise_mapL_small = cv::imread(folder + "/motion_filtered_noise_mapL_small_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
    motion_max_noise_mapL = cv::imread(folder + "/motion_max_noise_mapL_" + to_string(id) + ".png", cv::IMREAD_ANYDEPTH);
}

void VisionData::save_maps_before_monitoring(std::string folder) {
    cv::imwrite(folder + "/overexposed_mapL.png", overexposed_mapL);
    cv::imwrite(folder + "/overexposed_mapR.png", overexposed_mapR);
    cv::imwrite(folder + "/motion_filtered_noise_mapL.png", motion_filtered_noise_mapL);
    cv::imwrite(folder + "/motion_filtered_noise_mapR.png", motion_filtered_noise_mapR);
    cv::imwrite(folder + "/motion_filtered_noise_mapL_small.png", motion_filtered_noise_mapL_small);
    cv::imwrite(folder + "/motion_max_noise_mapL.png", motion_max_noise_mapL);
}
void VisionData::save_maps_before_flight(int id, std::string folder) {
    cv::imwrite(folder + "/overexposed_mapL_" + to_string(id) + ".png", overexposed_mapL);
    cv::imwrite(folder + "/overexposed_mapR_" + to_string(id) + ".png", overexposed_mapR);
    cv::imwrite(folder + "/motion_filtered_noise_mapL_" + to_string(id) + ".png", motion_filtered_noise_mapL);
    cv::imwrite(folder + "/motion_filtered_noise_mapR_" + to_string(id) + ".png", motion_filtered_noise_mapR);
    cv::imwrite(folder + "/motion_filtered_noise_mapL_small_" + to_string(id) + ".png", motion_filtered_noise_mapL_small);
    cv::imwrite(folder + "/motion_max_noise_mapL_" + to_string(id) + ".png", motion_max_noise_mapL);
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
            throw std::runtime_error("Cannot read: " + settings_file);
        }
        VisionParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw std::runtime_error("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw std::runtime_error("File not found: " + settings_file);
    }

    motion_update_iterator_max = params.motion_update_iterator_max.value();
    brightness_event_tresh = params.brightness_event_tresh.value();
}

void VisionData::serialize_settings() {
    VisionParameters params;
    params.motion_update_iterator_max = motion_update_iterator_max;
    params.brightness_event_tresh = brightness_event_tresh;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream(settings_file);
    outfile << xmlData ;
    outfile.close();
}

void VisionData::close() {
    if (initialized) {
        std::cout << "Closing visdat" << std::endl;
        initialized = false;
    }
}
