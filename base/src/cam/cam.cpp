#include "cam.h"
#include <librealsense2/rsutil.h>

void Cam::set_write_file_paths(std::string output_dir) {

    string idstr = std::to_string(_id);

    calib_wfn = output_dir + "/cam_calib" + idstr + ".xml";;
    depth_map_wfn = output_dir + "/depth_filtered" + idstr + ".png";
    depth_unfiltered_map_wfn = output_dir + "/depth" + idstr + ".png";
    disparity_map_wfn = output_dir + "/disparity" + idstr + ".png";
}

void Cam::set_read_file_paths(std::string read_dir) {

    string idstr = std::to_string(_id);
    calib_rfn = read_dir + "/cam_calib" + idstr + ".xml";;
    if (!file_exist(calib_rfn)) {
        calib_rfn = read_dir + "/cam_calib.xml";
        idstr = "";
    }
    depth_map_rfn = read_dir + "/depth_filtered" + idstr + ".png";
    depth_unfiltered_map_rfn = read_dir + "/depth" + idstr + ".png";
    disparity_map_rfn = read_dir + "/disparity" + idstr + ".png";
    brightness_map_rfn = read_dir + "/brightness" + idstr + ".png";
}


//Converting the raw depths of depth_background distances to
//world coordinates in the IR camera frame
//There's probably a way to do this more efficient...
void Cam::convert_depth_background_to_world() {
    depth_background_3mm = cv::Mat::zeros(depth_background.rows, depth_background.cols, CV_32FC3);
    depth_background_3mm_world = cv::Mat::zeros(depth_background.rows, depth_background.cols, CV_32FC3);
    depth_background_mm = cv::Mat::zeros(depth_background.rows, depth_background.cols, CV_32FC1);
    for (int i = 0; i < depth_background.cols; i++)
        for (int j = 0; j < depth_background.rows; j++) {
            uint16_t back = depth_background.at<uint16_t>(j, i);
            float backf = static_cast<float>(back) * camparams.depth_scale;
            float pixel[2];
            pixel[0] = i;
            pixel[1] = j;
            float p[3];
            rs2_deproject_pixel_to_point(p, intr, pixel, backf);
            cv::Vec3f pixelColor(p[0], p[1], p[2]);
            //            std::cout << "depth_background_3mm( " << i << " , " << j << " ): p[0]: " << p[0] << " ,p[1]: " << p[1] << " ,p[2]: " << p[2] <<std::endl;
            depth_background_3mm.at<cv::Vec3f>(j, i) = pixelColor;
            cv::Vec3f pixelWorldPos(p[0],
                                    p[1]*cosf(camparams.camera_angle_y * deg2rad) + p[2] * -sinf(-camparams.camera_angle_y * deg2rad),
                                    p[1]*sinf(-camparams.camera_angle_y * deg2rad) + p[2]*cosf(camparams.camera_angle_y * deg2rad));
            depth_background_3mm_world.at<cv::Vec3f>(j, i) = pixelWorldPos;
            depth_background_mm.at<float>(j, i) = sqrtf(powf(p[0], 2) + powf(p[1], 2) + powf(p[2], 2));
            if (depth_background_mm.at<float>(j, i) < 0.3f) // prevent holes in the depth map from loosing tracking
                depth_background_mm.at<float>(j, i) = INFINITY;
        }
}

ViewLimit Cam::view_limits() {
    // Some planes are defined with the slopes of the corner pixels:
    // In the past this was more precice then the difintion with background pixels.
    auto point_left_top = im2world(cv::Point2f(0, 0), 1, Qf, camparams.camera_angle_x, camparams.camera_angle_y);
    auto point_right_top = im2world(cv::Point2f(IMG_W, 0), 1, Qf, camparams.camera_angle_x, camparams.camera_angle_y);
    auto point_left_bottom = im2world(cv::Point2f(0, IMG_H), 1, Qf, camparams.camera_angle_x, camparams.camera_angle_y);
    auto point_right_bottom = im2world(cv::Point2f(IMG_W, IMG_H), 1, Qf, camparams.camera_angle_x, camparams.camera_angle_y);
    ViewLimit ret = ViewLimit(point_left_top, point_right_top, point_left_bottom, point_right_bottom);
    return ret;
}

void Cam::delete_old_frames() {
    while (buf.size() > 10) {
        auto sp =  buf.begin()->second;
        buf.erase(buf.begin()->first);
        delete sp;
    }
}
void Cam::delete_all_frames() {
    for (auto &sp : buf) {
        delete sp.second;
    }
}
