#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>

int sign(float x) {
    if (x < 0)
        return -1;
    return 1;
}

vector<string> split_csv_line(string line) {
    stringstream liness(line);
    vector<string> line_data;
    while (!liness.eof()) {
        string tmp;
        getline(liness, tmp, ';');
        line_data.push_back(tmp);
    }
    return line_data;
}
vector<string> split_csv_line(string line, char denominator) {
    stringstream liness(line);
    vector<string> line_data;
    while (!liness.eof()) {
        string tmp;
        getline(liness, tmp, denominator);
        line_data.push_back(tmp);
    }
    return line_data;
}


//strips disparity from world2im_3d
cv::Point2f world2im_2d(cv::Point3f w, cv::Mat Qfi, float camera_roll, float camera_pitch) {
    cv::Point3f p_im = world2im_3d(w, Qfi, camera_roll, camera_pitch);
    return cv::Point2f(p_im.x, p_im.y);
}

cv::Point3f world2im_3d(cv::Point3f w, cv::Mat Qfi, float camera_roll, float camera_pitch) {
    float phi = camera_roll * deg2rad;
    float theta = camera_pitch * deg2rad;
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);

    cv::Matx33f R_pitch_mat(1, 0,         0,
                            0, cos_theta, -sin_theta,
                            0, sin_theta, cos_theta);
    cv::Matx33f R_roll_mat(cos_phi,  sin_phi, 0,
                           -sin_phi, cos_phi, 0,
                           0,        0,       1);
    cv::Point3f w_rot_f = R_roll_mat * (R_pitch_mat * w);
    cv::Point3d w_rot_d(w_rot_f.x, w_rot_f.y, w_rot_f.z); // convert to double because perspectiveTransform gives wrong results

    std::vector<cv::Point3d> world_coordinates, camera_coordinates;
    world_coordinates.push_back(w_rot_d);
    cv::perspectiveTransform(world_coordinates, camera_coordinates, Qfi);
    camera_coordinates.at(0).z = -camera_coordinates.at(0).z;

    return cv::Point3f(camera_coordinates.at(0).x, camera_coordinates.at(0).y, camera_coordinates.at(0).z); // convert to float
}

cv::Point3f im2world(cv::Point2f p_im, float disparity, cv::Mat Qf, float camera_roll, float camera_pitch) {
    std::vector<cv::Point3d> camera_coordinates, world_coordinates;
    camera_coordinates.push_back(cv::Point3d(p_im.x, p_im.y, -disparity));
    cv::perspectiveTransform(camera_coordinates, world_coordinates, Qf);
    cv::Point3f w(world_coordinates[0].x, world_coordinates[0].y, world_coordinates[0].z); // convert the output to floats because perspectiveTransform only does doubles but we want floats:

    float phi = -camera_roll * deg2rad;
    float theta = -camera_pitch * deg2rad;
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);
    cv::Matx33f R_pitch(1, 0,         0,
                        0, cos_theta, -sin_theta,
                        0, sin_theta, cos_theta);
    cv::Matx33f R_roll(cos_phi,  sin_phi, 0,
                       -sin_phi, cos_phi, 0,
                       0,        0,       1);
    cv::Point3f w_rot = R_pitch * (R_roll * w);
    return w_rot;
}

int world2im_dist(cv::Point3f p1, float dist, cv::Mat Qfi, float camera_roll, float camera_pitch) {
    return world2im_size(p1 - cv::Point3f(dist, 0, 0), p1 + cv::Point3f(dist, 0, 0), Qfi, camera_roll, camera_pitch);
}
//returns the pixel size of a world object
int world2im_size(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_roll, float camera_pitch) {
    return round(normf(world2im_2d(p1, Qfi, camera_roll, camera_pitch) - world2im_2d(p2, Qfi, camera_roll, camera_pitch)));
}

float world2im_sizef(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_roll, float camera_pitch) {
    return normf(world2im_2d(p1, Qfi, camera_roll, camera_pitch) - world2im_2d(p2, Qfi, camera_roll, camera_pitch));
}

bool file_exist(const std::string &name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool path_exist(const std::string &s) {
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}

//combines a sperate left and right image into one combined concenated image
void combine_image(cv::Mat iml, cv::Mat imr, cv::Mat *res) {

    *res = cv::Mat(iml.rows, iml.cols + imr.cols, CV_8UC3);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols + imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

//combines a sperate left and right image into one combined concenated image
void combine_gray_image(cv::Mat iml, cv::Mat imr, cv::Mat *res) {

    *res = cv::Mat(iml.rows, iml.cols + imr.cols, CV_8UC1);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols + imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

cv::Mat create_row_image(std::vector<cv::Mat> ims, int type, float resizef) {
    //find max height and total width:
    int width = 0;
    int height = -1;
    for (size_t i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).rows > height)
        {
            height = ims.at(i).rows;
        }
        width += ims.at(i).cols;
    }

    cv::Mat res = cv::Mat(height * resizef, width * resizef, type);

    cv::Point p1(0, 0);

    for (size_t i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(p1.x + im.cols * resizef, im.rows * resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));
        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
            else
                cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
        }

        cv::resize(im, roi, cv::Size(im.cols * resizef, im.rows * resizef));
        if (i < ims.size() - 1)
            cv::line(roi, cv::Point(roi.cols - 1, 0), cv::Point(roi.cols - 1, roi.rows - 1), cv::Scalar(128, 128, 128));

        p1.x += im.cols * resizef;
    }
    return res;
}

cv::Mat create_column_image(std::vector<cv::Mat> ims, int type, float resizef) {
    //find max width and total height:
    int width = -1;
    int height = 0;
    for (size_t i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).cols > width)
        {
            width = ims.at(i).cols;
        }
        height += ims.at(i).rows;
    }

    cv::Mat res = cv::Mat::zeros(height * resizef, width * resizef, type);

    cv::Point p1(0, 0);

    for (size_t i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(im.cols * resizef, p1.y + im.rows * resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));

        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
            else
                cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
        }

        cv::resize(im, roi, cv::Size(im.cols * resizef, im.rows * resizef));
        if (i < ims.size() - 1)
            cv::line(roi, cv::Point(0, roi.rows - 1), cv::Point(roi.cols, roi.rows - 1), cv::Scalar(128, 128, 128));
        p1.y += im.rows * resizef;
    }
    return res;
}

/* combines a bunch of images into one column, and shows it */
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef) {
    cv::Mat res = create_column_image(ims, type, resizef);
    cv::imshow(window_name, res);
}

/* combines a bunch of images into one row, and shows it */
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef) {
    cv::Mat res = create_row_image(ims, type, resizef);
    cv::imshow(window_name, res);
}
cv::Rect clamp_rect(cv::Rect r, int w, int h) {
    r.x = std::clamp(r.x, 0, w - 1); // rect width must be atleast 1, so w-1
    r.y = std::clamp(r.y, 0, h - 1); // idem
    if (r.x + r.width >= w)
        r.width = w - r.x;
    if (r.y + r.height >= h)
        r.height = h - r.y;
    return r;
}

std::string to_string_with_precision(float f, const int n) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

std::string to_string_with_precision(double f, const int n) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

float normf(cv::Point2f m) { return static_cast<float>(cv::norm(m));}
float normf(cv::Point3f m) { return static_cast<float>(cv::norm(m));}

cv::Point3f multf(cv::Point3f  p1, cv::Point3f p2) {
    cv::Point3f p;
    p.x = p1.x * p2.x;
    p.y = p1.y * p2.y;
    p.z = p1.z * p2.z;
    return p;
}

cv::Point3f deadzone(cv::Point3f p, float lo, float hi) {
    return cv::Point3f(deadzone(p.x, lo, hi), deadzone(p.y, lo, hi), deadzone(p.z, lo, hi));
}
float deadzone(float v, float lo, float hi) {
    if (v < 0 && v > lo)
        v = 0;
    else if (v > 0 && v < hi)
        v = 0;
    return v;
}

cv::Point3f pats_to_betaflight_coord(cv::Point3f vec) {
    return cv::Point3f(-vec.z, -vec.x, -vec.y);
}
cv::Point3f betaflight_to_pats_coord(cv::Point3f vec) {
    return cv::Point3f(-vec.y, -vec.z, -vec.x);
}

std::string execute(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;

    auto pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");

    while (!feof(pipe)) {
        if (fgets(buffer.data(), 128, pipe) != nullptr)
            result += buffer.data();
    }
    return result;
}

float max_rs_auto_exposure() {
    if (pparams.fps == 90)
        return 10000;
    else if (pparams.fps == 60)
        return 15000;
    else
        return 20000;
}

float calc_light_level(int exposure, int gain, float brightness) {
    const float max_rs_gain = 248.f;
    const float ultimate_max_rs_exposure = 20e3f;
    const float min_rs_gain = 16.f;
    const float min_rs_exposure = 1.f;

    const float light_level_scale = 20 * log10((255 / (min_rs_exposure * min_rs_gain)) * (ultimate_max_rs_exposure * max_rs_gain));

    float light_level_db = 20 * log10((brightness / (exposure * gain)) * (ultimate_max_rs_exposure * max_rs_gain));
    return light_level_db / light_level_scale; // scale between 0-1
}
