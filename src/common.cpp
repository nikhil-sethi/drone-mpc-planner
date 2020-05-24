#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>

int sign(float x) {
    if (x<0)
        return -1;
    return 1;
}
//strips disparity from world2im_3d
cv::Point2f world2im_2d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle) {
    cv::Point3f p_im = world2im_3d(p_world,Qfi,camera_angle);
    return cv::Point2f(p_im.x,p_im.y);
}

//returns image coordinates from a world coordinate: x,y,disparity
cv::Point3f world2im_3d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle) {
    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    //derotate camera and convert to double:
    cv::Point3d tmpd (p_world.x,p_world.y,p_world.z);
    float theta = -camera_angle * deg2rad;
    float temp_y = p_world.y * cosf(theta) + p_world.z * sinf(theta);
    tmpd.z = -p_world.y * sinf(theta) + p_world.z * cosf(theta);
    tmpd.y = temp_y;
    tmpd.x = p_world.x;

    world_coordinates.push_back(tmpd);
    cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);

    camera_coordinates.at(0).z = -camera_coordinates.at(0).z; // negate disparity
    return camera_coordinates.at(0);
}

//returns image coordinates from a world coordinate: x,y,disparity
cv::Point3f im2world(cv::Point2f p_im,float disparity, cv::Mat Qf, float camera_angle) {
    std::vector<cv::Point3d> camera_coordinates, world_coordinates;
    camera_coordinates.push_back(cv::Point3d(p_im.x,p_im.y,-disparity));
    cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

    cv::Point3f w;
    w.x = world_coordinates[0].x;
    w.y = world_coordinates[0].y;
    w.z = world_coordinates[0].z;
    //compensate camera rotation:
    float theta = camera_angle * deg2rad;
    float temp_y = w.y * cosf(theta) + w.z * sinf(theta);
    w.z = -w.y * sinf(theta) + w.z * cosf(theta);
    w.y = temp_y;

    return w;
}
//returns the pixel size of a world object]
int world2im_size(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_angle) {
    return round(cv::norm(world2im_2d(p1,Qfi,camera_angle)-world2im_2d(p2,Qfi,camera_angle)));
}

float world2im_sizef(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_angle) {
    return cv::norm(world2im_2d(p1,Qfi,camera_angle)-world2im_2d(p2,Qfi,camera_angle));
}


bool file_exist (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool path_exist(const std::string &s) {
    struct stat buffer;
    return (stat (s.c_str(), &buffer) == 0);
}

//combines a sperate left and right image into one combined concenated image
void combine_image(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

    *res = cv::Mat(iml.rows,iml.cols + imr.cols,CV_8UC3);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols+imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

//combines a sperate left and right image into one combined concenated image
void combine_gray_image(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

    *res = cv::Mat(iml.rows,iml.cols + imr.cols,CV_8UC1);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols+imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

cv::Mat create_row_image(std::vector<cv::Mat> ims, int type,float resizef)
{
    //find max height and total width:
    int width = 0;
    int height = -1;
    for (uint i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).rows > height)
        {
            height = ims.at(i).rows;
        }
        width += ims.at(i).cols;
    }

    cv::Mat res = cv::Mat(height*resizef, width*resizef, type);

    cv::Point p1(0, 0);

    for (uint i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(p1.x + im.cols*resizef, im.rows*resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));
        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
            else
                cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(roi.cols-1,0),cv::Point(roi.cols-1,roi.rows-1),cv::Scalar(128,128,128));

        p1.x += im.cols*resizef;
    }
    return res;
}

cv::Mat create_column_image(std::vector<cv::Mat> ims, int type,float resizef)
{
    //find max width and total height:
    int width = -1;
    int height = 0;
    for (uint i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).cols > width)
        {
            width = ims.at(i).cols;
        }
        height += ims.at(i).rows;
    }

    cv::Mat res = cv::Mat::zeros(height*resizef, width*resizef, type);

    cv::Point p1(0, 0);

    for (uint i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(im.cols*resizef, p1.y + im.rows*resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));

        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
            else
                cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(0,roi.rows-1),cv::Point(roi.cols,roi.rows-1),cv::Scalar(128,128,128));
        p1.y += im.rows*resizef;
    }
    return res;
}

/* combines a bunch of images into one column, and shows it */
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef) {
    cv::Mat res = create_column_image(ims,type,resizef);
    cv::imshow(window_name, res);
}

/* combines a bunch of images into one row, and shows it */
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type,float resizef) {
    cv::Mat res = create_row_image(ims,type,resizef);
    cv::imshow(window_name, res);
}

std::string to_string_with_precision(float f, const int n)
{
    std::ostringstream out;

    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

std::string to_string_with_precision(double f, const int n)
{
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

cv::Point3f deadzone(cv::Point3f p,float lo, float hi) {
    return cv::Point3f(deadzone(p.x,lo,hi),deadzone(p.y,lo,hi),deadzone(p.z,lo,hi));
}
float deadzone( float v, float lo, float hi ) {
    if (v < 0 && v > lo )
        v = 0;
    else if (v > 0 && v < hi )
        v = 0;
    return v;
}

cv::Point3f pats_to_betaflight_coord(cv::Point3f vec) {
    return cv::Point3f(-vec.z, -vec.x, -vec.y);
}
cv::Point3f betaflight_to_pats_coord(cv::Point3f vec) {
    return cv::Point3f(-vec.y, -vec.z, -vec.x);
}
