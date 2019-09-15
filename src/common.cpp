#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>

void CameraVolume::init(cv::Point3f xblu, cv::Point3f xbru, cv::Point3f xbld, cv::Point3f xbrd, cv::Point3f xfld, cv::Point3f xfrd){
    std::vector<cv::Point3f> back_plane = calc_planeNormVec (xblu, xbru, xbld);
    back_plane.at(1) *= (-1);
    planes.push_back(back_plane);

    std::vector<cv::Point3f> front_plane = calc_planeNormVec (cv::Point3f(0,0,0), xfld, xfrd);
    front_plane.at(1) *= (-1);
    planes.push_back(front_plane);

    std::vector<cv::Point3f> left_plane = calc_planeNormVec (xblu, xbld, xfld);
    left_plane.at(1) *= (-1);
    planes.push_back(left_plane);

    std::vector<cv::Point3f> right_plane = calc_planeNormVec (xbru, xbrd, xfrd);
    planes.push_back(right_plane);

    std::vector<cv::Point3f> top_plane = calc_planeNormVec (xblu, xbru, cv::Point3f(0,0,0));
    planes.push_back(top_plane);

    std::vector<cv::Point3f> bottom_plane = calc_planeNormVec (xbld, xbrd, xfld);
    bottom_plane.at(0) *= (-1);
    planes.push_back(bottom_plane);

}

std::vector<cv::Point3f> CameraVolume::calc_planeNormVec(cv::Point3f x1, cv::Point3f x2, cv::Point3f x3){
    cv::Point3f vec_a = x2- x1;
    cv::Point3f vec_b = x3- x1;

    cv::Point3f n = vec_a.cross(vec_b);
    cv::Point3f p0 = x1;

    return {p0, n};
}

bool CameraVolume::is_inView(cv::Point3f p){
    for(uint i=0; i<planes.size(); i++){
        float dot_prd = (p-planes.at(i).at(0)).dot(planes.at(i).at(1));
        if(dot_prd<0){
            return false;
        }
    }
    return true;
}

cv::Point2f world2im(cv::Point3f p, cv::Mat Qfi, float camera_angle){
    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    //derotate camera and convert to double:
    cv::Point3d tmpd (p.x,p.y,p.z);
    float theta = -camera_angle * deg2rad;
    float temp_y = p.y * cosf(theta) + p.z * sinf(theta);
    tmpd.z = -p.y * sinf(theta) + p.z * cosf(theta);
    tmpd.y = temp_y;
    tmpd.x = p.x;

    world_coordinates.push_back(tmpd);
    cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);

    cv::Point2f res(camera_coordinates.at(0).x,camera_coordinates.at(0).y);
    return res;
}

void acc_orientation(float accx, float accy, float accz, float *out) {
    float R;
    float tx,ty,tz,pitch,roll;
    R = sqrtf(accx * accx + accy * accy +accz * accz);

    tx = acosf(accx/R)-M_PI_2f32;
    ty = acosf(accy/R)-M_PI_2f32;
    tz = acosf(accz/R)-M_PI_2f32;
    pitch = atan2f(-ty, -tz) + M_PI_2f32;
    roll =  atan2f(-tz, -tx) + M_PI_2f32;
    out[0] = roll;
    out[1] = pitch;
    out[2] = 0;
}

bool checkFileExist (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

//combines a sperate left and right image into one combined concenated image
void combineImage(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

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
void combineGrayImage(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

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

cv::Mat createRowImage(std::vector<cv::Mat> ims, int type,float resizef)
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
                cv::cvtColor(im, im, CV_BGR2GRAY);
            else
                cv::cvtColor(im, im, CV_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(roi.cols-1,0),cv::Point(roi.cols-1,roi.rows-1),cv::Scalar(128,128,128));

        p1.x += im.cols*resizef;
    }
    return res;
}

cv::Mat createColumnImage(std::vector<cv::Mat> ims, int type,float resizef)
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
                cv::cvtColor(im, im, CV_BGR2GRAY);
            else
                cv::cvtColor(im, im, CV_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(0,roi.rows-1),cv::Point(roi.cols,roi.rows-1),cv::Scalar(128,128,128));
        p1.y += im.rows*resizef;
    }
    return res;
}

/* combines a bunch of images into one column, and shows it */
void showColumnImage(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef) {
    cv::Mat res = createColumnImage(ims,type,resizef);
    cv::imshow(window_name, res);
}

/* combines a bunch of images into one row, and shows it */
void showRowImage(std::vector<cv::Mat> ims, std::string window_name, int type,float resizef) {
    cv::Mat res = createRowImage(ims,type,resizef);
    cv::imshow(window_name, res);
}

cv::Mat createBlurryCircle(cv::Point size) {
    cv::Point2f tmp;
    tmp.x = roundf((static_cast<float>(size.x))/4.f);
    tmp.y = roundf((static_cast<float>(size.y))/4.f);
    if (fabs((tmp.x / 2.f) - roundf(tmp.x / 2.f)) < 0.01f)
        tmp.x +=1;
    if (fabs((tmp.y / 2.f) - roundf(tmp.y / 2.f)) < 0.01f)
        tmp.y +=1;

    cv::Mat res = cv::Mat::zeros(size.y,size.x,CV_32F);

    cv::ellipse(res,cv::Point(size.x/2,size.y/2),cv::Size(tmp.x,tmp.y),0,0,360,1.0f,CV_FILLED);
    cv::GaussianBlur( res, res, cv::Size( tmp.x, tmp.y ), 0, 0 );
    return res;
}

std::string to_string_with_precision(float f, const int n)
{
    std::ostringstream out;

    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

int getSecondsSinceFileCreation(std::string filePath)
{
    if (!checkFileExist(filePath))
        return std::numeric_limits<int>::max();
    struct stat attrib;
    stat(filePath.c_str(), &attrib);
//    auto t1 = localtime(&(attrib.st_ctime));
//    std::cout << std::asctime( localtime(&(attrib.st_ctime)));
    auto curtime=  std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(curtime);
//    std::cout << std::asctime(std::localtime(&in_time_t)) << std::endl;
//    auto now = std::localtime(&in_time_t);
    double diff = difftime(in_time_t,attrib.st_ctime);
    return static_cast<int>(diff);
}




