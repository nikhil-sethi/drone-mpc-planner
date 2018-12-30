#ifndef COMMON_H
#define COMMON_H

#include "defines.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


cv::Point2f transformPixelToEarth(int x, int y, int centerX, int centerY, float depth, float pix2degx, float pix2degy);
float transformPixelToAngle(float x, float pix2radx);
int getCenterPixel(float angle, float imFOV, int imWidth);
void acc_orientation(float accx, float accy, float accz, float *out);
float scaleStereoHeight(float h);
float getDistance(cv::Point2f p1, cv::Point2f p2);
float transformPixelToAngle(cv::Point2f p, cv::Point2f pix2rad,cv::Point center);
cv::Mat getStabilizedCrop(float phi, float theta, cv::Mat frameL, int sdx, int sdy);
bool checkFileExist (const std::string& name);
void combineImage(cv::Mat iml, cv::Mat imr, cv::Mat *res);
void combineGrayImage(cv::Mat iml,cv::Mat imr,cv::Mat *res);
cv::Mat createColumnImage(std::vector<cv::Mat> ims, int type, float resizef = 1);
cv::Mat createRowImage(std::vector<cv::Mat> ims, int type, float resizef = 1);
void showColumnImage(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
void showRowImage(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
void alert(std::string cmd);
cv::Mat createBlurryCircle(cv::Point size);
std::string to_string_with_precision(float f, const int n);

const float FOV = 180.0f ;
const float FOV_size = 1280.0;
const int width_ff = 1280;
const int height_ff = 960;
const float FOVx_ff = FOV * (static_cast<float>(width_ff)/FOV_size);
const float FOVy_ff = FOV * (static_cast<float>(height_ff)/FOV_size);
const float pix2degx_ff = (FOVx_ff / static_cast<float>(width_ff));
const float pix2degy_ff = (FOVy_ff / static_cast<float>(height_ff));


const int width_dm = 96;
const int height_dm = 96;
const int width_dmB = 576;
const int height_dmB = 576;
const float FOVx_dm = FOVx_ff*(static_cast<float>(width_dmB)/static_cast<float>(width_ff));
const float FOVy_dm = FOVy_ff*(static_cast<float>(height_dmB)/static_cast<float>(height_ff));
const float pix2degx_dm = (FOVx_dm / static_cast<float>(width_dm));
const float pix2degy_dm = (FOVy_dm / static_cast<float>(height_dm));
const float pix2degx_dmB = (FOVx_dm / static_cast<float>(width_dmB));
const float pix2degy_dmB = (FOVy_dm / static_cast<float>(height_dmB));

const float depthscale = 256.0f;

const float rad2deg = 180.f/static_cast<float>(M_PI);
const float deg2rad = static_cast<float>(M_PI)/180.f;


struct trackData {
    float posX,posY,posZ;
    float dx,dy,dz,dt;
    float velX,velY,velZ,accX,accY,accZ;
    float sposX,sposY,sposZ,svelX,svelY,svelZ,saccX,saccY,saccZ;

    bool valid;
    int detected_after_take_off;
};


#endif //COMMON_H

