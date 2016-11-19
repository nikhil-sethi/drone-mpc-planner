#include "common.h"

cv::Point2f transformPixelToEarth(int x, int y, int centerX, int centerY, float depth, float pix2radx,float pix2rady) {
    //calculate pixel to angle:
    float radX = float(x - centerX)  * pix2radx*(M_PI/180);
    float radY = float(y - centerY)  * pix2rady*(M_PI/180);

    //calculate distance on ground
    float disX = -tan(radX) * depth;
    float disY = tan(radY) * depth;

    cv::Point2f p(disX,disY);
    return p;
}

float transformPixelToAngle(float x, float pix2radx) {
    //calculate pixel to angle:
    float angle = x * pix2radx;
    return angle;
}

float transformPixelToAngle(cv::Point2f p, cv::Point2f pix2rad,cv::Point center) {
    cv::Point2f pt (p.x-center.x,p.y-center.y);

    if (pt.x > pt.y)
        return pt.x * pix2rad.x;
    else
        return pt.y * pix2rad.y;
}

int getCenterPixel(float angle, float imFOV, int imWidth) {
    //calculate the center pixel based on the angle measured by an IMU
    //the 0.5 is because the IMU goes from [-PI .. PI], while the image goes from [0 .. imsize]
    return round(((angle/(imFOV/FOV)) / M_PI + 0.5) * imWidth);
}

void acc_orientation(float accx, float accy, float accz, float *out) {
    float R;
    float tx,ty,tz,pitch,roll;
    R = sqrt((float)accx * (float)accx + (float)accy * (float)accy +(float)accz * (float)accz);
    float pi2 = 0.5 * M_PI;
    tx = acos(accx/R)-pi2;
    ty = acos(accy/R)-pi2;
    tz = acos(accz/R)-pi2;
    pitch = atan2(-ty, -tz) + M_PI;
    roll =  atan2(-tz, -tx) + M_PI;
    out[0] = roll;
    out[1] = pitch;
    out[2] = 0;
}

float scaleStereoHeight(float height) {
    float h = height/depthscale;
    if (h > 20 ) {
        h = sqrt((h - 15.0)) +15.0;
    }
    return h;
}


float getDistance(cv::Point2f p1, cv::Point2f p2) {
    cv::Point2f p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;

    return sqrt(p.x*p.x + p.y*p.y);

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
