
#ifndef STEREOALG_H
#define STEREOALG_H

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

#include "defines.h"


class stereoAlg{


private:
    cv::Mat map11, map12, map21, map22;
    cv::StereoSGBM sgbm;
    float dispScale;

    const int ROIsize = 576;

public:
    cv::Mat frameD;
    cv::Mat frameLrect,frameRrect;
    float avgDisparity;
    cv::Mat frameC;

    bool init (std::string calib_folder);
    void combineImage(cv::Mat iml,cv::Mat imr);
	void rectify(cv::Mat frameL,cv::Mat frameR);
    void calcDisparityMap();



};


#endif //STEREOALG_H
