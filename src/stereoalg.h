
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

    const int ROIsize = 864/IMSCALEF;

public:
    cv::Mat frameD;
    cv::Mat frameLrect,frameRrect;
    cv::Mat Qf;
    float avgDisparity;    

    bool init (std::string calib_folder);
    void combineImage(cv::Mat iml,cv::Mat imr,cv::Mat *frameC);
	void rectify(cv::Mat frameL,cv::Mat frameR);
    void calcDisparityMap();



};


#endif //STEREOALG_H
