#ifndef VIZDAT_H
#define VIZDAT_H


#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"

#include "cam.h"
#include "defines.h"

class VisionData{

private:


public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;

    cv::Size smallsize;
    cv::Mat frameL_small;
    cv::Mat frameL_s_prev;

    cv::Mat threshL;
    cv::Mat Qf;
    void init(cv::Mat Qf, cv::Mat frameL,cv::Mat frameR){
        this->Qf = Qf;
        this->frameL = frameL;
        this->frameR = frameR;
        this->frameL_prev = frameL;
        this->frameR_prev = frameR;

        smallsize =cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
        cv::resize(frameL,frameL_small,smallsize);
        frameL_s_prev = frameL_small.clone();

    }

    void update(cv::Mat frameL,cv::Mat frameR) { // TODO: get rid of this function by direct pointers...?
        this->frameL_prev = this->frameL.clone();
        this->frameR_prev = this->frameR.clone();
        this->frameL = frameL;
        this->frameR = frameR;

        frameL_s_prev = frameL_small.clone();
        cv::resize(frameL,frameL_small,smallsize);

    }

};

#endif // VIZDAT_H
