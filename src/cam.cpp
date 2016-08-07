#include <iostream>
#include <stdio.h>
#include "cam.h"

void Cam::start (void) {
    std::cout << "Starting camera!" << std::endl;
    camRunning=true;
    thread_cam = std::thread(&Cam::workerThread,this);
    waitForImage();
    waitForImage();
    std::cout << "Camera started!" << im_width << "x" << im_height << std::endl;

}

void Cam::waitForImage() {
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage2.lock();
}

void Cam::close(void) {    
    camRunning = false;
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage2.unlock();
    thread_cam.join();
}


void Cam::splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR ) {
    *frameL = cv::Mat(frameC,cv::Rect(0,0,frameC.cols/2,frameC.rows));
    *frameR = cv::Mat(frameC,cv::Rect(frameC.cols/2,0,frameC.cols/2,frameC.rows));
}
