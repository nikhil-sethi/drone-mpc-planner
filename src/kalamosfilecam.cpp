#include "kalamosfilecam.h"

#include <iostream>
#include <fstream>      // std::ifstream
#include <unistd.h>       //usleep
#include <iomanip>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

inline bool fileExists (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

int skipframes = 300;
bool KalamosFileCam::init (std::string folder) {
    this->folder = folder;
    fps = VIDEOFPS;

    std::cout << "Kalamos init succes!" << std::endl;
    std::stringstream sLR;
    sLR << folder << "/videoRawLR.avi";
    bool res = fileExists(sLR.str());
    std::stringstream sD;
    sD << folder << "/videoDisp.avi";
    res &= fileExists(sD.str());

    videoLR =cv::VideoCapture (sLR.str());

    if (!videoLR.isOpened()) {
        std::cerr << "Error opening video raw LR file!\n";
        return true;
    } else {
        im_width = (int) videoLR.get(CV_CAP_PROP_FRAME_WIDTH)/2;
        im_height = (int)videoLR.get(CV_CAP_PROP_FRAME_HEIGHT);
        nFrames = (int) videoLR.get(CV_CAP_PROP_FRAME_COUNT);
        //videoLR.set(CV_CAP_PROP_POS_FRAMES,0);

        std::cout << "Opened filecam LR, nFrames: " << nFrames << std::endl;
        currentFrame=0;
    }

    if (res) {
        return 0;
    } else {
        std::cerr << "Error: no image found" << std::endl;
        return 1;
    }
}

void KalamosFileCam::workerThread(void) {
    frameL = cv::Mat::zeros(960,1280,CV_8UC3);
    frameR = cv::Mat::zeros(960,1280,CV_8UC3);
    cv::Mat frameLR(960,2560,CV_8UC3);
    std::cout << "Starting main fileKalamos loop!" << std::endl;
    while (skipframes>0) {
        videoLR >> frameLR;
        skipframes--;
        currentFrame++;
    }

    while (camRunning)  {
        usleep(round(1000000.0/(float)VIDEOFPS)); // simulate framerate
        //std::cout << "t: " << round(1000.0/(float)VIDEOFPS) << " --- " << std::endl << slr.str() << std::endl << sd.str() << std::endl;

        g_lockWaitForImage1.lock();
        videoLR >> frameLR;

        if (!frameLR.empty()) {
            currentFrame++;
            im_width = frameLR.cols / 2;
            im_height = frameLR.rows;
            frameL = frameLR(cv::Range(0, im_height),cv::Range(0, im_width));
            frameR = frameLR(cv::Range(0, im_height),cv::Range(im_width, 2*im_width));
            g_lockWaitForImage2.unlock();
        } else {
            camRunning=false;
            break;
        }
    } // while loop

    std::cout << "Camera loop stopped." << std::endl;
    g_lockWaitForImage2.unlock();

    std::cout << "Exiting camera thread." << std::endl;
}


cv::Mat KalamosFileCam::get_raw_frame() {
    return frameL;
}

cv::Mat KalamosFileCam::get_combined() {
    return frameL;
}

void KalamosFileCam::close (void) {
    Cam::close();
    videoLR.release();
}

