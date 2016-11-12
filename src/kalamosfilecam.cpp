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

int skipframes = 0;
bool KalamosFileCam::init (std::string folder) {
    this->folder = folder;
    fps = VIDEOFPS;


    std::cout << "Kalamos init succes!" << std::endl;
    std::stringstream sL;
    sL << folder << "/videoRawL.avi";
    bool res = fileExists(sL.str());
    std::stringstream sD;
    sD << folder << "/videoDisp.avi";
    res &= fileExists(sD.str());

    videoL =cv::VideoCapture (folder + "/videoRawL.avi");
    videoR =cv::VideoCapture (folder + "/videoRawR.avi");


    if (!videoL.isOpened()) {
        std::cerr << "Error opening video L file!\n";
        return true;
    } else {
        im_width = (int) videoL.get(CV_CAP_PROP_FRAME_WIDTH);
        im_height = (int)videoL.get(CV_CAP_PROP_FRAME_HEIGHT);
        nFrames = (int) videoL.get(CV_CAP_PROP_FRAME_COUNT);
        //videoL.set(CV_CAP_PROP_POS_FRAMES,0);

        std::cout << "Opened filecam L, nFrames: " << nFrames << std::endl;
        currentFrame=0;
        return false;
    }

    if (!videoR.isOpened()) {
        std::cerr << "Error opening video R file!\n";
        return true;
    } else {
        std::cout << "Opened filecam R, nFrames: " << nFrames << std::endl;
        return false;
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
    std::cout << "Starting main fileKalamos loop!" << std::endl;
    while (skipframes>0) {
        videoL >> frameL;
        videoR >> frameR;
        skipframes--;
        currentFrame++;
    }

    while (camRunning)  {
        usleep(round(1000000.0/(float)VIDEOFPS)); // simulate framerate
        //std::cout << "t: " << round(1000.0/(float)VIDEOFPS) << " --- " << std::endl << slr.str() << std::endl << sd.str() << std::endl;

        g_lockWaitForImage1.lock();
        videoL >> frameL;
        videoR >> frameR;

        if (!frameL.empty())
        {
            currentFrame++;
            im_width = frameL.cols;
            im_height = frameL.rows;
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
}

