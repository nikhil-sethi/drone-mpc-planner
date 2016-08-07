#include "filecam.h"
#include <iostream>
#include <string.h>
#include <unistd.h> // sleep
#include <opencv2/contrib/contrib.hpp>


bool FileCam::init() {
    return false;
}

bool FileCam::initFile (std::string file, int nskip) {
    this->file = file;

    skipstart = nskip;
    videoLength = 999999;
    video = cv::VideoCapture(file);

    if (!video.isOpened()) {
        std::cerr << "Error opening video file!\n";
        return true;
    } else {
        im_width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH);
        im_height = (int)video.get(CV_CAP_PROP_FRAME_HEIGHT);
        nFrames = (int) video.get(CV_CAP_PROP_FRAME_COUNT);
        video.set(CV_CAP_PROP_POS_FRAMES,0);
        if (videoLength > nFrames) {
            videoLength = nFrames;
        }
        std::cout << "Opened filecam, nFrames: " << nFrames << std::endl;
        currentFrame=0;
        return false;
    }
}

void FileCam::close () {    
    Cam::close();
    video.release();
}

void FileCam::workerThread() {

    cv::Mat frameC = cv::Mat::zeros(im_height,im_width*2, CV_8UC1);
    stopWatch.Start();

    //skip start
    for (int i =0; i < skipstart;i++) {
        video >> frameC;
        currentFrame++;
    }
//    video.set(CV_CAP_PROP_POS_FRAMES,skipstart);
//    currentFrame=skipstart;

    while (camRunning)  {

#ifdef HASSCREEN
        //arrange speed to be ~10fps
        float time = stopWatch.Read();
        time = 1000/(float)VIDEOFPS - time/1000;
        if (fastforward==0) {
            if (time > 0)  {usleep((int)time*1000);}
        }
        stopWatch.Restart();
#endif

        if (rewind==1) {
//            int id = video.get(CV_CAP_PROP_POS_FRAMES);
//            currentFrame-=2;
//            if (id>0) {
//                id-=2;
//            }
//            video.set(CV_CAP_PROP_POS_FRAMES,id);

            video.release();
            video = cv::VideoCapture(file);
            if (currentFrame > 50) {
                currentFrame -= 50;
            } else {
                currentFrame = 0;
            }
            for (int i =0; i < currentFrame;i++) {
                video >> frameC;
            }
            rewind=0;
        }

        video >> frameC;
        g_lockWaitForImage1.lock();
        currentFrame++;

        if (frameC.empty() || currentFrame >= videoLength)
        {
            camRunning=false;
            g_lockWaitForImage2.unlock();
            break;
        }
        frameC.copyTo(frameL_mat);
#ifdef DELFLY
        cvtColor(frameC,frameC,CV_RGB2GRAY,CV_8UC1);
        splitIm(frameC,&frameL,&frameR);

        cv::Point size(frameL.cols,frameL.rows);
        cv::resize(frameL,frameL_mat,size);


        frameL.copyTo(frameL_mat);
        frameR.copyTo(frameR_mat);
#endif

        g_lockWaitForImage2.unlock();

    } // while loop
}
