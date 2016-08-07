#include <iostream>
#include <stdio.h>
#include "cam.h"

void Cam::start (void) {
    std::cout << "Starting camera!" << std::endl;
    camRunning=true;
    thread_cam = std::thread(&Cam::workerThread,this);
    waitForImage();
    waitForImage();
	im_width = frameL_mat.cols;
	im_height = frameL_mat.rows;
    fps = VIDEOFPS;
    std::cout << "Camera started! " << im_width << "x" << im_height << std::endl;

}

void Cam::waitForImage() {
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage2.lock();
}

void Cam::close(void) {    
    camRunning = false;
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage2.unlock();
    //thread_cam.join();
}


