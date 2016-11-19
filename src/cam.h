
#ifndef ICAM_H
#define ICAM_H
#include "defines.h"
#include <mutex>
#include <thread>
#include <opencv2/highgui/highgui.hpp>

#include "stopwatch.h"

/*
 * This class will open a (stereo) video file and stream the data as if it was an real camera
 *
 */
class Cam{

private:

    std::thread thread_cam;
    bool copyNewImage;

protected:
    std::mutex g_lockWaitForImage1;
    std::mutex g_lockWaitForImage2;

    int currentFrame;
    int actualcurrentFrame = 0;

    int im_width;
    int im_height;
    int fps;
    int nFrames;

    bool isStereo = false;
    bool camRunning;

public:
    cv::Mat frameL;
    cv::Mat frameR;

    //public properties:
    int getImWidth() {return im_width;}
    int getImHeight() {return im_height;}
    int getFPS() {return fps;}
    bool getIsStereo() {return isStereo;}
    int getNFrames() {return nFrames;}
    bool getCamRunning() {return camRunning;}
    int getCurrentFrameID() { return actualcurrentFrame;}

    void stopcam (void) {camRunning=false;}
    void waitForImage(void);
    void start (void);
    virtual void close (void);
    //virtual bool init (void) =0;
    virtual void workerThread(void) = 0;
    virtual cv::Mat get_raw_frame(void) = 0;


};
#endif //ICAM_H
