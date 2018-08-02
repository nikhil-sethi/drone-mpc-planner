#ifndef GENERATORCAM_H
#define GENERATORCAM_H
#include "defines.h"
#include <opencv2/highgui/highgui.hpp>


class GeneratorCam {

public:

    enum cam_mode_enum{cam_mode_stopped = 0, cam_mode_disabled = 1, cam_mode_color = 2, cam_mode_stereo=3 };
    cv::Mat frameL,frameR,frameD,frame_rgb;

    cam_mode_enum get_mode() {return _mode;}
    void switch_mode(cam_mode_enum mode);
    void go_stereo();
    void go_color();
    void go_disabled();
    int get_frame_id() {return frame_id;}
    float get_frame_time() {return frame_id * (1.f/VIDEOFPS);}

    void close (void);
    bool init (int argc, char **argv);
    void update (void);
    bool get_cam_is_running() {return camRunning;}

    void nextFrame();
    void pause() {paused = true;};
    void resume() {paused = false;};

    cv::Mat Qf;

private:
    int nFrames_stereo,frame_id,frame_id_rgb,frame_id_stereo;
    bool camRunning;

    cam_mode_enum _mode = cam_mode_stopped;
    float frame_time;
    bool paused = false;

    cv::Point drone;
    cv::Point insect;

    void generateStereo();
    void generateRGB();
    void bound(cv::Point *p, int radius);

};
#endif //GENERATORCAM_H
