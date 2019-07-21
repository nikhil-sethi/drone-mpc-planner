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
    int frame_number() {return _frame_number;}
    float frame_time() {return _frame_number * (1.f/VIDEOFPS);}
    float camera_angle(){return 35.f;}

    void close (void);
    bool init ();
    bool init (int argc __attribute__((unused)), char **argv __attribute__((unused)));
    void update (void);
    bool get_cam_is_running() {return camRunning;}
    float measured_gain() {
        return 16;
    }

    void nextFrame();
    void pause() {paused = true;};
    void resume() {paused = false;};
    void reset () {}

    cv::Mat Qf;
    cv::Mat depth_background_mm;

private:
#define DRONE_IM_X_START  100
#define DRONE_IM_Y_START 100

    int nFrames_stereo,_frame_number,frame_id_rgb,frame_id_stereo;
    bool camRunning;

    cam_mode_enum _mode = cam_mode_stopped;
    bool paused = false;

    cv::Point drone;
    cv::Point insect;

    void generateStereo();
    void generateRGB();
    void bound(cv::Point *p, int radius);

};
#endif //GENERATORCAM_H
