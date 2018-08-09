#ifndef FILECAM_H
#define FILECAM_H
#include "defines.h"
#include <opencv2/highgui/highgui.hpp>


class FileCam {

public:

    enum cam_mode_enum{cam_mode_stopped = 0, cam_mode_disabled = 1, cam_mode_color = 2, cam_mode_stereo=3 };
    cv::Mat frameLR,frameL,frameR,frameD,frame_rgb;

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
    std::string file_rgb, file_stereo;


    cv::VideoCapture video_rgb, video_stereo;

    int skipstart = 0; //1810;
    int videoLength_rgb,videoLength_stereo;
    int im_width_rgb, im_height_rgb,nFrames_rgb,nFrames_stereo,frame_id,frame_id_rgb,frame_id_stereo;
    bool camRunning;

    cam_mode_enum _mode = cam_mode_stopped;
    float frame_time;
    bool paused = false;


};
#endif //FILECAM_H
