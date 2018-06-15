#ifndef DEFINES_H
#define DEFINES_H

#define VIDEOMODE_DISABLED 0
#define VIDEOMODE_AVI 1
#define VIDEOMODE_STREAM 2 
#define VIDEOMODE_AVI_OPENCV 3

#define SSE2

#define HASSCREEN // dont disable in qt debugger!
#define INSECT_DATA_LOGGING_MODE false
#define BEEP

#define VIDEORAWLR VIDEOMODE_DISABLED
#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 60 // the estimated frame rate of the video used for creating output videos
#define IMG_W 848
#define IMG_H 480
#define IMSCALEF 4
#define CAMERA_ANGLE 32.f

#define DRONE_IM_X_START 424/IMSCALEF
#define DRONE_IM_Y_START 420/IMSCALEF
#define DRONE_DISPARITY_START 36

#define MAX_BORDER_Y_DEFAULT 2.1f
#define MAX_BORDER_Z_DEFAULT 4.0f

#endif //DEFINES_H
