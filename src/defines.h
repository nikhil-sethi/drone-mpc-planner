#ifndef DEFINES_H
#define DEFINES_H

#define VIDEOMODE_DISABLED 0
#define VIDEOMODE_AVI 1
#define VIDEOMODE_STREAM 2 
#define VIDEOMODE_AVI_OPENCV 3

#define SSE2

#define HASSCREEN // dont disable in qt debugger!
#define INSECT_DATA_LOGGING_MODE true
#define BEEP


#define VIDEORAWLR VIDEOMODE_AVI
#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 30 // the estimated frame rate of the video used for creating output videos
#define IMG_W 848
#define IMG_H 480
#define IMSCALEF 4

#define DRONE_IM_X_START 424/IMSCALEF
#define DRONE_IM_Y_START 360/IMSCALEF
#define DRONE_DISPARITY_START 64/IMSCALEF


#endif //DEFINES_H
