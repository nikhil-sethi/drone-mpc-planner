#ifndef DEFINES_H
#define DEFINES_H

#define CAMMODE_REALSENSE 0
#define CAMMODE_FROMVIDEOFILE 1
#define CAMMODE_AIRSIM 2
#define CAMMODE_GENERATOR 3

#define VIDEOMODE_DISABLED 0
#define VIDEOMODE_AVI 1
#define VIDEOMODE_STREAM 2
#define VIDEOMODE_AVI_OPENCV 3

#define SSE2

#define HASSCREEN // dont disable in qt debugger!
#define BEEP

#define VIDEORAWLR VIDEOMODE_DISABLED
#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 60 // the estimated frame rate of the video used for creating output videos
#define IMG_W 848
#define IMG_H 480
#define IMSCALEF 2
#define CAMERA_ANGLE 33.0f

#define CAMMODE CAMMODE_REALSENSE

#define DRONE_IM_X_START 424/IMSCALEF
#define DRONE_IM_Y_START 420/IMSCALEF
#define DRONE_DISPARITY_START 19/IMSCALEF
#define DRONE_IM_START_SIZE 4.2f

#define MAX_BORDER_Y_DEFAULT 2.09f  // 2.20 for large scale flight plan  // 1.3 for small scale flight plan
#define MAX_BORDER_Z_DEFAULT 4.0f

#define RC_DEVO 0
#define RC_USB_HOBBYKING 1
#define RC_PLAYSTATION 2

#define JOYSTICK_TYPE RC_USB_HOBBYKING // [0: normal remote] [1: USB remote]

#define TX_DSMX 0
#define TX_CX10 1
#define TX_FRSKYD 2

#define TX_TYPE TX_FRSKYD

#endif //DEFINES_H
