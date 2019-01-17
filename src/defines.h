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
#define VIDEOMODE_BAG 4

#define RC_DISABLED 0
#define RC_DEVO 1
#define RC_USB_HOBBYKING 2
#define RC_PLAYSTATION 3
#define RC_XLITE 4

#define TX_DISABLED 0
#define TX_DSMX 1
#define TX_CX10 2
#define TX_FRSKYD 3

//#define INSECT_LOGGING_MODE

#ifdef INSECT_LOGGING_MODE
    #define VIDEORAWLR VIDEOMODE_DISABLED
    #define JOYSTICK_TYPE RC_DISABLED
    #define TX_TYPE TX_DISABLED
//    #define HASSCREEN
#else
    #ifndef HASGUI
        #define HASSCREEN
    #endif
    #define BEEP
    #define VIDEORAWLR VIDEOMODE_BAG
    #define JOYSTICK_TYPE RC_USB_HOBBYKING
    #define TX_TYPE TX_FRSKYD
#endif

#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 60 // auto exposure may change this. This is handled properly for 60 by limiting the exposure. But not for the the other possible framerates
#define IMG_W 848
#define IMG_H 480
#define IMSCALEF 2

#define CAMMODE CAMMODE_REALSENSE

#define DRONE_IM_X_START 110
#define DRONE_IM_Y_START 420/IMSCALEF
#define DRONE_DISPARITY_START 19/IMSCALEF
#define DRONE_IM_START_SIZE 4.2f

#define MAX_BORDER_Y_DEFAULT 1.85f  // 2.20 for large scale flight plan  // 1.3 for small scale flight plan
#define MAX_BORDER_Z_DEFAULT 4.0f

#endif //DEFINES_H
