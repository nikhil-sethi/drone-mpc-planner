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

#define RC_NONE 0
#define RC_DEVO 1
#define RC_USB_HOBBYKING 2
#define RC_PLAYSTATION 3
#define RC_XLITE 4

#define TX_NONE 0
#define TX_DSMX 1
#define TX_CX10 2
#define TX_FRSKYD 3
#define TX_FRSKYX 4
#define TX_FRSKYX_TC 5

#define INSECT_LOGGING_MODE

#ifdef INSECT_LOGGING_MODE
    #define VIDEORAWLR VIDEOMODE_AVI
    #define JOYSTICK_TYPE RC_NONE
    #define TX_TYPE TX_NONE
    #define WATCHDOG_ENABLED
//    #define HASSCREEN
#else
    #ifndef HASGUI
        #define HASSCREEN
    #endif
//    #define BEEP // leave of, causes realsense resets! Prolly needs moving to main UI thread...
    #define VIDEORAWLR VIDEOMODE_BAG
    #define JOYSTICK_TYPE RC_XLITE
    #define TX_TYPE TX_FRSKYX_TC
#endif

#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 60 // auto exposure may change this. This is handled properly for 60 by limiting the exposure. But not for the the other possible framerates
#define IMG_W 848
#define IMG_H 480
#define IMSCALEF 2

#define CAMMODE CAMMODE_REALSENSE

#define DRONE_IM_START_SIZE 12.f
//#define MANUAL_DRONE_LOCATE

#endif //DEFINES_H
