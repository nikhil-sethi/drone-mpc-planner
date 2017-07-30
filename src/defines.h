#ifndef DEFINES_H
#define DEFINES_H

#define VIDEOMODE_DISABLED 0
#define VIDEOMODE_AVI 1
#define VIDEOMODE_STREAM 2 
#define VIDEOMODE_AVI_OPENCV 3

#define IMSCALEF 2
#ifndef _PC

#define NEON
//#define HASSCREEN

// due to hardware accelerated encoding, only one gstream can be enabled at the same time!
#define VIDEORAWLR VIDEOMODE_AVI
#define VIDEORESULTS VIDEOMODE_DISABLED

//non-hw accelerated custom 16b grayscale 96x96 video render:
#define VIDEODISPARITY VIDEOMODE_AVI

#define VIDEOFPS 14 // the estimated frame rate of the video used for creating output videos

#else

#define SSE2

#define HASSCREEN // dont disable in qt debugger!

#define VIDEORAWLR VIDEOMODE_DISABLED
#define VIDEODISPARITY VIDEOMODE_DISABLED
#define VIDEORESULTS VIDEOMODE_DISABLED
#define VIDEOFPS 12 // the estimated frame rate of the video used for creating output videos


#endif
#endif //DEFINES_H
