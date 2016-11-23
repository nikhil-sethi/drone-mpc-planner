#ifndef DEFINES_H
#define DEFINES_H
#ifndef _PC

#define NEON

#define HASSCREEN // dont disable in qt debugger!

//#define VIDEORAW
//#define VIDEODISPARITY // render a video of the disparity map. Currently only works form STEREO_PARROT
//#define VIDEORESULTS
#define VIDEOFPS 5.2f // the estimated frame rate of the video used for creating output videos
#else
#define SSE2

#define HASSCREEN // dont disable in qt debugger!

//#define VIDEORAW
//#define VIDEODISPARITY // render a video of the disparity map. Currently only works form STEREO_PARROT
//#define VIDEORESULTS
#define VIDEOFPS 5.2f // the estimated frame rate of the video used for creating output videos


#endif
#endif //DEFINES_H
