
#ifndef FILECAM_H
#define FILECAM_H
#include "defines.h"
#include <mutex>
#include <thread>
#include <opencv2/highgui/highgui.hpp>


#include "cam.h"


/*
 * This class will open a (stereo) video file and stream the data as if it was an real camera
 *
 */
class FileCam : public Cam{


private:	
    std::string file;

    void workerThread(void);

    cv::VideoCapture video;
    stopwatch_c stopWatch;

    int skipstart;
    int videoLength;

public:

    int fastforward;
    int rewind;



    void close (void);

    bool init(void);

    bool initFile (std::string file, int nskip);

};
#endif //FILECAM_H
