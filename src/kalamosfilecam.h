#ifndef KALAMOSFILECAM_H
#define KALAMOSFILECAM_H
#include "defines.h"
#include "cam.h"

#include <iostream>

class KalamosFileCam : public Cam{

private:
    std::string folder;
    cv::VideoCapture videoL;
    cv::VideoCapture videoR;

public:

    bool init (std::string folder);
    void close (void);

    void workerThread(void);
    cv::Mat get_raw_frame(void);    
    cv::Mat get_combined(void);        

    KalamosFileCam() {
        isStereo = true;
    }

};
#endif // KALAMOSFILECAM_H
