#ifndef KALAMOSFILECAM_H
#define KALAMOSFILECAM_H
#include "defines.h"
#include "cam.h"

#include <iostream>

class KalamosFileCam : public Cam{

private:
    std::string folder;
    cv::VideoCapture videoLR;

public:

    bool init (std::string folder);
    void close (void);

//    std::vector<int> thrusts;
//    std::vector<int> rolls;
//    std::vector<int> pitchs;
//    std::vector<int> yaws;

//    float getCurrentThrust(void){return thrusts.at(currentFrame-1);}
//    float getCurrentRoll(void){return rolls.at(currentFrame-1);}
//    float getCurrentPitch(void){return pitchs.at(currentFrame-1);}
//    float getCurrentYaw(void){return yaws.at(currentFrame-1);}

    void workerThread(void);
    cv::Mat get_raw_frame(void);    
    cv::Mat get_combined(void);        

    KalamosFileCam() {
        isStereo = true;
    }

};
#endif // KALAMOSFILECAM_H
