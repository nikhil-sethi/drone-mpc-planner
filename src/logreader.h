#ifndef LOGREADER_H
#define LOGREADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <unistd.h>


#include <opencv2/core/core.hpp>


/*
 * This class performs moving average filtering
 *
 */
class LogReader{

public:
    struct Log_Entry {
      int ID;
      int RS_ID;
      int imLx;
      int imLy;
      int disparity;
      bool valid;
      float posErrX;
      float posErrY;
      float posErrZ;
      float velX;
      float velY;
      float velZ;
      int hoverthrottle;
      int autoThrottle;
      int autoRoll;
      int autoPitch;
      int autoYaw;
      int joyThrottle;
      int joyRoll;
      int joyPitch;
      int joyYaw;
      int joySwitch;
      int throttleP;
      int throttleI;
      int throttleD;
      float dt;
      float dx;
      float dy;
      float dz;
    };
    void init(std::string file);
    Log_Entry getItem(int frame_number);
private:

    Log_Entry createLogEntry(std::string line);
    void setHeadMap(std::string heads);
//    std::map<std::string, int> headmap;
    std::map<int, Log_Entry> log;
    std::map<std::string, int> headmap;
    std::map<std::string, int> headmap_fix;


};

#endif // LOGREADER_H
