#ifndef LOGREADER_H
#define LOGREADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <unistd.h>
#include <map>

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
      bool valid;
      int joyThrottle;
      int joyRoll;
      int joyPitch;
      int joyYaw;
      int joySwitch;
    };
    void init(std::string file);
    Log_Entry getItem(int frame_number);
private:

    Log_Entry createLogEntry(std::string line);
    void setHeadMap(std::string heads);
//    std::map<std::string, int> headmap;
    std::map<int, Log_Entry> log;
    std::map<std::string, int> headmap;

};

#endif // LOGREADER_H
