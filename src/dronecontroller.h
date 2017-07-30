#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "rs232.h"
#include "dronetracker.h"
#include "joystick.hpp"
#include "common.h"
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <iomanip>
#include <unistd.h>

using namespace cv;

/*
 * This class will control a micro drone via a Serial link
 *
 */
class DroneController {


private:
    
    struct controlParameters{

		//height control
        int heightP = 0;
		int heightI = 0;
		int heightD = 0;

		//roll control
        int rollP = 0;
		int rollI = 128;
		int rollD = 0;

		//pitch control
        int pitchP = 0;
		int pitchI = 128;
		int pitchD = 0;

		//yaw control
        int yawP = 0;
		int yawI = 128;
		int yawD = 0;

		template <class Archive>
        void serialize( Archive & ar )
        {
          ar( heightP,heightI,heightD,rollP,rollI,rollD,pitchP,pitchI,pitchD);
        }

    };

    controlParameters params;
    int baudrate;
    std::ofstream *_logger;


public:
int commandedRoll=6;
int commandedPitch=6;
int commandedYaw=6;
int commandedThrottle=6;

    void close (void);
    bool init(std::ofstream *logger);
    void control(trackData data);

};




#endif //DRONECONTROLLER_H
