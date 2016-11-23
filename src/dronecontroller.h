#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "rs232.h"
#include "dronetracker.h"
#include "common.h"
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>

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
		int rollI = 0;
		int rollD = 0;

		//pitch control
        int pitchP = 0;
		int pitchI = 0;
		int pitchD = 0;

		template <class Archive>
        void serialize( Archive & ar )
        {
          ar( heightP,heightI,heightD,rollP,rollI,rollD,pitchP,pitchI,pitchD);
        }

    };

    controlParameters params;
    int baudrate;


public:

    void close (void);
    bool init(void);
    void control(trackData data);

};




#endif //DRONECONTROLLER_H
