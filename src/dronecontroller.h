#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "rs232.h"

/*
 * This class will control a micro drone via a Serial link
 *
 */
class DroneController {


private:
    
    struct PIDparameters{
    };

    PIDparameters params;
    int baudrate;


public:

    void close (void);
    bool init(void);
    void control(float pos_X, float pos_Y, float pos_Z);

};




#endif //DRONECONTROLLER_H
