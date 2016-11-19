#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "rs232.h"
#include "dronetracker.h"

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
    void control(trackData data);

};




#endif //DRONECONTROLLER_H
