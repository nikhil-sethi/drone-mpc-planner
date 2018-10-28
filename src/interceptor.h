#ifndef INTERCEPT_H
#define INTERCEPT_H

#include "defines.h"
#include "insecttracker.h"
#include "dronetracker.h"



/*
 * This class calculates the best intersection location, and whether that is even possible, etc
 *
 */
class Interceptor{

private:
    DroneTracker * _dtrk;
    InsectTracker * _itrkr;

public:

    void init(DroneTracker * dtrk, InsectTracker * itrkr);
    void update();
    bool get_insect_in_range();

};

#endif
