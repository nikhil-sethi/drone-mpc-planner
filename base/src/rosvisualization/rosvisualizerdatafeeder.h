#pragma once
#include "rosvisualizerinterface.h"
#include "dronecontroller.h"
#include "dronenavigation.h"
#include "flightarea.h"
#include "pats.h"

class RosVisualizerDataCollector {
public:
    void init(Patser *patser);

private:
    RosVisualizerInterface ros_interface;
    tracking::DroneTracker *_dtrkr;
    tracking::InsectTracker *_itrkr;
    int64_t _time;
    DroneController *_dctrl;
    FlightArea *_flight_area;
    Interceptor *_interceptor;
    navigation::DroneNavigation *_dnav;
};
