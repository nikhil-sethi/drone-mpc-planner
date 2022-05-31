#pragma once
#include "trackers/trackermanager.h"
#include "pats.h"

#ifdef VIZ_3D
#include "TFMessagePublisher.h"
#include "PathPublisher.h"
// #include "cameraview.h"
#endif

class Visualizer3D
{
public:
    Visualizer3D();

#ifdef VIZ_3D
    bool init(Patser *patser);
    void run();
#else
    bool init(Patser *patser [[maybe_unused]]) {return true;}
    void run() {}
#endif

private:
#ifdef VIZ_3D
    void addDrone();
    void addInsect();
    void updateFlightarea();
    void addTarget();
    void addOptmizedTrajectory();

    TFMessagePublisher pub_tf;
    PathPublisher pub_path;
    tracking::DroneTracker *_dtrkr;
    tracking::InsectTracker *_itrkr;
    int64_t _time;
    DroneController *_dctrl;
    FlightArea *_flight_area;
    Interceptor *_interceptor;
    navigation::DroneNavigation *_dnav;
#endif
};
