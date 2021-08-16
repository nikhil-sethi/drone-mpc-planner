#pragma once
#include "trackers/trackermanager.h"
#include "dronecontroller.h"
#include "navigation/dronenavigation.h"

#ifdef VIZ_3D
#include "TFMessagePublisher.h"
#include "PathPublisher.h"
#include "cameraview.h"
#endif

class Visualizer3D
{
public:
    Visualizer3D();

#ifdef VIZ_3D
    bool init(tracking::TrackerManager *trackers, CameraView *cam_volume, DroneController *dctrl, navigation::DroneNavigation *dnav);
    void run();
#else
    bool init(tracking::TrackerManager *trackers [[maybe_unused]], FlightArea *flight_area [[maybe_unused]], DroneController *dctrl [[maybe_unused]], navigation::DroneNavigation *dnav [[maybe_unused]]) {return true;}
    void run() {}
#endif

private:
#ifdef VIZ_3D
    void addDrone();
    void addInsect();
    void addTarget();

    TFMessagePublisher pub_tf;
    PathPublisher pub_path;
    tracking::DroneTracker *_dtrkr;
    tracking::InsectTracker *_itrkr;
    int64_t _time;
    DroneController *_dctrl;
    navigation::DroneNavigation *_dnav;
#endif
};
