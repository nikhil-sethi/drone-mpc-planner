#ifndef VISUALIZER_3D_H
#define VISUALIZER_3D_H

#include "trackermanager.h"
#include "dronecontroller.h"
#include "dronenavigation.h"

#ifdef VIZ_3D
#include "TFMessagePublisher.h"
#include "PathPublisher.h"
#endif

class Visualizer3D
{
public:
    Visualizer3D();

#ifdef VIZ_3D
    bool init(TrackerManager *trackers, CameraVolume *cam_volume, DroneController *dctrl, DroneNavigation *dnav);
    void run();
#else
    bool init(TrackerManager *trackers, CameraVolume *cam_volume, DroneController *dctrl, DroneNavigation *dnav){return true;}
    void run(){}
#endif

private:
#ifdef VIZ_3D
    void addDrone();
    void addInsect();
    void addTarget();

    TFMessagePublisher pub_tf;
    PathPublisher pub_path;
    DroneTracker *_dtrkr;
    InsectTracker *_itrkr;
    int64_t _time;
    DroneController *_dctrl;
    DroneNavigation *_dnav;
#endif
};

#endif // VISUALIZER_3D_H
