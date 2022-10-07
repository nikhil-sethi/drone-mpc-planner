#pragma once

#include "drone.h"
#include "baseboardlink.h"
#include "visiondata.h"
#include "trackermanager.h"

#ifdef OPTI_ROSVIS
#include "rosvisualizerinterface.h"
#endif

static const char *patser_states_names[] = {
    "Pats_init",
    "pats_calibrating_motion",
    "pats_c",
    "pats_x"
};

enum patser_states {
    pats_init = 0,
    pats_calibrating_motion,
    pats_c,
    pats_x
};

class Patser {
private:

    patser_states _pats_state = pats_init;
    VisionData *_visdat;
    BaseboardLink *_baseboard_link;
    RC *_rc;
    std::ofstream *_logger;

    double time_first_frame = -1;
    double time_start_motion_calibration = 0;
    const float duration_motion_calibration = 2;

    void maintain_motion_map(double time);
public:
#ifdef OPTI_ROSVIS
    void ros_interface(RosVisualizerInterface *interface) {interceptor.ros_interface(interface);};
#endif

    tracking::TrackerManager trackers;
    Drone drone;
    Interceptor interceptor;
    FlightArea flight_area;
    RC *rc() { return _rc;}
    BaseboardLink *baseboard() { return _baseboard_link;}

    void init(std::ofstream *logger, int rc_id, RC *rc, std::string replay_dir, Cam *cam, VisionData *visdat,  BaseboardLink *baseboard);
    void init_insect_replay();
    void init_flight_replay(std::string replay_dir, int flight_id);
    void update(double time);
    void close();

    std::string state_str() {return patser_states_names[_pats_state];}
};
