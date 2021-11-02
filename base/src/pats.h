#pragma once

#include "drone.h"
#include "baseboard.h"
#include "visiondata.h"
#include "trackermanager.h"


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
    Baseboard *_baseboard;
    RC *_rc;
    std::ofstream *_logger;

    double time_first_frame = -1;
    double time_start_motion_calibration = 0;
    const float duration_motion_calibration = 2;

    void maintain_motion_map(double time);
public:

    tracking::TrackerManager trackers;
    Drone drone;
    Interceptor interceptor;
    FlightArea flight_area;
    RC *rc() { return _rc;}
    Baseboard *baseboard() { return _baseboard;}

    void init(std::ofstream *logger, int rc_id, RC *rc, std::string replay_dir, Cam *cam, VisionData *visdat,  Baseboard *baseboard);
    void update(double time);
    void close();

    std::string state_str() {return patser_states_names[_pats_state];}
};
