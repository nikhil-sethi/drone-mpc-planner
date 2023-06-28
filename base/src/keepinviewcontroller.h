#pragma once
#include "common.h"
#include "flightarea/flightarea.h"
#include "filtering/filtering.h"
#include "tracking.h"
#include "rapid_route.h"

class KeepInViewController {
private:
    // Parameter:

    // States:
    bool enabled = true;

    // Handles:
    xmls::DroneCalibration *_drone_calib;
    FlightArea *_flight_area;

    // Methods:

public:
    // Parameter:
    float drone_rotating_time;
    const float safety = 1.5f;

    // States:
    bool active = false;  //indicator for the log, whether kiv is active or not
    stopping_position_result current_stopping_position;

    // Methods:
    void init(FlightArea *flight_area, xmls::DroneCalibration *dcalib);
    cv::Point3f correction_acceleration(tracking::TrackData data_drone, float transmission_delay_duration, safety_margin_types safety_margin_type);
    void enable() {enabled = true;};
    void disable() {enabled = false;};
};
