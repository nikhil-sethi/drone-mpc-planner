#pragma once
#include "common.h"
#include "flightarea/flightarea.h"
#include "filtering/filtering.h"
#include "tracking.h"
#include "trajectory_optimization.h"

class KeepInViewController {
private:
    // Parameter:

    // States:
    bool enabled = true;

    // Handles:
    xmls::DroneCalibration *_drone_calib;

    // Methods:

public:
    // Parameter:
    float drone_rotating_time;
    const float safety = 1.5f;

    // States:
    bool active = false;  //indicator for the log, whether kiv is active or not
    stopping_position_result current_stopping_position;

    safety_margin_types _safety_margin_type;
    FlightArea *_flight_area;
    FlightAreaConfig *_flight_area_config;


    // Methods:
    void init(FlightArea *flight_area, xmls::DroneCalibration *dcalib, safety_margin_types safety_margin_type);
    cv::Point3f correction_acceleration(tracking::TrackData data_drone, float transmission_delay_duration, float thrust_factor);
    void enable() {enabled = true;};
    void disable() {enabled = false;};
};
