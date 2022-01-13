#pragma once

#include <tuple>
#include "rc.h"

enum trim_states {
    init_trim_roll,
    trim_roll,
    init_trim_pitch,
    trim_pitch
};

enum trim_axes {
    roll_axis,
    pitch_axis
};

class AccelerometerTrim {
public:
    std::tuple<int, int, bool> trim_accelerometer(double time);

    void intergrator_hovering(float integrator_roll, float integrator_pitch) {
        integrator_hovering_roll = integrator_roll;
        integrator_hovering_pitch = integrator_pitch;
    }

private:
    bool finished = true;
    trim_states trim_state = init_trim_roll;
    double time_start_trimming_roll = -1;
    double time_start_trimming_pitch = -1;
    float integrator_hovering_roll = 0;
    float integrator_hovering_pitch = 0;
    int delta_roll_trim_cycles;
    int delta_pitch_trim_cycles;

    const double bf_stick_autorepeat = 0.25; // See betaflight/src/main/fc/rc_controls.c::define STICK_AUTOREPEAT_MS
    const int trim_step = 2;                 // See betaflight/src/main/fc/rc_controls.c::accelerometerTrimsDelta.values.pitch
    const float integrator_trim_scale = 1.f;

    float calibration_error(float integrator, trim_axes trim_axis);
};
