#pragma once

#include <tuple>
#include "rc.h"

enum accelerometer_states {
    trim,
    reset,
};

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

struct accelerometer_trim_data {
    int roll = BF_CHN_MID;
    int pitch = BF_CHN_MID;
    int throttle = BF_CHN_MIN;
    int yaw = BF_CHN_MID;
    bool arm = true;
    bool finished = false;
};


class AccelerometerTrim {
public:
    accelerometer_trim_data trim_accelerometer(double time);

    void intergrator_hovering(float integrator_roll, float integrator_pitch) {
        integrator_hovering_roll = integrator_roll;
        integrator_hovering_pitch = integrator_pitch;
    }

    bool ready() {return integrator_hovering_roll || integrator_hovering_pitch;};

private:
    accelerometer_states accel_state = trim;
    trim_states trim_state = init_trim_roll;
    int reset_state = 0;
    double time_start_trimming_roll = -1;
    double time_start_trimming_pitch = -1;
    double time_reset_accelerometer_trim = -1;
    float integrator_hovering_roll = 0;
    float integrator_hovering_pitch = 0;
    int delta_roll_trim_cycles;
    int delta_pitch_trim_cycles;

    const double bf_stick_autorepeat = 0.25; // See betaflight/src/main/fc/rc_controls.c::define STICK_AUTOREPEAT_MS
    const int trim_step = 2;                 // See betaflight/src/main/fc/rc_controls.c::accelerometerTrimsDelta.values.pitch
    const float integrator_trim_scale = 1.f;

    accelerometer_trim_data trim_commands(double time);
    accelerometer_trim_data reset_commands(double time);
    void trigger_reset_state(double time) {
        accel_state = reset;
        time_reset_accelerometer_trim = time;
    }
    float calibration_error(float integrator, trim_axes trim_axis);
};
