#include "accelerometertrim.h"


accelerometer_trim_data AccelerometerTrim::trim_accelerometer(double time) {
    switch (accel_state) {
        case trim: {
                return trim_commands(time);
                break;
        } case reset: {
                return reset_commands(time);
                break;
            }
    }
    return accelerometer_trim_data();
}

accelerometer_trim_data AccelerometerTrim::trim_commands(double time) {
    accelerometer_trim_data ret;

    switch (trim_state) {
        case init_trim_roll: {
                delta_roll_trim_cycles = static_cast<int>(calibration_error(integrator_hovering_roll, roll_axis) / trim_step);
                std::cout << "Change roll trim by " << delta_roll_trim_cycles *trim_step << std::endl;
                if (abs(delta_roll_trim_cycles) > 0) {
                    trim_state = trim_roll;
                    time_start_trimming_roll = time;
                } else {
                    trim_state = init_trim_pitch;
                }
                ret.roll = RC_MIDDLE;
                ret.pitch = RC_MIDDLE;
                break;
        } case trim_roll: {
                if (abs((time - time_start_trimming_roll) / bf_stick_autorepeat) - 0.5 < abs(delta_roll_trim_cycles)) {
                    if (delta_roll_trim_cycles < 0)
                        ret.roll = RC_BOUND_MAX;
                    else
                        ret.roll = RC_BOUND_MIN;
                    ret.pitch = RC_MIDDLE;
                } else {
                    ret.roll = RC_MIDDLE;
                    ret.pitch = RC_MIDDLE;
                    integrator_hovering_roll = 0;
                    trim_state = init_trim_pitch;
                    time_start_trimming_roll = -1;
                }
                break;
            }
        case init_trim_pitch: {
                delta_pitch_trim_cycles = static_cast<int>(calibration_error(integrator_hovering_pitch, pitch_axis));
                std::cout << "Change pitch trim by " << delta_pitch_trim_cycles *trim_step << std::endl;
                if (abs(delta_pitch_trim_cycles) > 0) {
                    trim_state = trim_pitch;
                    time_start_trimming_pitch = time;
                } else {
                    trim_state = init_trim_roll;
                    // trigger_reset_state(time);
                    ret.finished = true;
                }
                ret.roll = RC_MIDDLE;
                ret.pitch = RC_MIDDLE;
                break;
        } case trim_pitch: {
                if (abs((time - time_start_trimming_pitch) / bf_stick_autorepeat) - 0.5 < abs(delta_pitch_trim_cycles)) {
                    if (delta_pitch_trim_cycles < 0)
                        ret.pitch = RC_BOUND_MAX;
                    else
                        ret.pitch = RC_BOUND_MIN;
                    ret.roll = RC_MIDDLE;
                } else {
                    ret.roll = RC_MIDDLE;
                    ret.pitch = RC_MIDDLE;
                    // trigger_reset_state(time);
                    ret.finished = true;
                    integrator_hovering_pitch = 0;
                    trim_state = init_trim_roll;
                    time_start_trimming_pitch = -1;
                }
                break;
            }
    }
    return ret;
}

accelerometer_trim_data AccelerometerTrim::reset_commands(double time) {
    accelerometer_trim_data ret;
    ret.throttle = RC_BOUND_MIN;
    ret.roll = RC_MIDDLE;
    ret.pitch = RC_MIDDLE;
    ret.yaw = RC_MIDDLE;

    const double arm_duration = 1.1;
    switch (reset_state) {
        case 0: {
                ret.arm = true;
                if ((time - time_reset_accelerometer_trim) > arm_duration) {
                    time_reset_accelerometer_trim = time;
                    reset_state++;
                }
                break;
        } case 1: {
                ret.arm = false;
                if (time - time_reset_accelerometer_trim > arm_duration) {
                    ret.finished = true;
                    reset_state = 0;
                    accel_state = trim;
                }
                break;
            }
    }

    return ret;
}


float AccelerometerTrim::calibration_error(float integrator, trim_axes trim_axis) {
    if (trim_axis == roll_axis)
        return atan2f(dparams.ki_pos_roll_hover * integrator, 9.81) * rad2deg * 10;
    else
        return atan2f(dparams.ki_pos_pitch_hover * integrator, 9.81) * rad2deg * 10;
}

