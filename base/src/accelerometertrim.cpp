#include "accelerometertrim.h"


std::tuple<int, int, bool> AccelerometerTrim::trim_accelerometer(double time) {
    int roll, pitch = RC_MIDDLE;

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
                roll = RC_MIDDLE;
                pitch = RC_MIDDLE;
                finished = false;
                break;
        } case trim_roll: {
                if (abs((time - time_start_trimming_roll) / bf_stick_autorepeat) - 0.5 < abs(delta_roll_trim_cycles)) {
                    if (delta_roll_trim_cycles < 0)
                        roll = RC_BOUND_MAX;
                    else
                        roll = RC_BOUND_MIN;
                    pitch = RC_MIDDLE;
                    finished = false;
                } else {
                    roll = RC_MIDDLE;
                    pitch = RC_MIDDLE;
                    finished = false;
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
                    finished = false;
                } else {
                    trim_state = init_trim_roll;
                    finished = true;
                }
                roll = RC_MIDDLE;
                pitch = RC_MIDDLE;
                break;
        } case trim_pitch: {
                if (abs((time - time_start_trimming_pitch) / bf_stick_autorepeat) - 0.5 < abs(delta_pitch_trim_cycles)) {
                    if (delta_pitch_trim_cycles < 0)
                        pitch = RC_BOUND_MAX;
                    else
                        pitch = RC_BOUND_MIN;
                    roll = RC_MIDDLE;
                    finished = false;
                } else {
                    roll = RC_MIDDLE;
                    pitch = RC_MIDDLE;
                    finished = true;
                    integrator_hovering_pitch = 0;
                    trim_state = init_trim_roll;
                    time_start_trimming_pitch = -1;
                }
                break;
            }
    }
    return std::tuple(roll, pitch, finished);
}

float AccelerometerTrim::calibration_error(float integrator, trim_axes trim_axis) {
    if (trim_axis == roll_axis)
        return atan2f(dparams.ki_pos_roll_hover * integrator, 9.81) * rad2deg * 10;
    else
        return atan2f(dparams.ki_pos_pitch_hover * integrator, 9.81) * rad2deg * 10;
}
