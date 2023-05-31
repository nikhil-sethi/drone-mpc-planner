#pragma once
#include <FastLED.h>
#include "utility.h"

#define NUM_RGB_LEDS 2

class RGBLeds {
public:
    enum rgb_led_0_states {
        LED0_init = 0,
        LED0_disabled,
        LED0_charging,
        LED0_trickle_charging,
        LED0_not_charging,
        LED0_discharging,
        LED0_battery_problem,
        LED0_telemetry_problem_with_charging,
        LED0_telemetry_problem_no_charging,
        LED0_locate_fail,
        LED0_crashed,
        LED0_unknown,
        LED0_x_ready
    };

    enum rgb_led_1_states {
        LED1_init = 0,
        LED1_inresponsive_NUC,
        LED1_executor_problem, // (angle, realsense, etc)
        LED1_realsense_reset,
        LED1_executor_start, // start / restart / closing
        LED1_wait_for_enable_window,
        LED1_wait_for_lightlevel,
        LED1_c_OK,
        LED1_x_OK,
        LED1_blind_OK,
        LED1_unknown
    };

    enum blink_modes {
        blink_solid = 0,
        blink_200ms,
        blink_500ms,
        blink_1000ms,
        blink_1_short_blink,
        blink_1_short_blink_reversed,
        blink_2_short_blinks,
        blink_3_short_blinks
    };

    void init();
    void run();
    void nuc_inresponsive() {
        led1_state(RGBLeds::LED1_inresponsive_NUC);
        internet_OK(false);
        post_processing(false);
        daemon_OK(false);
    }
    void led0_state(rgb_led_0_states s) {
        if (_drone_issue == drone_issues_telemetry_problem) {
            if ((s == LED0_trickle_charging || s == LED0_discharging || s == LED0_charging))
                s = LED0_telemetry_problem_with_charging;
            else
                s = LED0_telemetry_problem_no_charging;
        }

        if (s == _led0_state)
            return;

        _led0_state = s;
        update_state[0] = true;
    }

    void led1_state(rgb_led_1_states s) {
        if (s == _led1_state)
            return;
        _led1_state = s;
        update_state[1] = true;
    }
    void internet_OK(bool b) {
        if (b == _internet_OK)
            return;
        _internet_OK = b;
        update_state[1] = true;
    }
    void daemon_OK(bool b) {
        if (b == _daemon_OK)
            return;
        _daemon_OK = b;
        update_state[1] = true;
    }
    void post_processing(bool b) {
        if (b == _post_processing)
            return;
        _post_processing = b;
        update_state[1] = true;
    }
    void watchdog_trigger_imminent(bool b) {
        if (b == _watchdog_trigger_imminent)
            return;
        _watchdog_trigger_imminent = b;
        update_state[1] = true;
    }
    void drone_issue(drone_issues v) {
        if (v == _drone_issue && v != drone_issues_telemetry_problem)
            return;

        switch (v) {
            case drone_issues_drone_ok:
                _led0_state = LED0_init;
                update_state[0] = true;
                break;
            case drone_issues_no_drone:
                _led0_state = LED0_disabled;
                update_state[0] = true;
                break;
            case drone_issues_telemetry_problem:
                if (_led0_state != LED0_telemetry_problem_no_charging && _led0_state != LED0_telemetry_problem_with_charging) {
                    if (_led0_state == LED0_charging || _led0_state == LED0_trickle_charging || _led0_state == LED0_discharging)
                        _led0_state = LED0_telemetry_problem_with_charging;
                    else
                        _led0_state = LED0_telemetry_problem_no_charging;
                    update_state[0] = true;
                }
                break;
            case drone_issues_locate_fail:
                _led0_state = LED0_locate_fail;
                update_state[0] = true;
                break;
            case drone_issues_crashed:
                _led0_state = LED0_crashed;
                update_state[0] = true;
                break;
            case drone_issues_ready:
                _led0_state = LED0_x_ready;
                update_state[0] = true;
                break;
            default:
                break;
        }
        _drone_issue = v;
    }

    void light_level(float light_level) {
        light_level_ = constrain(light_level * 2.f, 0.02f, 1.f);
    }

private:
    rgb_led_0_states _led0_state = LED0_init;
    rgb_led_1_states _led1_state = LED1_init;
    bool _internet_OK = true;
    bool _daemon_OK = true;
    bool _post_processing = false;
    bool _watchdog_trigger_imminent = false;
    drone_issues _drone_issue;
    float light_level_ = 1;

    bool update_state[NUM_RGB_LEDS] = {true};
    CRGB rgb_leds[NUM_RGB_LEDS];
    CRGB rgb_setpoint_leds[NUM_RGB_LEDS];
    int period_id_leds[NUM_RGB_LEDS];
    blink_modes blink_leds[NUM_RGB_LEDS];

    bool update_FastLED = false;
    unsigned long t_switch = 0;


    void blink(int[], int, unsigned long, int);
    void blink(int led_id);

public:
    rgb_led_0_states led0_state() {return _led0_state;};
    rgb_led_1_states led1_state() {return _led1_state;};
};