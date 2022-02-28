#pragma once
#include <FastLED.h>

#define NUM_RGB_LEDS 2

class RGBLeds {
public:
    enum rgb_led_0_states {
        LED0_init = 0,
        LED0_disabled,
        LED0_charging,
        LED0_not_charging,
        LED0_calibrating,
        LED0_unkown
    };

    enum rgb_led_1_states {
        LED1_init = 0,
        LED1_inresponsive_NUC,
        LED1_executor_problem, // (angle, realsense, etc)
        LED1_realsense_reset,
        LED1_executor_start, // start / restart / closing
        LED1_wait_for_plukkers,
        LED1_wait_for_darkness,
        LED1_c_OK,
        LED1_x_OK,
        LED1_unkown
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

private:
    rgb_led_0_states _led0_state = LED0_init;
    rgb_led_1_states _led1_state = LED1_init;
    bool _internet_OK = true;
    bool _daemon_OK = true;
    bool _post_processing = false;
    bool _watchdog_trigger_imminent = false;

    bool update_state[NUM_RGB_LEDS] = {true};
    CRGB rgb_leds[NUM_RGB_LEDS];
    CRGB rgb_setpoint_leds[NUM_RGB_LEDS];
    int period_id_leds[NUM_RGB_LEDS];
    blink_modes blink_leds[NUM_RGB_LEDS];

    bool update_FastLED = false;
    unsigned long t_switch = 0;

    void blink(int[], int, unsigned long, int);

};