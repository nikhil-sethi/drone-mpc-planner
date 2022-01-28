#include "rgbleds.h"
#include "defines.h"


void RGBLeds::init() {
    FastLED.addLeds<WS2812, RGB_LEDS_PIN, GRB>(rgb_leds, NUM_RGB_LEDS);
    led1_state(LED1_init);
    rgb_setpoint_leds[0] = CRGB(25, 25, 25);
    rgb_setpoint_leds[1] = CRGB(0, 25, 0);
    rgb_leds[0] = rgb_setpoint_leds[0];
    rgb_leds[1] = rgb_setpoint_leds[1];
    blink_leds[0] = blink_solid;
    blink_leds[1] = blink_solid;
    period_id_leds[0] = true;
    period_id_leds[1] = true;
    FastLED.show();
}

void RGBLeds::blink(int periods[], int n_period, unsigned long millis, int led_id) {
    period_id_leds[led_id] %= n_period;
    if (millis - t_switch > periods[period_id_leds[led_id]]) {
        period_id_leds[led_id]++;
        t_switch = millis;
        update_FastLED = true;
        if (period_id_leds[led_id] & 1)
            rgb_leds[led_id] = {0};
        else {
            rgb_leds[led_id][0] = rgb_setpoint_leds[led_id][0];
            rgb_leds[led_id][1] = rgb_setpoint_leds[led_id][1];
            rgb_leds[led_id][2] = rgb_setpoint_leds[led_id][2];
        }
    }
}

void RGBLeds::run() {

    if (update_state) {
        blink_leds[1] = blink_solid;
        update_FastLED = true;
        switch (_led1_state) {
            case LED1_init: { // solid red
                    rgb_setpoint_leds[1] = CRGB(50, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_inresponsive_NUC: { // red blink 0.5s
                    rgb_setpoint_leds[1] = CRGB(200, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    blink_leds[1] = blink_200ms;
                    break;
            } case LED1_internal_system_error: { // red blink 1s
                    rgb_setpoint_leds[1] = CRGB(200, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    blink_leds[1] = blink_1_short_blink;
                    break;
            } case LED1_realsense_reset: { // solid pink
                    rgb_setpoint_leds[1] = CRGB(255, 5, 80);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_executor_start: { // solid White
                    rgb_setpoint_leds[1] = CRGB(50, 50, 50);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_wait_for_plukkers: { // solid blue
                    rgb_setpoint_leds[1] = CRGB(0, 0, 100);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_wait_for_darkness: { // solid yellow
                    rgb_setpoint_leds[1] = CRGB(200, 200, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_c_OK: { // solid green
                    rgb_setpoint_leds[1] = CRGB(0, 25, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_x_OK: { // solid cyan
                    rgb_setpoint_leds[1] = CRGB(0, 25, 25);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            }  default: { // off
                    rgb_setpoint_leds[1] = CRGB(0, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
                }
        }
        update_state = false;
    }

    //some states that can exist in parallel with the main state:
    if (_led1_state != LED1_inresponsive_NUC) {
        if (!_internet_OK)
            blink_leds[1] = blink_3_short_blinks;
        else if (!_daemon_OK)
            blink_leds[1] = blink_2_short_blinks;
        else if (_post_processing)
            blink_leds[1] = blink_1_short_blink_reversed;
    }

    switch (blink_leds[1])
    {
        case blink_solid:
            break;
        case blink_1000ms: {
                int tmp[] = {1000, 1000};
                blink(tmp, 2, millis(), 1);
                break;
        } case blink_500ms: {
                int tmp[] = {500, 500};
                blink(tmp, 2, millis(), 1);
                break;
        } case blink_200ms: {
                int tmp[] = {200, 200};
                blink(tmp, 2, millis(), 1);
                break;
        } case blink_1_short_blink: {
                int tmp[] = {100, 1000};
                blink(tmp, 2, millis(), 1);
                break;
        } case blink_1_short_blink_reversed: {
                int tmp[] = {1000, 100};
                blink(tmp, 2, millis(), 1);
                break;
        } case blink_2_short_blinks: {
                int tmp[] = {150, 150, 150, 1000};
                blink(tmp, 4, millis(), 1);
                break;
        } case blink_3_short_blinks: {
                int tmp[] = {150, 150, 150, 150, 150, 1000};
                blink(tmp, 6, millis(), 1);
                break;
        } default:
            break;
    }

    if (update_FastLED) {
        FastLED.show();
        update_FastLED = false;
    }
}
