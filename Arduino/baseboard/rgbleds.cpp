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

void RGBLeds::blink(int led_id) {

    switch (blink_leds[led_id])
    {
        case blink_solid:
            break;
        case blink_1000ms: {
                int periods[] = {1000, 1000};
                blink(periods, 2, millis(), led_id);
                break;
        } case blink_500ms: {
                int periods[] = {500, 500};
                blink(periods, 2, millis(), led_id);
                break;
        } case blink_200ms: {
                int periods[] = {200, 200};
                blink(periods, 2, millis(), led_id);
                break;
        } case blink_1_short_blink: {
                int periods[] = {100, 1000};
                blink(periods, 2, millis(), led_id);
                break;
        } case blink_1_short_blink_reversed: {
                int periods[] = {1000, 100};
                blink(periods, 2, millis(), led_id);
                break;
        } case blink_2_short_blinks: {
                int periods[] = {150, 150, 150, 1000};
                blink(periods, 4, millis(), led_id);
                break;
        } case blink_3_short_blinks: {
                int periods[] = {150, 150, 150, 150, 150, 1000};
                blink(periods, 6, millis(), led_id);
                break;
        } default:
            break;
    }

}

void RGBLeds::run() {

    if (update_state[0]) {
        blink_leds[0] = blink_solid;
        update_FastLED = true;
        switch (_led0_state) {
            case LED0_init: { // solid white
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 255 * light_level_, 255 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
            } case LED0_unknown: { //
            } case LED0_disabled: { // off
                    rgb_setpoint_leds[0] = CRGB(0, 0, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
            } case LED0_battery_problem: { // yellow 3 short blinks
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 255 * light_level_, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_3_short_blinks;
                    break;
            } case LED0_not_charging: { // solid red
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 0, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
            } case LED0_charging: { // solid blue
                    rgb_setpoint_leds[0] = CRGB(0, 0, 255 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
            } case LED0_trickle_charging: { // solid cyan
                    rgb_setpoint_leds[0] = CRGB(0, 255 * light_level_, 255 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
            } case LED0_discharging: { // blue 3 short blinks
                    rgb_setpoint_leds[0] = CRGB(0, 0, 255 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_3_short_blinks;
                    break;
            } case LED0_telemetry_problem_with_charging: { // pink 1s symmetric blink
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 0, 128 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_500ms;
                    break;
            } case LED0_telemetry_problem_no_charging: { // red 1s symmetric blink
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 0, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_500ms;
                    break;
            } case LED0_locate_fail: { // orange 1 short blink every second
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 128 * light_level_, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_1_short_blink;
                    break;
            } case LED0_crashed: { // white 1s symmetric blink
                    rgb_setpoint_leds[0] = CRGB(255 * light_level_, 255 * light_level_, 255 * light_level_);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    blink_leds[0] = blink_500ms;
                    break;
            } case LED0_x_ready: { // solid green
                    rgb_setpoint_leds[0] =  CRGB(0, 255 * light_level_, 0);
                    rgb_leds[0] = rgb_setpoint_leds[0];
                    break;
                }
        }
        update_state[0] = false;
    }

    if (update_state[1]) {
        blink_leds[1] = blink_solid;
        update_FastLED = true;
        if (_watchdog_trigger_imminent)
            _led1_state = LED1_inresponsive_NUC;

        switch (_led1_state) {
            case LED1_init: { // solid red
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_inresponsive_NUC: { // 1s symmetric red blink
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    blink_leds[1] = blink_500ms;
                    break;
            } case LED1_executor_problem: { // red 1 short blink every second
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    blink_leds[1] = blink_1_short_blink;
                    break;
            } case LED1_realsense_reset: { // solid pink
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 5 * light_level_, 80 * light_level_);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_executor_start: { // solid White
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 255 * light_level_, 255 * light_level_);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_wait_for_enable_window: { // solid blue
                    rgb_setpoint_leds[1] = CRGB(0, 0, 255 * light_level_);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_wait_for_lightlevel: { // solid yellow
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 255 * light_level_, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_c_OK: { // solid green
                    rgb_setpoint_leds[1] = CRGB(0, 255 * light_level_, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_x_OK: { // solid cyan
                    rgb_setpoint_leds[1] = CRGB(0, 255 * light_level_, 255 * light_level_);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_blind_OK: { // solid White
                    rgb_setpoint_leds[1] = CRGB(255 * light_level_, 255 * light_level_, 255 * light_level_);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
            } case LED1_unknown: { //
            }  default: { // off
                    rgb_setpoint_leds[1] = CRGB(0, 0, 0);
                    rgb_leds[1] = rgb_setpoint_leds[1];
                    break;
                }
        }
        update_state[1] = false;
    }

    //some states that can exist in parallel with the main state:
    if (_led1_state != LED1_inresponsive_NUC && _led1_state != LED1_executor_problem) {
        if (!_internet_OK)
            blink_leds[1] = blink_3_short_blinks;
        else if (!_daemon_OK)
            blink_leds[1] = blink_2_short_blinks;
        else if (_post_processing)
            blink_leds[1] = blink_1_short_blink_reversed;
    }

    blink(0);
    blink(1);

    if (update_FastLED) {
        FastLED.show();
        update_FastLED = false;
    }
}
