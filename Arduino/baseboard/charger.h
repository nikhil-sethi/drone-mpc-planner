#pragma once

#include <WString.h>

#define LED_ENABLE_PIN 7
// #define DEBUG
// #define ENABLE_BEEPING
class Charger {
public:
    void init();
    void handle_commands(char*  input);
    void run();
    void append_charging_log(char* buf);
    bool enabled() {return charging_state != state_disabled;};
    bool calibrating() {return charging_state == state_calibrating;}

private:
    enum State {
        state_disabled,
        state_init,
        state_charging,
        state_check_battery_voltage_init,
        state_check_battery_voltage_turbo,
        state_check_battery_voltage,
        state_init_calibrating,
        state_calibrating
    };

    enum BeepState {
        beep_version = 0,
        beep_charging,
        beep_no_current,
        beep_no_drone,
        beep_no_drone_alarm,
        beep_drone_detected,
        beep_calibrating_init,
        beep_calibrating,
        beep_drone_disconnected,
        beep_drone_connected,
        beep_voltage_too_low,
        beep_silent,
        beep_drone_connected_turbo,
        beep_charging_turbo
    };

    enum BeepEvent {
        beep_event_no_event,
        beep_event_no_drone,
        beep_event_drone_detected,
        beep_event_dead_drone_detected,
        beep_event_no_current,
        beep_event_current,
        beep_event_calibrate,
        beep_event_voltage_too_low,
        beep_event_charging_disabled,
        beep_event_drone_detected_turbo,
        beep_event_charging
    };

    enum ChargingFlag {
        charging_flag_drone_connected = 0
    };

    void run_state_machine();

    void print_voltage_calibration();
    void init_voltage_calibration();
    void calibrate_voltage_measurement(float voltage);
    void reset_voltage_calibration();
    void handle_calibration_commands(char*  input);

    String beep_state_name(BeepState state);
    void set_beep_state(BeepState state);
    bool beep_state_timeout(int time_in_millis);
    void handle_beep_event(BeepEvent event);

    String state_name(State state);
    void print_charging_state();
    void update_desired_amperage();

    void update_amperage();
    float voltage_on_pads();
    float voltage_on_pads_uncalibrated();
    void update_readings(float alpha);
    void update_smoothed_voltage(float alpha);
    void update_smoothed_amperage();
    void update_display_amperage();

    void no_charging();
    void update_pwm_value(float amperage_setpoint);

    void enable_charger();
    void disable_charger();
    void handle_charger_enable_commands(char*  input);
    void charging_flag(uint8_t pos, bool value);

    float desired_amperage = 0.0f;
    float charging_pwm_value = 0.0f;
    float battery_voltage = 0.0f;
    float smoothed_amperage = 0.0f;
    float display_amperage = 0.0f;
    float smoothed_voltage = 0.0f;
    float amperage = 0.0f;

    const unsigned long turbo_charge_timeout = 60000;
    const float current_measurement_resistance = 0.33f;
    float voltage_calibration_value = 0.0f;
    const float min_battery_voltage_turbo = 1.f;
    const float min_battery_voltage = 2.5f;
    const float trickle_battery_voltage = 4.15f;
    const float max_battery_voltage = 4.20f;
    const float dangerous_battery_voltage = 4.25f;

    unsigned long drone_connected_time = 0L;
    uint8_t charging_flags = 0;

    State charging_state = state_disabled;
    BeepState beep_state = beep_version;
};
