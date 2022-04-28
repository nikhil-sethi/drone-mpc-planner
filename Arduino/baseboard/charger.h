#pragma once
#include <WString.h>
#include "utility.h"
#include "rgbleds.h"
class Charger {
public:
    enum charging_states {
        state_disabled,
        state_init,
        state_drone_not_on_pad,
        state_contact_problem,
        state_bat_dead,
        state_bat_does_not_charge,
        state_revive_charging,
        state_normal_charging,
        state_trickle_charging,
        state_discharge,
        state_measure,
        state_wait_until_drone_ready,
        state_calibrating
    };

    enum charging_modes {
        charging_mode_not_charging,
        charging_mode_contact_problem,
        charging_mode_normal,
        charging_mode_revive,
        charging_mode_trickle
    };
private:

    const float d_volts_is_stable_threshold = 0.05f;
    const unsigned long revive_charge_timeout = 60000L;
    const unsigned long min_volt_measuring_duration = 300L;
    const unsigned long periodic_volt_measuring_duration = 30000L;
    const unsigned long charge_amp_measurement_valid_timeout = 450L;
    const float amps_measurement_resistance = 0.333f;
    const float battery_size_mah = 300.f;

    const float min_volts_detection = 0.8f;
    const float min_battery_volts_revive_charge = 0.9f;
    const float min_battery_volts_normal_charge = 1.4f; //TMP! Should be 2.5 or something
    const float battery_volts_very_empty = 3.5f;
    const float battery_volts_almost_full = 4.1f;
    const float min_battery_volts_trickle_charge = 4.15f;
    const float max_battery_volts = 4.20f;
    const float dangerous_battery_volts = 4.25f;
    const float min_charge_amps = 0.05f;
    const float min_charge_volts_offset = 0.6f; // the voltage over vbat at which current actually starts to flow. Probably related to the diode. And probably its not really const... https://github.com/pats-drones/pats/issues/1047
    const uint8_t min_charge_pwm = 14; // the minimum pwm at which a measurable current starts to flow. I define 0.03A - 0.07A as measurable
    const uint8_t max_charge_pwm = 60; // safety  limit, at the moment limited by the p-channel mosfet to 0.5A
    const float charge_max = battery_size_mah * 0.001f * 2.f; // 2C. Because more seems to overload the charger...
    const float charge_1C = battery_size_mah * 0.001f * 1.f;
    const float charge_half_C = battery_size_mah * 0.001f * 0.5f;

    const float p_amps_gain =  0.2f;
    const float ff_amps_gain = 50.f;
    const float p_volts_gain = 0.0005f;

    charging_states _charging_state = state_disabled;
    float setpoint_amps = 0.f;
    float setpoint_amp_prev = 0.f;
    float pv = 0.f; // control process variable

    bool volt_mode_pv_initialised = false;
    float battery_volts = 0.f;
    float d_battery_volts = 0.f;
    bool battery_volt_measurement_stable = false;
    float ground_volts = 0;
    float charging_volts = 0;
    float last_charging_volts = 0;
    float d_charging_volts = 0.f;
    float charging_amps = 0.f;
    float last_charging_amps = 0.f;
    float average_charging_amps = 0.f;
    float charge_resistance = 1.f;
    const float drone_amps_burn_initial_guess = 0.15f;
    float drone_amps_burn = drone_amps_burn_initial_guess;
    float mah_charged = 0.f;
    float voltage_calibration_value = 0.f;
    unsigned long measure_during_charge_start_time = 0L;
    unsigned long measure_during_charge_end_time = 0L;
    unsigned long charging_duration = 0L;
    unsigned long last_mah_adjust_time = 0L;
    unsigned long last_control_time = 0L;

    RGBLeds *rgbleds;

    void estimate_resistance();
    void amp_control();
    void volt_control();

    void update_pwm(uint8_t pwm_);
    void update_amps();
    void update_readings_while_charging();
    void update_readings_while_not_charging();
    void update_charging_volts();
    void update_batt_volts();

    bool measuring_time();
    void measure_during_charge();
    void charge(charging_modes mode);
    void no_charging();

    void calibrate_voltage_measurement(float volts);
    void reset_voltage_calibration();

public:
    void init(RGBLeds *rgbleds);
    void handle_serial_input_package(SerialNUC2BaseboardChargingPackage *pkg);
    void handle_serial_input_package(SerialExecutor2BaseboardAllowChargingPackage *pkg);
    void run();
    bool enabled() {return _charging_state != state_disabled;};
    charging_states charging_state() {return _charging_state;}
    void fill_serial_output_package(SerialBaseboard2NUCPackage *pkg);
};
