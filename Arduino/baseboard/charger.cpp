#include "charger.h"
#include "utility.h"
#include "eeprom_settings.h"
#include <EEPROM.h>
#include "Arduino.h"
#ifdef BEEPING_ENABLED
#include "pats_tunes.h"
#endif

#define PWM_PIN 9
#define VOLTAGE_PIN A2
#define CURRENT_PIN A0

#define VOLTAGE_SETTLE_TIME 3000
#define MAX_PWM_VALUE 255.f

void Charger::init() {
    analogReference(EXTERNAL);

    pinMode(PWM_PIN, OUTPUT);
#ifdef BEEPING_ENABLED
    pinMode(BEEPER_PIN, OUTPUT);
#endif
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);

    if (charger_state_eeprom())
        charging_state = state_init;
    else
        charging_state = state_disabled;
}

void Charger::handle_commands(char* input) {
    handle_calibration_commands(input);
    handle_charger_enable_commands(input);
}

void Charger::append_charging_log(char* buf) {
    if (charging_state == state_disabled) {
        append_log_line(buf, "stt", charging_state, false);
    }
    else
    {
        append_log_line(buf, "stt", charging_state);
        append_log_line(buf, "vlt", smoothed_voltage * 100);
        append_log_line(buf, "lvl", battery_voltage * 100);
        append_log_line(buf, "cur", display_amperage * 1000);
        append_log_line(buf, "csp", desired_amperage * 1000);
        append_log_line(buf, "out", charging_pwm_value);
        append_log_line(buf, "flg", charging_flags, false);
    }
}

void Charger::run() {
    handle_beep_event(beep_event_no_event);
    run_state_machine();
}

void Charger::print_voltage_calibration() {
    static unsigned long calibration_print_time = millis();
    if (millis() - calibration_print_time > 250) {
        calibration_print_time = millis();
        Serial.print("analog val ");
        Serial.print(analogRead(VOLTAGE_PIN));
        Serial.print(" volt_raw ");
        Serial.print(voltage_on_pads_uncalibrated());
        Serial.print(" voltage_calibration_value ");
        Serial.print(voltage_calibration_value);
        Serial.print(" smoothed_voltage ");
        Serial.print(smoothed_voltage);
        Serial.println("");
    }
}

void Charger::handle_calibration_commands(char* input) {
    char subbuff[5];
    memcpy( subbuff, &input[6], 4 );
    subbuff[4] = '\0';

    if (match_command(input, "calib"))
        calibrate_voltage_measurement(atof(subbuff));

    if (match_command(input, "reset"))
        reset_voltage_calibration();
}

void Charger::enable_charger() {
    write_charger_state_eeprom(true);
    charging_state = state_init;
}

void Charger::disable_charger() {
    charging_state = state_disabled;
    handle_beep_event(beep_event_charging_disabled);
    write_charger_state_eeprom(false);
}

void Charger::handle_charger_enable_commands(char*  input) {
    if (match_command(input, (char*)"enable charger"))
        enable_charger();
    if (match_command(input, (char*)"disable charger"))
        disable_charger();
}

void Charger::reset_voltage_calibration() {
    reset_calibration_eeprom();
    charging_state = state_init_calibrating;
    voltage_calibration_value = 0;
}

void Charger::init_voltage_calibration() {
    charging_state = state_init_calibrating;
    voltage_calibration_value = voltage_calibration_value_eeprom();

    char str_volt[6];
    dtostrf(voltage_calibration_value, 4, 2, str_volt);
    debugln("voltage calibration %s", str_volt);
}

void Charger::calibrate_voltage_measurement(float voltage) {
    Serial.println("calibrate: ");
    Serial.println(voltage);

    voltage_calibration_value += voltage - smoothed_voltage;

    if (fabs(voltage_calibration_value) > 0.2f) {
        voltage_calibration_value = 0.0f;
        Serial.println("ERROR: invalid calibration:");
        Serial.println(voltage_calibration_value);
        Serial.println(voltage);
        return;
    }

    write_calibration_eeprom(voltage_calibration_value);
}

String Charger::beep_state_name(BeepState state) {
    switch (state) {
    case beep_version:
        return "beep_version";
    case beep_charging:
        return "beep_charging";
    case beep_no_current:
        return "beep_no_current";
    case beep_no_drone:
        return "beep_no_drone";
    case beep_no_drone_alarm:
        return "beep_no_drone_alarm";
    case beep_drone_detected:
        return "beep_drone_detected";
    case beep_calibrating_init:
        return "beep_calibrating_init";
    case beep_calibrating:
        return "beep_calibrating";
    case beep_drone_disconnected:
        return "beep_drone_disconnected";
    case beep_drone_connected:
        return "beep_drone_connected";
    case beep_voltage_too_low:
        return "beep_voltage_too_low";
    case beep_silent:
        return "beep_silent";
    case beep_drone_connected_turbo:
        return "beep_drone_connected_turbo";
    case beep_charging_turbo:
        return "beep_charging_turbo";
    default:
        return "unknown state";
    }
}

void Charger::set_beep_state(BeepState state) {
    static BeepState old_state = beep_state;
    beep_state = state;
#ifdef BEEPING_ENABLED
    reset_beep_time();
#endif
#ifdef DEBUG
    Serial.print("switching from beep state   ");
    Serial.print(beep_state_name(old_state));
    Serial.print("    to    ");
    Serial.print(beep_state_name(state));
    Serial.print("\n");
#endif
    old_state = state;
}

void Charger::handle_beep_event(BeepEvent event) {
#ifdef BEEPING_ENABLED
    if (event == beep_event_charging_disabled)
        set_beep_state(beep_silent);

    if (calibrating())
        beep_state = beep_calibrating;

    switch (beep_state) {
    case beep_charging: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        else if (event == beep_event_no_current)
            set_beep_state(beep_no_current);
        play_tune_charging();
        break;
    }
    case beep_drone_connected_turbo: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        else if (event == beep_event_drone_detected)
            set_beep_state(beep_charging);
        else if (play_tune_connected_turbo())
            set_beep_state(beep_charging_turbo);
        break;
    }
    case beep_charging_turbo: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        else if (event == beep_event_no_current)
            set_beep_state(beep_no_current);
        play_tune_charging_turbo();
        break;
    } case beep_no_current: {
        if (event == beep_event_current)
            set_beep_state(beep_charging);
        else if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        play_tune_no_current();
        break;
    } case beep_drone_disconnected: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        else if (event == beep_event_drone_detected_turbo)
            set_beep_state(beep_drone_connected_turbo);
        else if (play_tune_disconnected())
            set_beep_state(beep_no_drone);
        break;
    } case beep_drone_connected: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        else if (play_tune_connected())
            set_beep_state(beep_charging);
        break;
    } case beep_no_drone: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        else if (event == beep_event_drone_detected_turbo)
            set_beep_state(beep_drone_connected_turbo);
        else if (beep_time() > 20000)
            set_beep_state(beep_no_drone_alarm);
        break;
    } case beep_no_drone_alarm: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        else if (event == beep_event_drone_detected_turbo)
            set_beep_state(beep_drone_connected_turbo);
        play_tune_no_drone();
        break;
    } case beep_calibrating: {
        play_tune_calibrating();
        if (charging_state != state_calibrating)
            set_beep_state(beep_no_drone);
        break;
    } case beep_silent: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        else if (event == beep_event_drone_detected_turbo)
            set_beep_state(beep_drone_connected_turbo);
        play_tune_silent();
        break;
    }
    case beep_version: {
        if (play_tune_version(LOG_LINE_VERSION)) {
            if (event == beep_event_no_drone)
                set_beep_state(beep_no_drone);
            else if (event == beep_event_drone_detected)
                set_beep_state(beep_drone_connected);
            else if (event == beep_event_drone_detected_turbo)
                set_beep_state(beep_drone_connected_turbo);
            else if (event == beep_event_charging)
                set_beep_state(beep_drone_connected_turbo);
        }
        break;
    }
    }
#endif
}

String Charger::state_name(State state) {
    switch (state) {
    case state_disabled:
        return "state_disabled";
    case state_charging:
        return "state_charging";
    case state_check_battery_voltage_turbo:
        return "state_check_battery_voltage_turbo";
    case state_check_battery_voltage:
        return "state_check_battery_voltage";
    case state_init_calibrating:
        return "state_init_calibrating";
    case state_calibrating:
        return "state_calibrating";
    case state_check_battery_voltage_init:
        return "state_check_battery_voltage_init";
    default:
        return "unknown state";
    }
}

void Charger::print_charging_state() {
    static State old_state = state_disabled;
    if (charging_state != old_state) {
        Serial.print("switching from charging state   ");
        Serial.print(state_name(old_state));
        Serial.print("    to    ");
        Serial.print(state_name(charging_state));
        Serial.print("\n");
        old_state = charging_state;
    }
}

void Charger::update_desired_amperage() {
    const float drone_current_consumption = 0.1f;
    if (battery_voltage > dangerous_battery_voltage)
        desired_amperage = -drone_current_consumption;
    else if (battery_voltage > max_battery_voltage)
        desired_amperage = 0.0f;
    else if (battery_voltage > trickle_battery_voltage)
        desired_amperage = 0.2f;
    else if (battery_voltage > 3.5f)
        desired_amperage = 0.6f;
    else if (battery_voltage > min_battery_voltage)
        desired_amperage = 0.4f;
    else if (battery_voltage > min_battery_voltage_turbo && millis() < turbo_charge_timeout)
        desired_amperage = 0.3f;
    else
        desired_amperage = 0.0f;

    desired_amperage += drone_current_consumption;
}

void Charger::charging_flag(uint8_t pos, bool value) {
    if (pos < 8)
        if (value)
            charging_flags |= 1 << pos;
        else
            charging_flags &= 0 << pos;
}

void Charger::run_state_machine() {
    static unsigned long next_check_time = 0L;
    static unsigned long checked_voltage_time = 0L;
    static unsigned long current_can_flow_time = 0L;

    switch (charging_state) {
    case state_disabled: {
        no_charging();
        break;
    } case state_init: {
        if (calibration_mode_eeprom())
            init_voltage_calibration();
        else
            charging_state = state_check_battery_voltage_init;
        break;
    } case state_charging: {
        update_readings(0.1f);
        update_desired_amperage();
        update_pwm_value(desired_amperage);

        if (smoothed_amperage >= desired_amperage / 4.0f || charging_pwm_value < MAX_PWM_VALUE - 1)
            current_can_flow_time = millis();

        bool zero_current_timeout = false;
        if (millis() - current_can_flow_time > 1000L) {
            zero_current_timeout = true;
            if (millis() - drone_connected_time > 3000L)
                handle_beep_event(beep_event_no_current);
        }

        if (millis() > next_check_time) {
            debugln("periodic check charge");
        } else if (zero_current_timeout) {
            debugln("no current timeout");
        } else {
            handle_beep_event(beep_event_charging);
            charging_flag(charging_flag_drone_connected, 1);
            break;
        }

        charging_state = state_check_battery_voltage_init;
        [[fallthrough]];
    } case state_check_battery_voltage_init: {
        update_readings(0.1f);
        no_charging();

        checked_voltage_time = millis();
        if (checked_voltage_time < turbo_charge_timeout) {
            charging_state = state_check_battery_voltage_turbo;
            next_check_time = millis() + 60000;
        } else {
            if (millis() - drone_connected_time < 41000)
                next_check_time = millis() + 20000;
            else
                next_check_time = millis() + 60000;
            charging_state = state_check_battery_voltage;

        }
        break;
    } case state_check_battery_voltage_turbo: {
        update_readings(0.1f);
        battery_voltage = smoothed_voltage;
        if (millis() > turbo_charge_timeout)
            charging_state = state_check_battery_voltage;

        if (battery_voltage < min_battery_voltage_turbo) {
            handle_beep_event(beep_event_no_drone);
            charging_flag(charging_flag_drone_connected, 0);
            drone_connected_time = millis();
            checked_voltage_time = millis();
        } else {
            if (battery_voltage < min_battery_voltage) {
                handle_beep_event(beep_event_no_drone);
                charging_flag(charging_flag_drone_connected, 0);
            }
            else {
                handle_beep_event(beep_event_drone_detected_turbo);
                charging_flag(charging_flag_drone_connected, 1);
            }
            if (millis() - checked_voltage_time > VOLTAGE_SETTLE_TIME) {
                charging_state =state_charging;
                charging_flag(charging_flag_drone_connected, 1);
            }
        }
        break;
    } case state_check_battery_voltage: {
        update_readings(0.1f);
        battery_voltage = smoothed_voltage;

        if (battery_voltage < min_battery_voltage) {
            handle_beep_event(beep_event_no_drone);
            charging_flag(charging_flag_drone_connected, 0);
            drone_connected_time = millis();
            checked_voltage_time = millis();
        } else {
            handle_beep_event(beep_event_drone_detected);
            charging_flag(charging_flag_drone_connected, 1);
            if (millis() - checked_voltage_time > VOLTAGE_SETTLE_TIME)
                charging_state = state_charging;
        }
        break;
    } case state_init_calibrating: {
        no_charging();
        charging_state =state_calibrating;
        [[fallthrough]];
    } case state_calibrating: {
        update_readings(0.02f);
        print_voltage_calibration();
        handle_beep_event(beep_event_calibrate);
        break;
    } default:
        break;
    }

    print_charging_state();
}

float Charger::voltage_on_pads() {
    return voltage_on_pads_uncalibrated() + voltage_calibration_value;
}

float Charger::voltage_on_pads_uncalibrated() {
    // read voltage pin twice to give the internal capacitor time to settle
    analogRead(VOLTAGE_PIN);
    delay(3);
    return (analogRead(VOLTAGE_PIN) + 0.5f) * 5.0f / 1024.0f;
}

void Charger::update_readings(float alpha) {
    update_amperage();
    update_smoothed_voltage(alpha);
    update_smoothed_amperage();
    update_display_amperage();
}

void Charger::update_smoothed_voltage(float alpha) {
    smoothed_voltage = moving_average(alpha, voltage_on_pads(), smoothed_voltage);
}

void Charger::update_amperage() {
    float voltage = analogRead(CURRENT_PIN) / 1024.0f * 5.0f;
    amperage = voltage / current_measurement_resistance;
}

void Charger::update_smoothed_amperage() {
    smoothed_amperage = moving_average(0.05, amperage, smoothed_amperage);
}

void Charger::update_display_amperage() {
    display_amperage = moving_average(0.005, amperage, display_amperage);
}

void Charger::no_charging() {
    desired_amperage = 0.0f;
    charging_pwm_value = 0;
    analogWrite(PWM_PIN, 0);
}

void Charger::update_pwm_value(float amperage_setpoint) {
    float error = (amperage_setpoint - smoothed_amperage);

    // gain scheduling
    float p_gain = 0.3f;
    if (fabs(error) > 0.6f)
        p_gain *= 9;
    else if (fabs(error) > 0.3f)
        p_gain *= 3;

    charging_pwm_value += error * p_gain;
    charging_pwm_value = constrain(charging_pwm_value, 0.f, MAX_PWM_VALUE);

    if (amperage_setpoint > 0.f)
        analogWrite(PWM_PIN, roundf(charging_pwm_value));
    else
        analogWrite(PWM_PIN, 0);
}
