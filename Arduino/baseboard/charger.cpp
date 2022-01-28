#include "charger.h"
#include "defines.h"
#include "utility.h"
#include "eeprom_settings.h"
#include <EEPROM.h>
#include "Arduino.h"



void Charger::init() {
    analogReference(EXTERNAL);

    pinMode(CHARGING_PWM_PIN, OUTPUT);
    pinMode(CHARGING_VOLTAGE_PIN, INPUT);
    pinMode(CHARGING_CURRENT_PIN, INPUT);

    if (charger_state_eeprom())
        _charging_state = state_init;
    else
        _charging_state = state_disabled;
}

void Charger::run() {
    switch (_charging_state) {
        case state_disabled: {
                disable_charging();
                break;
        } case state_init: {
                if (calibration_mode_eeprom())
                    init_voltage_calibration();
                else
                    _charging_state = state_drone_not_on_pad;
                break;
        } case state_drone_not_on_pad: {
                disable_charging();
                update_readings(0.5f);
                battery_volts = smoothed_volts;
                pv = 0;
                volt_mode_pv_initialised = false;
                setpoint_amp_prev = 0;
                charging_duration = 0;
                volts_before_measuring = 0;
                mah_charged = 0;
                if (smoothed_volts > min_volts_detection) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                }
                break;
        } case state_contact_problem: {
                if (millis() - measure_during_charge_end_time > periodic_volt_measuring_duration ||
                        (measured_smoothed_amps > min_charge_amps && millis() - measure_during_charge_end_time > charge_amp_measurement_valid_timeout)) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_contact_problem);
                break;
        } case state_bat_dead: {
                disable_charging();
                update_readings(0.5f);
                battery_volts = smoothed_volts;
                if (smoothed_volts > min_volts_detection)
                    _charging_state = state_drone_not_on_pad;
                break;
        } case state_bat_does_not_charge: {
                disable_charging();
                update_readings(0.5f);
                battery_volts = smoothed_volts;
                if (smoothed_volts < min_volts_detection)
                    _charging_state = state_drone_not_on_pad;
                break;
        } case state_revive_charging: {
                if (measuring_time()) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_revive);
                break;
        } case state_normal_charging: {
                if (measuring_time()) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                if (mah_charged > 2.f * battery_size_mah)
                    _charging_state = state_bat_does_not_charge;
                charge(charging_mode_normal);
                break;
        } case state_trickle_charging: {
                if (measuring_time()) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_trickle);
                break;
        } case state_discharge: {
                disable_charging();
                update_readings(0.5f);
                battery_volts = smoothed_volts;
                if (smoothed_volts < max_battery_volts) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                }
                break;
        } case state_measure: {
                disable_charging();
                update_volts(0.5f);
                if (fabs(d_smoothed_volts) < d_voltage_is_stable_threshold && millis() - measure_during_charge_start_time > min_volt_measuring_duration) {
                    d_battery_voltage = smoothed_volts - battery_volts;
                    measure_during_charge_end_time = millis();
                    if (smoothed_volts > dangerous_battery_volts) {
                        _charging_state = state_discharge;
                        volt_mode_pv_initialised = false;
                    } else if (charging_duration >= periodic_volt_measuring_duration && measured_smoothed_amps < min_charge_amps && setpoint_amp > min_charge_amps && volts_before_measuring > smoothed_volts + min_charge_volts_offset && smoothed_volts >= min_volts_detection) {
                        _charging_state = state_contact_problem;
                        volt_mode_pv_initialised = false;
                    } else if (smoothed_volts >= min_battery_volts_trickle_charge) {
                        if (!volt_mode_pv_initialised) {
                            //initialize the pv with some current base charging first, because convergence with volts based charging is tuned to takes very long
                            // current base charging is faster because it has a ff term
                            setpoint_amp = drone_amps_burn;
                            _charging_state = state_normal_charging;
                            volt_mode_pv_initialised = true;
                        } else {
                            if (fabs(battery_volts - max_battery_volts) < 0.005f && fabs(d_battery_voltage) < 0.005f) {
                                drone_amps_burn = moving_average(0.05f, measured_smoothed_amps, drone_amps_burn);
                                drone_amps_burn = constrain(drone_amps_burn, 0, 0.3f);
                            }
                            volt_control(smoothed_volts);
                            _charging_state = state_trickle_charging;
                        }
                    } else if (smoothed_volts >= battery_volts_almost_full) {
                        setpoint_amp = charge_half_C + drone_amps_burn;
                        _charging_state = state_normal_charging;
                    } else if (smoothed_volts >= min_battery_volts_normal_charge && smoothed_volts < battery_volts_very_empty) {
                        setpoint_amp = charge_1C + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_normal_charging;
                    } else if (smoothed_volts >= min_battery_volts_normal_charge) {
                        setpoint_amp = charge_max + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_normal_charging;
                    } else if (smoothed_volts >= min_battery_volts_revive_charge && millis() < revive_charge_timeout) {
                        setpoint_amp = charge_1C + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_revive_charging;
                    } else if (smoothed_volts > min_volts_detection) {
                        _charging_state = state_bat_dead;
                        volt_mode_pv_initialised = false;
                    } else {
                        _charging_state = state_drone_not_on_pad;
                        volt_mode_pv_initialised = false;
                    }

                    battery_volts = smoothed_volts;
                }
                break;
        } case state_wait_until_drone_ready: {
                disable_charging();
                update_volts(0.5f);
                break;
        } case state_calibrating: {
                disable_charging();
                update_readings(0.02f);
                print_voltage_calibration();
                break;
        } default:
            break;
    }
}

bool Charger::measuring_time() {
    if (millis() - measure_during_charge_end_time > periodic_volt_measuring_duration)
        return true;
    else if (millis() - measure_during_charge_end_time > charge_amp_measurement_valid_timeout &&
             measured_smoothed_amps < min_charge_amps &&
             setpoint_amp > min_charge_amps &&
             smoothed_volts > battery_volts + min_charge_volts_offset)
        return true;
    else
        return false;
}

void Charger::update_readings(float alpha) {
    update_amps();
    update_volts(alpha);
    float dt = (millis() - last_amps_measure_time);
    last_amps_measure_time = millis();
    mah_charged += (measured_smoothed_amps - drone_amps_burn)  * dt / 3600.f ;
    charging_duration += dt;
}

float Charger::volts_on_pads_uncalibrated() {

    return (analogRead(CHARGING_VOLTAGE_PIN) + 0.5f) * 5.0f / 1024.0f;
}
float Charger::volts_on_pads() {
    return volts_on_pads_uncalibrated() + voltage_calibration_value;
}
void Charger::update_volts(float alpha) {
    float v = moving_average(alpha, volts_on_pads(), smoothed_volts);
    d_smoothed_volts = v - smoothed_volts;
    smoothed_volts = v;
}

void Charger::update_amps() {
    float volts = analogRead(CHARGING_CURRENT_PIN) / 1024.0f * 5.0f;
    measured_amps = volts / current_measurement_resistance;
    measured_smoothed_amps = moving_average(0.05, measured_amps, measured_smoothed_amps);
    measured_display_amps = moving_average(0.005, measured_amps, measured_display_amps);

}

void Charger::disable_charging() {
    pwm = 0;
    analogWrite(CHARGING_PWM_PIN, 0);
}
void Charger::charge(charging_modes mode) {
    update_readings(0.1f);
    volts_before_measuring = smoothed_volts;
    estimate_resistance();
    if (mode == charging_mode_contact_problem) {
        pwm = min_charge_pwm;
        analogWrite(CHARGING_PWM_PIN, pwm);
    } else if (mode != charging_mode_trickle)
        current_control();
}

void Charger::estimate_resistance() {
//     |--[Rc]--|--[D]---|----------------|    Rc = R contact
//     |        |--[Rd]--|    |           |    Rd = R parallel to diode on drone
//     |                      |           |    Rb = R battery
//     |                      |           |    L = load from drone
//     |                     (Vb)         |
//    (Vc)                    |          {L}
//     |                     [Rb]         |
//     |                      |           |    Vc = V charge
//     |                      |           |    Vb = V bat
//     |______________________|___________|
    if (measured_smoothed_amps > 0.1f) {
        float unfiltered_contact_r = (smoothed_volts - battery_volts) / measured_smoothed_amps;
        charge_resistance = moving_average(0.005, unfiltered_contact_r, charge_resistance); // R_contact + R_battery
    }
}
void Charger::current_control() {
    if (setpoint_amp > 0.f) {
        float dt = (millis() - last_control_time) ;
        last_control_time = millis();

        //feedback
        float fb_error = (setpoint_amp - measured_amps);
        float fb_term = fb_error * p_current_gain;
        //feed forward, not completely ironed out. https://github.com/pats-drones/pats/issues/1047
        float ff_error = (setpoint_amp - setpoint_amp_prev);

        float ff_term;
        if (setpoint_amp - setpoint_amp_prev > 0 && pwm == 0) {
            ff_error -= 0.15; // this is a guestimate of what the extra current should with adding the min_charge_pwm
            ff_term = ff_error * ff_current_gain + min_charge_pwm;
        } else {
            ff_term = ff_error * ff_current_gain;
        }

        pv = constrain(pv + constrain(ff_term + fb_term, -60, 60), 0, 255); // constrained because some serious non-lineairy of the charger
        pwm = roundf(pv);
        analogWrite(CHARGING_PWM_PIN, pwm);
    } else
        analogWrite(CHARGING_PWM_PIN, 0);
    setpoint_amp_prev = setpoint_amp;
}
void Charger::volt_control(float measured_battery_volts) {
    float dt = (millis() - last_control_time) ;
    last_control_time = millis();
    float error = (max_battery_volts - measured_battery_volts);
    pv = constrain(pv + error * p_volts_gain * dt, 0, 255);
    pwm = roundf(pv);
    analogWrite(CHARGING_PWM_PIN, pwm);
    setpoint_amp = drone_amps_burn;
}

/*** serial commands handling ***/
void Charger::fill_serial_output_package(SerialBaseboard2NUCPackage *pkg) {
    pkg->charging_state = _charging_state;
    pkg->battery_volts = battery_volts;
    pkg->charging_volts = smoothed_volts;
    pkg->charging_amps = measured_smoothed_amps;
    pkg->setpoint_amp = setpoint_amp;
    pkg->mah_charged  = mah_charged;
    pkg->charge_resistance = charge_resistance;
    pkg->drone_amps_burn = drone_amps_burn;
    pkg->charging_pwm = pwm;
    pkg->charging_duration  = charging_duration;
}

void Charger::handle_serial_input_package(SerialExecutor2BaseboardAllowChargingPackage *pkg) {
    if (_charging_state == state_disabled)
        return;
    else if (!pkg->allow_charging)
        _charging_state = state_wait_until_drone_ready;
    else if (_charging_state == state_wait_until_drone_ready)
        _charging_state = state_measure;
}

void Charger::handle_serial_input_package(SerialNUC2BaseboardChargingPackage *pkg) {
    if (pkg->enable_charging && _charging_state == state_disabled) {
        debugln("Enable charging");
        write_charger_state_eeprom(true);
        _charging_state = state_init;
    } else  if (!pkg->enable_charging && _charging_state != state_disabled) {
        debugln("Disable charging");
        _charging_state = state_disabled;
        write_charger_state_eeprom(false);
    }

    if (_charging_state != state_disabled) {
        if (pkg->calibrate)
            calibrate_voltage_measurement(pkg->volts);
        if (pkg->reset_calibration)
            reset_voltage_calibration();
    }
}

void Charger::reset_voltage_calibration() {
    reset_calibration_eeprom();
    _charging_state = state_calibrating;
    voltage_calibration_value = 0;
}
void Charger::init_voltage_calibration() {
    _charging_state = state_calibrating;
    voltage_calibration_value = voltage_calibration_value_eeprom();

    char str_volt[6];
    dtostrf(voltage_calibration_value, 4, 2, str_volt);
    debugln("volts calibration %s", str_volt);
}
void Charger::calibrate_voltage_measurement(float volts) {
    Serial.println("calibrate: ");
    Serial.println(volts);

    voltage_calibration_value += volts - smoothed_volts;

    if (fabs(voltage_calibration_value) > 0.2f) {
        voltage_calibration_value = 0.0f;
        Serial.println("ERROR: invalid calibration:");
        Serial.println(voltage_calibration_value);
        Serial.println(volts);
        return;
    }

    write_calibration_eeprom(voltage_calibration_value);
}
void Charger::print_voltage_calibration() {
    static unsigned long calibration_print_time = millis();
    if (millis() - calibration_print_time > 250) {
        calibration_print_time = millis();
        Serial.print("analog val ");
        Serial.print(analogRead(CHARGING_VOLTAGE_PIN));
        Serial.print(" volt_raw ");
        Serial.print(volts_on_pads_uncalibrated());
        Serial.print(" voltage_calibration_value ");
        Serial.print(voltage_calibration_value);
        Serial.print(" smoothed_volts ");
        Serial.print(smoothed_volts);
        Serial.println("");
    }
}
