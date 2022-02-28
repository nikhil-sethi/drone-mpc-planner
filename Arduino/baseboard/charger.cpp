#include "charger.h"
#include "defines.h"
#include "utility.h"
#include "eeprom_settings.h"
#include <EEPROM.h>
#include "Arduino.h"

//hack necessary because these variables need to be accessible from the interrupt routine:
static volatile bool stop_charging_interrupt = false;
static volatile unsigned char pwm = 0;

void stop_charging_interrupt_handle() {
    if (pwm) {
        stop_charging_interrupt = true;
        digitalWrite(CHARGING_ENABLE_PIN, 0);
        analogWrite(CHARGING_PWM_PIN, 255);
    }
}

void Charger::init(RGBLeds *rgbleds_) {
    rgbleds = rgbleds_;
    analogReference(INTERNAL);

    pinMode(CHARGING_PWM_PIN, OUTPUT);
    pinMode(CHARGING_ENABLE_PIN, OUTPUT);
    pinMode(CHARGING_BATT_VOLT_PIN, INPUT);
    pinMode(CHARGING_AMPS_PIN, INPUT);
    pinMode(CHARGING_OVERLOAD_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CHARGING_OVERLOAD_INTERRUPT_PIN), stop_charging_interrupt_handle, FALLING);

    if (charger_state_eeprom())
        _charging_state = state_init;
    else
        _charging_state = state_disabled;

    TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    // TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
    update_pwm(0);
}

void Charger::run() {
    switch (_charging_state) {
        case state_disabled: {
                if (pwm)
                    update_pwm(0);
                rgbleds->led0_state(RGBLeds::LED0_disabled);
                break;
        } case state_init: {
                rgbleds->led0_state(RGBLeds::LED0_init);
                _charging_state = state_drone_not_on_pad;
                break;
        } case state_drone_not_on_pad: {
                no_charging();
                pv = 0;
                volt_mode_pv_initialised = false;
                setpoint_amp_prev = 0;
                charging_duration = 0;
                mah_charged = 0;
                if (battery_volts > min_volts_detection) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                }
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_contact_problem: {
                if (millis() - measure_during_charge_end_time > periodic_volt_measuring_duration ||
                        (charging_amps > min_charge_amps && millis() - measure_during_charge_end_time > charge_amp_measurement_valid_timeout)) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_contact_problem);
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_bat_dead: {
                no_charging();
                if (battery_volts > min_volts_detection)
                    _charging_state = state_drone_not_on_pad;
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_bat_does_not_charge: {
                no_charging();
                if (battery_volts < min_volts_detection)
                    _charging_state = state_drone_not_on_pad;
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_revive_charging: {
                if (measuring_time()) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_revive);
                if (charging_amps < min_charge_amps)
                    rgbleds->led0_state(RGBLeds::LED0_not_charging);
                else
                    rgbleds->led0_state(RGBLeds::LED0_charging);
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
                if (charging_amps < min_charge_amps)
                    rgbleds->led0_state(RGBLeds::LED0_not_charging);
                else
                    rgbleds->led0_state(RGBLeds::LED0_charging);
                break;
        } case state_trickle_charging: {
                if (measuring_time()) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                    break;
                }
                charge(charging_mode_trickle);
                if (charging_amps < min_charge_amps)
                    rgbleds->led0_state(RGBLeds::LED0_not_charging);
                else
                    rgbleds->led0_state(RGBLeds::LED0_charging);
                break;
        } case state_discharge: {
                no_charging();
                if (battery_volts < max_battery_volts) {
                    measure_during_charge_start_time = millis();
                    _charging_state = state_measure;
                }
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_measure: {
                no_charging();
                if (battery_volt_measurement_stable && millis() - measure_during_charge_start_time > min_volt_measuring_duration) {
                    measure_during_charge_end_time = millis();
                    if (battery_volts > dangerous_battery_volts) {
                        _charging_state = state_discharge;
                        volt_mode_pv_initialised = false;
                    } else if (charging_duration >= periodic_volt_measuring_duration && charging_amps < min_charge_amps && setpoint_amps > min_charge_amps && charging_volts > battery_volts + min_charge_volts_offset && battery_volts >= min_volts_detection) {
                        _charging_state = state_contact_problem;
                        volt_mode_pv_initialised = false;
                    } else if (battery_volts >= min_battery_volts_trickle_charge) {
                        if (!volt_mode_pv_initialised) {
                            //initialize the pv with some current base charging first, because convergence with volts based charging is tuned to takes very long
                            // current base charging is faster because it has a ff term
                            setpoint_amps = drone_amps_burn;
                            _charging_state = state_normal_charging;
                            volt_mode_pv_initialised = true;
                        } else {
                            if (fabs(battery_volts - max_battery_volts) < 0.005f && fabs(d_battery_volts) < 0.005f) {
                                drone_amps_burn = moving_average(0.05f, charging_amps, drone_amps_burn);
                                drone_amps_burn = constrain(drone_amps_burn, 0, 0.3f);
                            }
                            volt_control();
                            _charging_state = state_trickle_charging;
                        }
                    } else if (battery_volts >= battery_volts_almost_full) {
                        setpoint_amps = charge_half_C + drone_amps_burn;
                        _charging_state = state_normal_charging;
                    } else if (battery_volts >= min_battery_volts_normal_charge && battery_volts < battery_volts_very_empty) {
                        setpoint_amps = charge_1C + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_normal_charging;
                    } else if (battery_volts >= min_battery_volts_normal_charge) {
                        setpoint_amps = charge_max + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_normal_charging;
                    } else if (battery_volts >= min_battery_volts_revive_charge && millis() < revive_charge_timeout) {
                        setpoint_amps = charge_1C + drone_amps_burn;
                        volt_mode_pv_initialised = false;
                        _charging_state = state_revive_charging;
                    } else if (battery_volts > min_volts_detection) {
                        _charging_state = state_bat_dead;
                        volt_mode_pv_initialised = false;
                    } else {
                        _charging_state = state_drone_not_on_pad;
                        volt_mode_pv_initialised = false;
                    }
                }
                break;
        } case state_wait_until_drone_ready: {
                no_charging();
                rgbleds->led0_state(RGBLeds::LED0_not_charging);
                break;
        } case state_calibrating: {
                no_charging();
                rgbleds->led0_state(RGBLeds::LED0_calibrating);
                break;
        } default:
            break;
    }
}

bool Charger::measuring_time() {
    if (stop_charging_interrupt) {
        stop_charging_interrupt = false;
        debugln("stop_charging_interrupt!")
        return true;
    }
    if (millis() - measure_during_charge_end_time > periodic_volt_measuring_duration)
        return true;
    else if (millis() - measure_during_charge_end_time > charge_amp_measurement_valid_timeout &&
             charging_amps < min_charge_amps &&
             setpoint_amps > min_charge_amps &&
             charging_volts > battery_volts + min_charge_volts_offset) {
        update_pwm(0); // because slightly faster response
        return true;
    } else
        return false;
}

void Charger::update_readings_while_not_charging() {
    charging_amps = 0;
    update_batt_volts();
    float dt = (millis() - last_mah_adjust_time);
    last_mah_adjust_time = millis();
    mah_charged -=  drone_amps_burn  * dt / 3600.f;
    update_charging_volts();
}

void Charger::update_readings_while_charging() {
    update_amps();
    update_charging_volts();
    float dt = (millis() - last_mah_adjust_time);
    last_mah_adjust_time = millis();
    mah_charged += (charging_amps - drone_amps_burn)  * dt / 3600.f ;
    charging_duration += dt;
}

void Charger::update_charging_volts() {
    float v_gnd = (analogRead(CHARGING_GND_PIN) + 0.5f) * 5.0f / 1024.0f;
    float vdd = (analogRead(CHARGING_VOLT_PIN) + 0.5f) * 5.0f / 1024.0f;
    float v = (vdd - v_gnd) * 4.f; // + voltage_calibration_value;
    d_charging_volts = charging_volts - v;
    charging_volts = v;
}
void Charger::update_batt_volts() {
    float v_gnd = (analogRead(CHARGING_GND_PIN) + 0.5f) * 5.0f / 1024.0f;
    float vdd = (analogRead(CHARGING_VOLT_PIN) + 0.5f) * 5.0f / 1024.0f;
    float v = (vdd - v_gnd) * 4.f; //  + voltage_calibration_value;
    d_battery_volts = battery_volts - v;
    battery_volt_measurement_stable =  fabs(d_battery_volts) < d_volts_is_stable_threshold;
    battery_volts = v;
}

void Charger::update_amps() {
    float volts = analogRead(CHARGING_AMPS_PIN) / 1024.0f * 5.0f;
    charging_amps = volts / amps_measurement_resistance;
    charging_amps = moving_average(0.05, charging_amps, charging_amps);
}

void Charger::update_pwm(uint8_t pwm_) {
    if (pwm_ > 50)
        pwm_ = 50; //TMP
    pwm = pwm_;

    if (pwm_) {
        noInterrupts();
        if (!stop_charging_interrupt) {
            analogWrite(CHARGING_PWM_PIN, pwm);
            digitalWrite(CHARGING_ENABLE_PIN, 1);
        }
        interrupts();
    } else {
        noInterrupts();
        if (!stop_charging_interrupt) {
            digitalWrite(CHARGING_ENABLE_PIN, 0);
            delay(1);
            analogWrite(CHARGING_PWM_PIN, 255);
        }
        interrupts();
    }
}

void Charger::no_charging() {
    update_readings_while_not_charging();
    setpoint_amps = 0;
    update_pwm(0);
}
void Charger::charge(charging_modes mode) {
    update_readings_while_charging();
    estimate_resistance();
    if (mode == charging_mode_contact_problem) {
        update_pwm(min_charge_pwm);
    } else if (mode != charging_mode_trickle)
        amp_control();
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
    if (charging_amps > 0.1f) {
        float unfiltered_contact_r = (charging_volts - battery_volts) / charging_amps;
        charge_resistance = moving_average(0.005f, unfiltered_contact_r, charge_resistance); // R_contact + R_battery
    }
}
void Charger::amp_control() {
    if (setpoint_amps > 0.f) {
        float dt = (millis() - last_control_time) ;
        last_control_time = millis();

        //feedback
        float fb_error = (setpoint_amps - charging_amps);
        float fb_term = fb_error * p_amps_gain;
        //feed forward, not completely ironed out. https://github.com/pats-drones/pats/issues/1047
        float ff_error = (setpoint_amps - setpoint_amp_prev);

        float ff_term;
        if (setpoint_amps - setpoint_amp_prev > 0 && pwm == 0) {
            ff_error -= drone_amps_burn_initial_guess; // this is a guestimate of what the extra current should with adding the min_charge_pwm
            ff_term = ff_error * ff_amps_gain + min_charge_pwm;
        } else {
            ff_term = ff_error * ff_amps_gain;
        }
        if (charging_amps > 1.0f && charging_volts > battery_volts)  // this may happen where there was a bad / resistive contact for a while (driving up the pwm), and someone suddenly pushes on the drone
            pv = min_charge_pwm;
        else if (charging_amps > 0.7f && charging_volts > battery_volts)  // this may happen where there was a bad / resistive contact for a while (driving up the pwm), and someone suddenly pushes on the drone
            pv = 2 * min_charge_pwm;
        else
            pv = constrain(pv + constrain(ff_term + fb_term, -255, 60), 0, max_charge_pwm); // constrained because some serious non-lineairy of the charger
        update_pwm(roundf(pv));
    } else
        update_pwm(0);
    setpoint_amp_prev = setpoint_amps;
}
void Charger::volt_control() {
    float dt = (millis() - last_control_time) ;
    last_control_time = millis();
    float error = (max_battery_volts - battery_volts);
    pv = constrain(pv + error * p_volts_gain * dt, 0, 255);
    update_pwm(roundf(pv));
    analogWrite(CHARGING_PWM_PIN, pwm);
    setpoint_amps = drone_amps_burn;
}

/*** serial commands handling ***/
void Charger::fill_serial_output_package(SerialBaseboard2NUCPackage *pkg) {
    pkg->charging_state = _charging_state;
    pkg->battery_volts = battery_volts;
    pkg->charging_volts = charging_volts;
    pkg->charging_amps = charging_amps;
    pkg->setpoint_amps = setpoint_amps;
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
void Charger::calibrate_voltage_measurement(float volts) {
    if (_charging_state == state_calibrating) {
        voltage_calibration_value = volts - battery_volts;
        _charging_state == state_init;
        write_calibration_eeprom(voltage_calibration_value);
        Serial.print("voltage_calibration_value: ");
        Serial.println(voltage_calibration_value);
    } else {
        Serial.println("Error: calibrate package received while not in calibrating mode!");
    }
}
