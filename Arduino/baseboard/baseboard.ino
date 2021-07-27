#include <EEPROM.h>
#include "pats_tunes.h"

#define debugln(msg, ...)                          \
  {                                                \
    char debug_buf[64];                            \
    sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); \
    Serial.write(debug_buf);                       \
  }

#define PWM_PIN 9
#define VOLTAGE_PIN A2
#define CURRENT_PIN A0
#define LED_ENABLE_PIN 7
#define NUC_ENABLE_PIN 8

#define EEPROM_CALIB_VALUE 0
#define EEPROM_WATCHDOG_STATE 1
#define EEPROM_CALIB_DONE_START 2
#define EEPROM_CALIB_DONE_STEPS 5

#define SOFTWARE_VERSION 001
// #define CHARGING_ENABLED

enum State {
    state_idle,
    state_charging,
    state_check_battery_voltage_init,
    state_check_battery_voltage,
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
    beep_drone_connected
};

enum BeepEvent {
    beep_event_no_event,
    beep_event_no_drone,
    beep_event_drone_detected,
    beep_event_no_current,
    beep_event_current,
    beep_event_calibrate,
};

unsigned long last_beep_state_change = 0;

int charging_state = 0;

float desired_amperage = 0;
float charging_pwm_value = 0;
float battery_voltage = 0;
float smoothed_amperage = 0;
float smoothed_voltage = 0;

float current_measurement_resistance = 0.33;
float voltage_calibration_value = 0.0;
float min_battery_voltage = 1.0;
float max_battery_voltage = 4.2;

#define HOUR_IN_MILLIS 3600L * 1000L
#define DAY_IN_MILLIS 24L * 3600L * 1000L
#define NUC_OFF_TIME 10000L
#define WATCHDOG_TIMEOUT HOUR_IN_MILLIS
#define VOLTAGE_SETTLE_TIME 3000

bool reset_nuc = false;
bool watchdog_enabled = false;
unsigned long last_nuc_reset = 0;
unsigned long nuc_watchdog_timer = millis();

uint8_t drone_detected_beeper_flag = false;
unsigned long total_charging_time = 0;
unsigned long last_detect_time;

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5);
    analogReference(EXTERNAL);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(BEEPER_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);

    digitalWrite(LED_ENABLE_PIN, 1);
    pinMode(LED_ENABLE_PIN, OUTPUT);
    digitalWrite(LED_ENABLE_PIN, 1);

    digitalWrite(NUC_ENABLE_PIN, 1);
    pinMode(NUC_ENABLE_PIN, OUTPUT);
    digitalWrite(NUC_ENABLE_PIN, 1);

    charging_state = state_check_battery_voltage;

    smoothed_amperage = 0;
    smoothed_voltage = 0;

    init_calibration();
    watchdog_enabled = read_watchdog_state_from_eeprom();

    debugln("baseboard started");
}

void apply_loop_delay(int loop_time) {
    static unsigned long timer = millis();
    int wait_time = min(loop_time, timer + loop_time - millis());
    timer = millis();
    if (wait_time > 0)
        delay(min(wait_time,1));
}

void loop() {
    apply_loop_delay(25);

    String input = serial_input();

    handle_watchdog(input);
    handle_led_commands(input);
    handle_reboot_commands(input);
    handle_nuc_reset();

    handle_beep_event(beep_event_no_event);
#ifdef CHARGING_ENABLED
    handle_calibration_commands(input);

    calculate_smoothed_voltage();
    calculate_smoothed_amperage();


    static long next_run_time = 0;
    if (millis() > next_run_time)
        next_run_time = millis() + run_state_machine();

    if (charging_state != state_calibrating) {
        set_amperage(desired_amperage);
    }
#endif
    run_display();
}

void run_display(void) {
    static long value_update_time = 0;
    if (millis() - value_update_time > 500) {
        static char buf[100];
        sprintf(buf, "disp: len, 93, ver, %03d, vlt, %03d, lvl, %03d, cur, %03d, csp, %03d, stt, %03d, out, %03d, led, %03d",
                SOFTWARE_VERSION, int(smoothed_voltage * 100), int(battery_voltage * 100), int(smoothed_amperage * 1000),
                int(desired_amperage * 1000), charging_state, int(charging_pwm_value), int(digitalRead(LED_ENABLE_PIN)));
        Serial.println(buf);
        value_update_time = millis();
    }
}
bool match_command(String input, String command) {
    return input.substring(0, command.length()) == command;
}

void handle_led_commands(String input) {
    if (match_command(input, "enable led")) {
        debugln("Enable LED");
        digitalWrite(LED_ENABLE_PIN, 1);
    }
    else if (match_command(input, "disable led")) {
        debugln("Disable LED");
        digitalWrite(LED_ENABLE_PIN, 0);
    }
}

void handle_reboot_commands(String input) {
    if (match_command(input, "reboot nuc")) {
        debugln("Reboot NUC");
        reset_nuc = true;
    }
}

void handle_watchdog(String input) {
    if (input.length() > 0)
        nuc_watchdog_timer = millis();

    if (match_command(input, "enable watchdog")) {
        debugln("Watchdog enabled");
        watchdog_enabled = true;
        write_watchdog_state_to_eeprom();
    }
    else if (match_command(input, "disable watchdog")) {
        debugln("Watchdog disabled");
        watchdog_enabled = false;
        write_watchdog_state_to_eeprom();
    }

    if (!watchdog_enabled)
        return;

    if (millis() - last_nuc_reset < DAY_IN_MILLIS && last_nuc_reset)
        return;

    if (millis() - nuc_watchdog_timer > WATCHDOG_TIMEOUT)
        reset_nuc = true;
}

void handle_nuc_reset() {
    static unsigned long nuc_off_timer = millis();

    if (digitalRead(NUC_ENABLE_PIN) && reset_nuc) {
        debugln("Turn off NUC");
        nuc_off_timer = millis();
        digitalWrite(NUC_ENABLE_PIN, 0);
        reset_nuc = false;
        last_nuc_reset = millis();
        nuc_watchdog_timer = millis();
    }

    if (millis() - nuc_off_timer > NUC_OFF_TIME && !digitalRead(NUC_ENABLE_PIN)) {
        debugln("Turn on NUC");
        digitalWrite(NUC_ENABLE_PIN, 1);
    }
}

void run_calibration() {
    analogWrite(PWM_PIN, 0);

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

String serial_input() {
    String input = Serial.readStringUntil('\n');;

    if (input.length() > 0)
        Serial.println(input);

    return input;
}

float moving_average(float alpha, float value, float smoothed_value) {
    return alpha * value + (1.0 - alpha) * smoothed_value;
}

void handle_calibration_commands(String input) {
    String command, value;

    if (input.length() > 0) {
        command = input.substring(0, 5);
        value = input.substring(6, 10);

        if (command == "calib")
            calibrate(value.toFloat());

        if (command == "reset") {
            reset_calibration_eeprom();
            charging_state = state_calibrating;
            voltage_calibration_value = 0;
        }
    }
}

void init_calibration() {
    if (read_calibration_mode_from_eeprom())
        charging_state = state_calibrating;

    if (charging_state == state_calibrating)
        return;

    voltage_calibration_value = read_calibration_value_from_eeprom();

    char str_volt[6];
    dtostrf(voltage_calibration_value, 4, 2, str_volt);
    debugln("voltage calibration %s", str_volt);
}

void reset_calibration_eeprom() {
    for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
        EEPROM.write(i + EEPROM_CALIB_DONE_START, 0);
    }
}

float read_calibration_value_from_eeprom() {
    int8_t value = EEPROM.read(EEPROM_CALIB_VALUE);
    return float(value) / 100;
}

bool read_watchdog_state_from_eeprom() {
    return EEPROM.read(EEPROM_WATCHDOG_STATE);
}

void write_watchdog_state_to_eeprom() {
    EEPROM.write(EEPROM_WATCHDOG_STATE, int8_t(watchdog_enabled));
}

bool read_calibration_mode_from_eeprom() {
    for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
        if (EEPROM.read(i + EEPROM_CALIB_DONE_START) != i) {
            return true;
        }
    }
    return false;
}

void write_calibration_to_eeprom() {
    for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
        EEPROM.write(i + EEPROM_CALIB_DONE_START, i);
    }

    EEPROM.write(EEPROM_CALIB_VALUE, int8_t(voltage_calibration_value * 100));
}

void calibrate(float voltage) {
    Serial.println("calibrate: ");
    Serial.println(voltage);

    voltage_calibration_value += voltage - smoothed_voltage;

    if (fabs(voltage_calibration_value > 0.2)) {
        voltage_calibration_value = 0;
        Serial.println("ERROR: invalid calibration:");
        Serial.println(voltage_calibration_value);
        Serial.println(voltage);
        return;
    }

    write_calibration_to_eeprom();
}

String beep_state_name(int state) {
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
    default:
        return "unknown state";
    }
}

static int beep_state = 0;
void set_beep_state(int state) {
    static int old_state = state;
    beep_state = state;
    reset_beep_time();

    Serial.print("switching form beep state   ");
    Serial.print(beep_state_name(charging_state));
    Serial.print("    to    ");
    Serial.print(beep_state_name(state));
    Serial.print("\n");
    old_state = state;
}

bool beep_state_timeout(int time_in_millis) {
    return millis() - last_beep_state_change > millis();
}

void handle_beep_event(int event) {
    static int old_beep_state = 0;

    if (charging_state == state_calibrating)
        beep_state = beep_calibrating;

    switch (beep_state) {
    case beep_charging: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        if (event == beep_event_no_current && beep_time() > 3000)
            set_beep_state(beep_no_current);
        play_tune_charging();
        break;
    }
    case beep_no_current: {
        if (event == beep_event_current)
            set_beep_state(beep_charging);
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        play_tune_no_current();
        break;
    }
    case beep_drone_disconnected: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        if (play_tune_disconnected())
            set_beep_state(beep_no_drone);
        break;
    }
    case beep_drone_connected: {
        if (event == beep_event_no_drone)
            set_beep_state(beep_drone_disconnected);
        if (play_tune_connected())
            set_beep_state(beep_charging);
        break;
    }
    case beep_no_drone: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        if (beep_time() > 20000)
            set_beep_state(beep_no_drone_alarm);
        break;
    }
    case beep_no_drone_alarm: {
        if (event == beep_event_drone_detected)
            set_beep_state(beep_drone_connected);
        play_tune_no_drone();
        break;
    }
    case beep_calibrating: {
        play_tune_calibrating();
        if (charging_state != state_calibrating)
            set_beep_state(beep_no_drone);
        break;
    }
    case beep_version: {
        if (play_tune_version(SOFTWARE_VERSION)) {
            if (event == beep_event_drone_detected)
                set_beep_state(beep_drone_connected);
            if (event == beep_event_no_drone)
                set_beep_state(beep_no_drone);
        }
        break;
    }
    }
}

String state_name(int charging_state_name) {
    switch (charging_state_name) {
    case state_idle:
        return "state_idle";
    case state_charging:
        return "state_charging";
    case state_check_battery_voltage:
        return "state_check_battery_voltage";
    case state_calibrating:
        return "state_calibrating";
    case state_check_battery_voltage_init:
        return "state_check_battery_voltage_init";
    default:
        return "unknown state";
    }
}

void set_charging_state(int new_state) {
    if (charging_state != new_state) {
        Serial.print("switching form charging state   ");
        Serial.print(state_name(charging_state));
        Serial.print("    to    ");
        Serial.print(state_name(new_state));
        Serial.print("\n");
        charging_state = new_state;
    }
}

void set_desired_amperage() {
    desired_amperage = 0;

    if (battery_voltage > max_battery_voltage)
        desired_amperage = 0;
    else if (battery_voltage > max_battery_voltage - 0.05)
        desired_amperage = 0.1;
    else if (battery_voltage > 3.5)
        desired_amperage = 0.5;
    else if (battery_voltage > min_battery_voltage)
        desired_amperage = 0.3;

    float drone_current_consumption = 0.2;
    desired_amperage += drone_current_consumption;
}

void set_min_battery_voltage() {
    int lower_min_voltage_time = 60000;
    if (millis() > lower_min_voltage_time)
        min_battery_voltage = 2.5;
}

float run_state_machine() {
    static unsigned long no_current_time = 0;
    static uint8_t isCharging = false;

    static unsigned long last_charging = 0;
    static int previous_charging_pwm_value = 0;

    static uint32_t next_check_time = 0;
    static unsigned long voltage_settle_timer = 0;

    set_min_battery_voltage();

    uint32_t check_period = 60000;
    if (millis() - total_charging_time < 41000)
        check_period = 20000;

    switch (charging_state) {
    case state_idle: {
        break;
    }
    case state_charging: {
        set_desired_amperage();

        if (smoothed_amperage >= desired_amperage / 2.0)
            handle_beep_event(beep_event_current);
        else
            handle_beep_event(beep_event_no_current);

        no_current_time = millis();

        bool no_current_timeout = millis() - no_current_time > 1000 && charging_pwm_value == 255;

        bool check_battery_voltage = false;
        if (millis() > next_check_time) {
            debugln("periodic check charge");
            check_battery_voltage = true;
        }
        if (no_current_timeout) {
            debugln("no current timeout");
            check_battery_voltage = true;
        }
        if (charging_pwm_value > previous_charging_pwm_value + 50) {
            debugln("charging_pwm_value %d previous_charging_pwm_value %d", charging_pwm_value, previous_charging_pwm_value);
            check_battery_voltage = true;
        }

        if (!check_battery_voltage)
            return 1;

        set_charging_state(state_check_battery_voltage_init);
        [[fallthrough]];
    }
    case state_check_battery_voltage_init: {
        desired_amperage = 0;
        set_amperage(0);
        previous_charging_pwm_value = charging_pwm_value;
        charging_pwm_value = 0;

        next_check_time = millis() + check_period;

        voltage_settle_timer = millis();
        set_charging_state(state_check_battery_voltage);
        return 100;
    }
    case state_check_battery_voltage: {
        battery_voltage = smoothed_voltage;
        drone_detected_beeper_flag = true;

        if (battery_voltage < min_battery_voltage) {
            drone_detected_beeper_flag = false;
            total_charging_time = millis();
            previous_charging_pwm_value = 0;
            handle_beep_event(beep_event_no_drone);
            return 100;
        }

        handle_beep_event(beep_event_drone_detected);

        if (millis() - voltage_settle_timer > VOLTAGE_SETTLE_TIME) {

            last_detect_time = millis();

            if (battery_voltage >= max_battery_voltage)
                return 100;

            set_charging_state(state_charging);
        }
        return 100;
    }
    case state_calibrating: {
        handle_beep_event(beep_event_calibrate);
        run_calibration();
        return 500;
    }
    default:
        return 1000;
    }
    return 1;
}

float voltage_on_pads() {
    return voltage_on_pads_uncalibrated() + voltage_calibration_value;
}

float voltage_on_pads_uncalibrated() {
    analogRead(VOLTAGE_PIN);
    delay(3);
    return (analogRead(VOLTAGE_PIN) + 0.5) * 5.0 / 1024.0;
}

void calculate_smoothed_voltage() {
    float alpha = 0.1;

    if (charging_state == state_calibrating)
        alpha /= 5;

    smoothed_voltage = moving_average(alpha, voltage_on_pads(), smoothed_voltage);
}

float amperage() {
    float voltage = analogRead(CURRENT_PIN) / 1024.0 * 5.0;
    voltage *= 1.3;

    return voltage / current_measurement_resistance;
}

void calculate_smoothed_amperage() {
    smoothed_amperage = moving_average(0.05, amperage(), smoothed_amperage);
}

void set_amperage(float amperage_setpoint) {
    float error = (smoothed_amperage - amperage_setpoint);

    // gain scheduling
    float p_gain = 1.0;
    if (error > 0.5)
        p_gain *= 2;
    if (error > 1.0)
        p_gain *= 2;

    charging_pwm_value -= error * p_gain;
    charging_pwm_value = constrain(charging_pwm_value, 0, 255);

    if (charging_pwm_value > 0)
        analogWrite(PWM_PIN, charging_pwm_value);
    else
        analogWrite(PWM_PIN, 0);
}
