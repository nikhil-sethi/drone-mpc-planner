#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Wire.h>
#include "alarm.h"

#define debugln(msg, ...)                          \
  {                                                \
    char debug_buf[64];                            \
    sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); \
    Serial.write(debug_buf);                       \
  }

#define PWM_PIN 9
#define VOLTAGE_PIN A2
#define CURRENT_PIN A0

#define EEPROM_CALIB_VALUE 0
#define EEPROM_CALIB_DONE_START 1
#define EEPROM_CALIB_DONE_STEPS 10

#define SOFTWARE_VERSION 131

#define ONE_S_DRONE true

enum State { STATE_IDLE, STATE_CHARGING, STATE_CHECK_CHARGE, STATE_FULL, STATE_TESTING, STATE_CHARGING_PRECHECK };

int state = 0;

float desired_current = 0;
float output = 0;
float battery_charge = 0;
float smoothed_current;
float smoothed_voltage;

float current_measurement_resistor = 0.33;
float voltage_calibration_value = 0.0;
float min_battery_charge = 1.0;
float max_battery_charge = 8.4;

bool calibration_mode = false;

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  calcSmoothedVoltage();
  calcSmoothedCurrent();

  state = STATE_CHECK_CHARGE;

  smoothed_current = 0;
  smoothed_voltage = 0;

  analogWrite(BEEPER_PIN, 0);
}

void loop() {
  calcSmoothedVoltage();
  set_calibration_mode();

  static long next_loop_time = 0;
  if (millis() > next_loop_time)
    next_loop_time = millis() + run_state_machine();

  if (calibration_mode) {
    analogWrite(PWM_PIN, 0);
    set_alarm_calibrating();

    static long print_volt_timer = 0;
    if (millis() - print_volt_timer > 1000) {
      char str_volt[6];
      float voltage = getSmoothedVoltage();

      dtostrf(getSmoothedVoltage(), 4, 2, str_volt);
      debugln("voltage %s", str_volt);
      print_volt_timer = millis();
    }
  } else {
    set_current(desired_current);
    calcSmoothedCurrent();
    set_status_beeps();
  }

  static unsigned long display_timer = 0;
  if (millis() > display_timer) {
    run_display();
    display_timer = millis() + 300;
  }

  static unsigned long timer = millis();
  int wait_time = 10;
  wait_time = min(wait_time, timer + wait_time - millis());
  timer = millis();
}

float smooth_value(float alpha, float value, float smoothed_value) {
  return alpha * value + (1.0 - alpha) * smoothed_value;
}

void set_calibration_mode(void) {
  String input_string, command, value;

  while (Serial.available()) {
    delay(3);
    if (Serial.available() > 0) {
      char c = Serial.read();
      input_string += c;
    }
  }

  if (input_string.length() > 0) {
    Serial.println(input_string);

    command = input_string.substring(0, 5);
    value = input_string.substring(6, 10);

    if (command == "calib")
      calibrate(value.toFloat());

    if (command == "reset") {
      for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
        EEPROM.write(i + EEPROM_CALIB_DONE_START, 0);
      }
      calibration_mode = true;
      voltage_calibration_value = 0;
    }
  }

  static bool doOnce = true;
  if (doOnce) {
    for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
      if (EEPROM.read(i + EEPROM_CALIB_DONE_START) != i) {
        calibration_mode = true;
        return;
      }
    }
    int8_t value = EEPROM.read(EEPROM_CALIB_VALUE);
    ;

    voltage_calibration_value = float(value) / 100;

    char str_volt[6];
    dtostrf(voltage_calibration_value, 4, 2, str_volt);
    debugln("voltage calibration %s", str_volt);

    doOnce = false;
  }
}

void calibrate(float voltage) {
  Serial.println("calibrate: ");
  Serial.println(voltage);

  voltage_calibration_value += voltage - getSmoothedVoltage();

  if (fabs(voltage_calibration_value > 0.2)) {
    voltage_calibration_value = 0;
    Serial.println("ERROR: invalid calibration:");
    Serial.println(voltage_calibration_value);
    Serial.println(voltage);
    return;
  }

  for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
    EEPROM.write(i + EEPROM_CALIB_DONE_START, i);
  }

  EEPROM.write(EEPROM_CALIB_VALUE, int8_t(voltage_calibration_value * 100));
}

static uint8_t drone_detected = false;
static unsigned long total_charging_timer = 0;
static unsigned long last_detect_time;

void set_status_beeps(void) {
  static int old_beep_state = 0;
  enum BeepState {
    BEEP_VERSION,
    BEEP_CHARGING,
    BEEP_NO_CURRENT,
    BEEP_NO_DRONE,
    BEEP_NO_DRONE_ALARM,
    BEEP_DRONE_DETECTED
  };

  static unsigned long last_charge_time = 0;
  static unsigned long current_timeout = 0;
  unsigned long time_since_last_charge = millis() - last_charge_time;
  static int beep_state = BEEP_NO_DRONE;

  if (millis() < 5000)
    beep_state = BEEP_VERSION;
  else if (battery_charge >= max_battery_charge)
    beep_state = BEEP_CHARGING;
  else if (state == STATE_CHARGING && current_timeout > 2000L)
    beep_state = BEEP_NO_CURRENT;
  else if (!drone_detected)
    beep_state = BEEP_NO_DRONE;
  else
    beep_state = BEEP_CHARGING;

  if (getSmoothedCurrent() > 0.2 || battery_charge >= max_battery_charge)
    last_charge_time = millis();

  static unsigned long old_time = 0;
  int dt = millis() - old_time;

  if (state == STATE_CHARGING && getSmoothedCurrent() < 0.2 && battery_charge < max_battery_charge)
    current_timeout += dt;
  if (getSmoothedCurrent() > 0.2)
    current_timeout = 0;

  old_time = millis();
  if (beep_state != old_beep_state) {
    reset_beep_time();
  }

  switch (beep_state) {
    case BEEP_CHARGING: {
      if (getSmoothedCurrent() > 0.100 && millis() - total_charging_timer > 2000L)
        set_alarm_charging();
      else
        set_alarm_connected();
      break;
    }
    case BEEP_NO_CURRENT: {
      set_alarm_no_current();
      break;
    }
    case BEEP_VERSION: {
      set_alarm_version(SOFTWARE_VERSION);
      break;
    }
    case BEEP_NO_DRONE: {
      if (millis() - last_detect_time < 10000)
        set_alarm_disconnected();
      else
        set_alarm_no_drone();
      break;
    }
  }
  old_beep_state = beep_state;
}

void run_display(void) {
  static float smoothed_voltage = 0;
  smoothed_voltage = smooth_value(0.5, getSmoothedVoltage(), smoothed_voltage);

  static int disp_state, disp_output;
  static float disp_level, disp_voltage, disp_current;

  static long value_update_time = 0;
  if (millis() > value_update_time) {
    disp_output = output;
    disp_level = battery_charge;
    disp_voltage = getSmoothedVoltage();

    if (ONE_S_DRONE) {
      disp_voltage /= 2;
      disp_level /= 2;
    }
    disp_current = getSmoothedCurrent();
    value_update_time = millis() + 200;
  }

  if (!calibration_mode) {
    static char buf[100];
    sprintf(buf, "disp: len, 65, ver, %03d, vlt, %03d, lvl, %03d, cur, %03d, csp, %03d, stt, %03d, out, %03d",
            SOFTWARE_VERSION, int(disp_voltage * 100), int(disp_level * 100), int(disp_current * 1000),
            int(desired_current * 1000), state, int(output));
    Serial.println(buf);
  }
}

String get_state_name(int state) {
  switch (state) {
    case STATE_IDLE:
      return "STATE_IDLE";
    case STATE_CHARGING:
      return "STATE_CHARGING";
    case STATE_CHECK_CHARGE:
      return "STATE_CHECK_CHARGE";
    case STATE_FULL:
      return "STATE_FULL";
    case STATE_TESTING:
      return "STATE_TESTING";
    case STATE_CHARGING_PRECHECK:
      return "STATE_CHARGING_PRECHECK";
  }
}

void set_state(int new_state) {
  if (state != new_state) {
    Serial.print("switching form state   ");
    Serial.print(get_state_name(state));
    Serial.print("         ");
    Serial.print(get_state_name(new_state));
    Serial.print("\n");
    state = new_state;
  }
}

float run_state_machine() {
  if (millis() > 60000)
    min_battery_charge = 3.2;

  static int no_current_timeout = 0;
  static uint8_t isCharging = false;

  static unsigned long last_charging = 0;
  static int last_output = 0;

  switch (state) {
    case STATE_IDLE: {
      break;
    }
    case STATE_CHARGING: {
      desired_current = 0;

      if (battery_charge > max_battery_charge)
        desired_current = 0;
      else if (battery_charge > max_battery_charge - 0.05)
        desired_current = 0.1;
      else if (battery_charge > 7.0)
        desired_current = 0.5;
      else if (battery_charge > min_battery_charge)
        desired_current = 0.3;

      float drone_current_consumption = 0.2;
      desired_current += drone_current_consumption;

      uint32_t check_period = 60000;
      static uint32_t next_check_time = 0;

      if (output > 254 && getSmoothedCurrent() < 0.05)
        no_current_timeout++;
      else
        no_current_timeout = 0;

      if (millis() - total_charging_timer < 41000)
        check_period = 20000;

      if (millis() > next_check_time || no_current_timeout > 1000 || output > last_output + 50 ||
          battery_charge >= max_battery_charge) {
        debugln("no charge %d  %d  %d  %d  %d", millis() > next_check_time, no_current_timeout > 1000,
                output > last_output + 50, battery_charge >= max_battery_charge);

        set_state(STATE_CHECK_CHARGE);
        desired_current = 0;
        set_current(desired_current);
        last_output = output;
        output = 0;
        next_check_time = millis() + check_period;
        return 1;
      }
      break;
    }
    case STATE_CHECK_CHARGE: {
      set_current(0);
      float voltage = getSmoothedVoltage();

      if (voltage < min_battery_charge) {
        drone_detected = false;
        total_charging_timer = millis();
        /* debugln("no drone detected"); */
        battery_charge = voltage;
        last_output = 0;
        return 100;
      }
      last_detect_time = millis();
      drone_detected = true;
      set_state(STATE_CHARGING_PRECHECK);
      return 2000;
    }
    case STATE_CHARGING_PRECHECK: {
      battery_charge = getSmoothedVoltage();
      debugln("bettery_charge %d   %d", int(battery_charge * 1000), int(min_battery_charge * 1000));
      if (battery_charge < min_battery_charge) {
        set_state(STATE_CHECK_CHARGE);
        return 1;
      }

      if (millis() - total_charging_timer < 14000) {
        set_state(STATE_CHECK_CHARGE);
        return 1000;
      }
      set_state(STATE_CHARGING);
      no_current_timeout = 0;
      break;
    }
  }
  return 1;
}

float getVoltage() {
  for (int i = 0; i < 5; i++) {
    analogRead(VOLTAGE_PIN);
    delay(1);
  }

  float voltage = (analogRead(VOLTAGE_PIN) + 0.5) * 5.0 / 1024.0;
  static int print_count = 0;

  print_count++;
  if (print_count > 1000) {
    debugln("raw_voltage %d", analogRead(VOLTAGE_PIN));
    debugln("voltage_calibration %d", voltage_calibration_value * 1000);
    print_count = 0;
  }

  if (ONE_S_DRONE && !calibration_mode)
    voltage *= 2;

  voltage *= 2.02;
  voltage += voltage_calibration_value;
  /* debugln("%d",int(voltage*1000)); */
  return voltage;
}

float getSmoothedVoltage() {
  return smoothed_voltage;
}

void calcSmoothedVoltage() {
  float alpha = 0.1;

  if (calibration_mode)
    alpha /= 5;

  smoothed_voltage = (1 - alpha) * smoothed_voltage + alpha * getVoltage();
}

float getCurrent() {
  float voltage = analogRead(CURRENT_PIN) / 1024.0 * 5.0;
  voltage *= 1.3;

  return voltage / current_measurement_resistor;
}

float getSmoothedCurrent() {
  return smoothed_current;
}

void calcSmoothedCurrent() {
  smoothed_current = smooth_value(0.05, getCurrent(), smoothed_current);
}

void set_current(float current_setpoint) {
  float error = (getSmoothedCurrent() - current_setpoint);
  float c = 1.0;
  int max_output = 255;

  // gain scheduling
  if (error > 0.5)
    c *= 2;
  if (error > 1.0)
    c *= 2;

  output -= error * c;

  if (getSmoothedCurrent() > 1.2)
    output /= 2;

  if (output > max_output)
    output = max_output;
  if (output < 0)
    output = 0;

  int o = (int)output;
  if (current_setpoint == 0)
    o = 0;

  analogWrite(PWM_PIN, o);
}
