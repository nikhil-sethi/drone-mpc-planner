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
#define CALIB_PIN A7
#define CURRENT_PIN A0

#define RES_CHECKER_PIN 9
#define RES_SETTER_PIN 10

#define EEPROM_CALIB_VALUE 0
#define EEPROM_CALIB_DONE_START 1
#define EEPROM_CALIB_DONE_STEPS 10

#define CLK A4
#define DIO A5

#define SOFTWARE_VERSION 125

#define ONE_S_DRONE true

float voltage_calibration_value = 0.0;

float getVoltage(bool reset = false);

enum State { STATE_IDLE, STATE_CHARGING, STATE_CHECK_CHARGE, STATE_FULL, STATE_TESTING, STATE_CHARGING_PRECHECK };

int state = 0;

float desired_current = 0;
float output = 0;
float battery_charge = 0.0001;

bool test_mode = false;

unsigned long last_state_change = 0;
float smoothed_current;

float current_measurement_resistor;

float min_battery_charge = 1.5;
float max_battery_charge = 8.4;

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CALIB_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  pinMode(CLK, INPUT);
  pinMode(DIO, INPUT);

  getVoltage();

  state = STATE_CHECK_CHARGE;

  smoothed_current = 0;

  analogWrite(BEEPER_PIN, 0);

  set_current_measurement_resistor();

  if (ONE_S_DRONE) {
    set_alarm(0);
    delay(2000);
    set_alarm(0);
    delay(500);
  }

  // setupPWM16();
}
uint16_t icr = 0xffff;
void setupPWM16() {
  DDRB |= _BV(PB1) | _BV(PB2);                  /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)            /* non-inverting PWM */
           | _BV(WGM11);                        /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); /* prescaler 1 */
  ICR1 = icr;                                   /* TOP counter value (freeing OCR1A*/
}

void analogWrite16(uint8_t pin, uint16_t val) {
  switch (pin) {
    case 9:
      OCR1A = val;
      break;
    case 10:
      OCR1B = val;
      break;
  }
}

void loop() {
  getVoltage();

  static long next_loop_time = 0;

  if (millis() > next_loop_time) {
    next_loop_time = millis() + run_state_machine();
  }

  set_test_mode();

  if (test_mode) {
    analogWrite(PWM_PIN, 0);
    set_alarm_test_mode();

    static long print_volt_timer = 0;
    if (millis() - print_volt_timer > 1000) {
      char str_volt[6];
      float voltage = getVoltage();

      dtostrf(getVoltage(), 4, 2, str_volt);
      debugln("voltage %s", str_volt);
      print_volt_timer = millis();
    }
  } else {
    set_current(desired_current);

    calcSmoothedCurrent();

    set_status_led();
  }

  /*
    static int divider = 0;

    divider++;
    TCCR0B = TCCR0B_orig & B11111000 | (divider / 10)%8;    // set timer 1 divisor to     8 for PWM frequency of 3921.16
    Hz
  */
  // analogWrite(PWM_PIN,  (millis() / 2000) % 2 * 255);
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

void set_current_measurement_resistor() {
  current_measurement_resistor = 0.33;
}

float smooth_value(float alpha, float value, float smoothed_value) {
  return alpha * value + (1.0 - alpha) * smoothed_value;
}

void set_test_mode(void) {
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
      test_mode = true;
      voltage_calibration_value = 0;
    }
  }

  static bool doOnce = true;
  if (doOnce) {
    for (byte i = 0; i < EEPROM_CALIB_DONE_STEPS; i++) {
      if (EEPROM.read(i + EEPROM_CALIB_DONE_START) != i) {
        test_mode = true;
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

  voltage_calibration_value += voltage - getVoltage();

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

static long last_detection_time;
static uint8_t drone_detected = false;
static unsigned long total_charging_timer;

void set_status_led(void) {
  static int old_beep_state = 0;
  enum BeepState { BEEP_CHARGING, BEEP_NO_CURRENT, BEEP_NO_DRONE, BEEP_NO_DRONE_ALARM, BEEP_DRONE_DETECTED };

  static unsigned long last_detect_time;
  static unsigned long last_charge_time;

  unsigned long time_since_last_detect = millis() - last_detect_time;
  unsigned long time_since_last_charge = millis() - last_charge_time;
  static unsigned long current_timeout = 0;

  static int beep_state = BEEP_NO_DRONE;

  // debugln("beep %d %d %d %d %d", int(time_since_last_detect), int(time_since_last_charge), int(total_charging_timer),
  // beep_state, state);

  /* debugln("%d", state); */
  /* debugln("%d", int( millis() - total_charging_timer) ); */
  /* debugln("beep state %d", beep_state ); */
  /* if (state == STATE_CHARGING && millis() - total_charging_timer < 6000) { */
  /* debugln("beep drone detected"); */
  /* beep_state = BEEP_DRONE_DETECTED; */
  /* } */

  if (state == STATE_CHARGING)
    beep_state = BEEP_CHARGING;
  /* else if (time_since_last_charge > 2000L && time_since_last_detect < 2000L) */
  else if (current_timeout > 2000L && time_since_last_detect < 2000L && battery_charge < max_battery_charge)
    beep_state = BEEP_NO_CURRENT;

  else if (time_since_last_charge < 2000L)
    beep_state = BEEP_CHARGING;

  else if (time_since_last_detect > 10000L)
    beep_state = BEEP_NO_DRONE_ALARM;

  else if (!drone_detected)
    beep_state = BEEP_NO_DRONE;

  if (state == STATE_CHARGING)
    last_detect_time = millis();

  /* debugln("battery_charge %d", (int)(1000*battery_charge)); */
  if (smoothed_current > 0.2 || battery_charge >= 8.35)
    last_charge_time = millis();
  /*
    if (state != STATE_CHARGING && millis() - last_detect_time > 1000)
      total_charging_time = millis();
    */

  static unsigned long old_time = 0;
  int dt = millis() - old_time;

  /* debugln("current_timeout %d ", current_timeout); */
  if (state == STATE_CHARGING && smoothed_current < 0.2 && battery_charge < max_battery_charge)
    current_timeout += dt;

  if (smoothed_current > 0.2)
    current_timeout = 0;

  old_time = millis();
  if (beep_state != old_beep_state) {
    reset_beep_time();
  }

  switch (beep_state) {
    case BEEP_CHARGING: {
      set_alarm_connected();
      break;
    }
    case BEEP_NO_CURRENT: {
      set_alarm_no_current();
      break;
    }
    case BEEP_NO_DRONE: {
      if (get_beep_time < 10000)
        set_alarm_disconnected();
      else
        set_alarm_no_drone();
      break;
    }
    case BEEP_NO_DRONE_ALARM: {
      set_alarm_no_drone();
      break;
    }
    case BEEP_DRONE_DETECTED: {
      /* set_alarm_half_on_super_fast(); */
      set_alarm_connected();
      break;
    }
  }
  old_beep_state = beep_state;
}

void set_shifter(uint8_t* shift_data, uint8_t* text, int len) {
  for (int i = 0; i < 40; i++)
    shift_data[i] = 0;

  for (int i = 0; i < len; i++)
    shift_data[i + 4] = text[i];
}

void run_display(void) {
  static float smoothed_voltage = 0;
  smoothed_voltage = smooth_value(0.5, getVoltage(), smoothed_voltage);

  static int disp_state, disp_output;
  static float disp_level, disp_voltage, disp_current;

  static long value_update_time = 0;
  if (millis() > value_update_time) {
    disp_output = output;
    disp_level = battery_charge;
    disp_voltage = getVoltage();

    if (ONE_S_DRONE) {
      disp_voltage /= 2;
      disp_level /= 2;
    }
    disp_current = smoothed_current;
    value_update_time = millis() + 200;
  }

  if (!test_mode) {
    static char buf[100];
    sprintf(buf, "disp: len, 65, ver, %03d, vlt, %03d, lvl, %03d, cur, %03d, csp, %03d, stt, %03d, out, %03d",
            SOFTWARE_VERSION, int(disp_voltage * 100), int(disp_level * 100), int(disp_current * 1000),
            int(desired_current * 1000), state, int(output));
    Serial.println(buf);
  }
}

void set_state(int new_state) {
  if (state != new_state) {
    debugln("switching form state %d to state %d", state, new_state);
    last_state_change = millis();
  }

  state = new_state;
}

float run_state_machine() {
  if (millis() > 0000)
    min_battery_charge = 3.2;

  static int no_current_timeout = 0;
  static uint8_t isCharging = false;

  static int last_output = 0;
  // debugln("state %d", state);
  switch (state) {
    case STATE_IDLE: {
      break;
    }
    case STATE_CHARGING: {
      if (battery_charge < 5)
        desired_current = 0.3;
      else if (battery_charge < 8)
        desired_current = 0.45;
      else if (battery_charge < 8.3)
        desired_current = 0.4;
      else if (battery_charge < 8.40)
        desired_current = 0.3;
      else if (battery_charge < 8.5)
        desired_current = 0.3;
      else
        desired_current = 0.0;

      float drone_current_consumption = 0.300;

      desired_current += drone_current_consumption;

      if (battery_charge > 8.4)
        desired_current = 0.0;

      if (smoothed_current > 0.25)
        isCharging = true;

      if (isCharging && smoothed_current < 0.01) {
        output = 0;
        desired_current = 0;
        set_current(desired_current);
        isCharging = false;
        debugln("no current");
        set_state(STATE_CHECK_CHARGE);
      }

      if (output > 254 && smoothed_current < 0.05)
        no_current_timeout++;
      else
        no_current_timeout = 0;

      if (battery_charge < min_battery_charge || millis() % 120000 < 300 ||
          (no_current_timeout > 1000 && millis() > 60000) || output > last_output + 50) {
        debugln("no charge %d  %d  %d  %d  %d", int(100 * battery_charge), int(100 * min_battery_charge),
                int(millis() % 120000 < 300), int(no_current_timeout), int(output > last_output + 50));

        set_state(STATE_CHECK_CHARGE);
        desired_current = 0;
        set_current(desired_current);

        return 300;
      }

      break;
    }
    case STATE_CHECK_CHARGE: {
      last_output = output;
      set_current(0);
      // output = 0;
      battery_charge = getVoltage();

      if (battery_charge == 0)
        battery_charge = 0.01;

      if (battery_charge < min_battery_charge) {
        drone_detected = false;
        total_charging_timer = millis();
        /* debugln("no drone detected"); */
        return 1;
      }

      drone_detected = true;

      set_state(STATE_CHARGING_PRECHECK);
      return 2000;

      break;
    }
    case STATE_CHARGING_PRECHECK: {
      desired_current = 0;
      set_current(desired_current);

      battery_charge = getVoltage();

      if (battery_charge < min_battery_charge)
        set_state(STATE_CHECK_CHARGE);

      set_state(STATE_CHARGING);

      no_current_timeout = 0;

      break;
    }
  }

  return 1;
}

float getVoltage(bool reset) {
  static float smoothed_voltage = 0;

  if (reset)
    smoothed_voltage = 0;

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

  if (ONE_S_DRONE && !test_mode)
    voltage *= 2;

  voltage *= 2.02;
  voltage += voltage_calibration_value;

  float alpha = 0.1;
  if (test_mode)
    alpha /= 5;

  smoothed_voltage = (1 - alpha) * smoothed_voltage + alpha * voltage;

  return smoothed_voltage;
}

float getCurrent() {
  float voltage = analogRead(CURRENT_PIN) / 1024.0 * 5.0;
  voltage *= 1.3;
  return voltage / current_measurement_resistor;
}

void calcSmoothedCurrent() {
  smoothed_current = smooth_value(0.01, getCurrent(), smoothed_current);
}

void set_current(float current_setpoint) {
  float error = (smoothed_current - current_setpoint);
  output -= error * 1.0;
  if (smoothed_current > 0.9)
    output -= error * 10;

  if (output > 255)
    output = 255;
  if (output < 0)
    output = 0;

  int o = (int)output;

  if (current_setpoint == 0) {
    o = 0;
    // ssmoothed_current = 0;
  }

  static int test = 1;
  test = 1 - test;

  analogWrite(PWM_PIN, o);
}
