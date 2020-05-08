#include <Wire.h>
#include <Adafruit_GFX.h>
#include <TM1637Display.h>
#include "characters.h"

#define debugln(msg, ...)  {char debug_buf[64]; sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}

#define PWM_PIN 11
#define VOLTAGE_PIN A2
#define CALIB_PIN A7
#define LED_PIN 6

#define CLK A4
#define DIO A5

#define SOFTWARE_VERSION 0.3

float getVoltage(bool reset = false);

enum State { STATE_IDLE, STATE_CHARGING, STATE_CHECK_CHARGE, STATE_FULL, STATE_TESTING, STATE_CHARGING_PRECHECK };

TM1637Display display(CLK, DIO);

int state = 0;

float desired_current = 0;
float output = 0;
float battery_charge = 0.0001;

bool test_mode = false;
bool turbo_mode = false;

float smoothed_current;

enum DisplayStates {  S_VERSION_INIT,  S_VERSION,  S_TURBO_INIT,  S_TURBO,  S_VOLTAGE,
                      S_CURRENT,  S_LEVEL,  S_OUTPUT,  S_STATE,  S_HELP
                   };

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);


  pinMode(CLK, INPUT);
  pinMode(DIO, INPUT);

  if (analogRead(CLK) == 0 && analogRead(DIO) == 1023)
    turbo_mode = true;

  display.setBrightness(0x0f);

  getVoltage();

  state = STATE_CHECK_CHARGE;

  smoothed_current = 0;

  display = TM1637Display(CLK, DIO);
}

void loop() {
  calcSmoothedCurrent();

  set_test_mode();

  set_status_led();

  getVoltage();

  static long next_loop_time = 0;

  if (millis() > next_loop_time)
  {
    next_loop_time = millis() + run_state_machine();
  }

  if (test_mode)
    analogWrite(PWM_PIN,  0);
  //else
  //analogWrite(PWM_PIN,  (millis() / 2000) % 2 * 255);


  set_current(desired_current);
  /*
    debugln("%d", (millis() / 2000) % 2 * 255);
  */
  run_display();

  static unsigned long timer = millis();
  int wait_time = 10;
  wait_time = min(wait_time, timer + wait_time - millis());
  timer = millis();
}

float smooth_value(float alpha, float value, float smoothed_value)
{
  return alpha * value + (1.0 - alpha) * smoothed_value;
}

void set_test_mode(void)
{
  static bool isHigh = false;
  static int count = 0;

  if (analogRead(CALIB_PIN) > 800 && isHigh == false)
  {
    count++;
    isHigh = true;
  }

  if (analogRead(CALIB_PIN) < 200 && isHigh == true)
  {
    count++;
    isHigh = false;
  }

  if (count >= 2)
  {
    test_mode = true;
  }
}

void set_status_led(void)
{
  int d = 1 + 2 * turbo_mode;

  switch (state)
  {
    case STATE_IDLE:
      {
        if (millis() % 100 > 50)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);

        break;
      }
    case STATE_CHARGING:
      {
        if (battery_charge < 8.4)
        {
          if (millis() % (3000 / d) > 1500 / d)
            digitalWrite(LED_PIN, 1);
          else
            digitalWrite(LED_PIN, 0);
        }
        else
        {
          if (millis() % (3000 / d) > 200 / d)
            digitalWrite(LED_PIN, 1);
          else
            digitalWrite(LED_PIN, 0);
        }
        break;
      }
    case STATE_CHECK_CHARGE:
      {
        if (millis() % (3000 / d) > 2920 / d)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
        break;
      }
    case STATE_CHARGING_PRECHECK:
      {
        if (millis() % (600 / d) > 300 / d)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
        break;
      }
    default:
      {
        if (millis() % 100 > 50)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
        break;
      }
  }
}


void set_val(uint8_t header, int val, uint8_t* data)
{
  data[3] = num[0];

  for (int i = 0; i < 4; i++)
  {
    data[3 - i] = num[val % 10];
    if (val < 10)
      break;
    val /= 10;
  }

  if (header > 0)
    data[0] = header;
}

void set_val(uint8_t header, float val, int precision, uint8_t* data)
{
  int ival = val * pow(10, precision);

  if (precision >= 0 && precision < 4)
    for (int i = 0; i < precision + 1; i++)
      data[3 - i] |= N_0;

  set_val(header, ival, data);

  if (precision >= 0 && precision < 4)
    data[3 - precision] |= SEG_DP;
}

void get_version_text(uint8_t* data)
{
  data[0] = N_v;
  data[1] = N_E;
  data[2] = N_r;
  data[3] = N_s;
  data[4] = N_i;
  data[5] = N_o;
  data[6] = N_n;
  data[7] = 0;

  uint8_t vers[4] = {};
  set_val(N_v, SOFTWARE_VERSION, 1, vers);

  data[8] = vers[0];
  data[9] = vers[1];
  data[10] = vers[2];
  data[11] = vers[3];
}

void get_turbo_text(uint8_t* data)
{
  data[0] = N_t;
  data[1] = N_u;
  data[2] = N_r;
  data[3] = N_b;
  data[4] = N_o;
  data[5] = N_o;
  data[6] = N_o;
  data[7] = N_0;
  data[8] = N_o;
  data[9] = N_o;
  data[10] = N_0;
  data[11] = N_o;
  data[12] = N_o;
  data[13] = N_0;
  data[14] = N_0;
  data[15] = N_o;
  data[16] = N_o;
  data[17] = N_o;
  data[18] = N_o;
}

void set_shifter(uint8_t* shift_data, uint8_t* text, int len)
{
  for (int i = 0; i < 40; i++)
    shift_data[i] = 0;

  for (int i = 0; i < len; i++)
    shift_data[i + 4] = text[i];
}

void run_display(void)
{
  static float smoothed_voltage = 0;
  smoothed_voltage = smooth_value(0.5, getVoltage(), smoothed_voltage);

  uint8_t data[] = { 0, 0, 0, 0 };

  static uint8_t shift[40] = { };

  static unsigned long timer = millis();

  static unsigned long next_shift = 500;

  static uint8_t state_disp = S_VERSION_INIT;

  int header_time = 200;
  int transition_time = 3000;
  int total_time = 6 * transition_time;

  int disp_state, disp_output;
  float disp_level, disp_voltage, disp_current;

  disp_state = state;

  static long value_update_time = 0;
  if (millis() > value_update_time)
  {
    disp_output = output;
    disp_level = battery_charge;
    disp_voltage = getVoltage();
    disp_current = getCurrent();
    value_update_time = millis() + 200;
  }

  static int shift_count = 0;

  switch (state_disp)
  {
    case S_TURBO_INIT:
      {
        uint8_t turbo_text[21] = {};
        get_turbo_text(turbo_text);
        set_shifter(shift, turbo_text, 21);
        state_disp = S_TURBO;
        timer = millis();
        shift_count = 0;
        next_shift = millis() + 300;

        break;
      }
    case S_VERSION_INIT:
      {
        uint8_t version_text[12] = {};
        get_version_text(version_text);
        set_shifter(shift, version_text, 12);
        state_disp = S_VERSION;
        timer = millis();
        shift_count = 0;
        next_shift = millis() + 300;

        break;
      }
    case S_TURBO:
      {
        if (millis() > next_shift)
        {
          next_shift = millis() + 50;

          for (int i = 0; i < 28; i++)
            shift[i] = shift[i + 1];

          shift_count++;
        }

        for (int i = 0; i < 4; i++)
          data[i] = shift[i];

        display.setSegments(shift, 4, 0);

        if (shift_count > 28)
        {
          state_disp = S_VOLTAGE;
          timer = millis();
        }
        break;
      }
    case S_VERSION:
      {
        if (millis() > next_shift)
        {
          next_shift = millis() + 100;

          if (shift_count < 12)
            for (int i = 0; i < 28; i++)
              shift[i] = shift[i + 1];

          shift_count++;
        }

        for (int i = 0; i < 4; i++)
          data[i] = shift[i];

        display.setSegments(shift, 4, 0);

        if (shift_count > 32)
        {
          state_disp = S_HELP;
          timer = millis();
        }
        break;
      }
    case S_HELP:
      {
        int period = 1500;

        if (millis() - timer  > 5 * period)
        {
          state_disp = S_VOLTAGE;
          timer = millis();
        }

        if (millis() - timer  < 1 * period)
        {
          data[0] = N_V;
          data[1] = N_o;
          data[2] = N_L;
          data[3] = N_t;
        }
        else if (millis() - timer  < 2 * period)
        {
          data[0] = N_L;
          data[1] = N_E;
          data[2] = N_v;
          data[3] = N_E;
        }
        else if (millis() - timer  < 3 * period)
        {
          data[0] = N_C;
          data[1] = N_u;
          data[2] = N_r;
          data[3] = N_r;
        }
        else if (millis() - timer  < 4 * period)
        {
          data[0] = N_o;
          data[1] = N_u;
          data[2] = N_t;
          data[3] = N_p;
        }
        else if (millis() - timer  < 5 * period)
        {
          data[0] = N_S;
          data[1] = N_t;
          data[2] = N_A;
          data[3] = N_t;
        }

        break;
      }
    case S_VOLTAGE:
      {
        if (millis() - timer  > transition_time)
        {
          state_disp = S_LEVEL;
          timer = millis();
        }

        if (test_mode)
        {
          set_val(0, disp_voltage, 4,  data);
        }
        else
        {
          set_val(N_U, disp_voltage, 2, data);
        }
        break;
      }
    case S_LEVEL:
      {
        if (millis() - timer  > transition_time)
        {
          state_disp = S_CURRENT;
          timer = millis();
        }
        set_val(N_L, disp_level, 2, data);
        break;
      }
    case S_CURRENT:
      {
        if (millis() - timer  > transition_time)
        {
          state_disp = S_OUTPUT;
          timer = millis();
        }

        set_val(N_C, disp_current, 3,  data);
        break;
      }
    case S_OUTPUT:
      {
        if (millis() - timer  > transition_time)
        {
          state_disp = S_STATE;
          timer = millis();
        }
        set_val(N_o, disp_output, data);
        break;
      }
    case S_STATE:
      {
        if (millis() - timer  > transition_time)
        {
          state_disp = S_VOLTAGE;
          if (turbo_mode)
            state_disp = S_TURBO_INIT;
          timer = millis();
        }

        set_val(N_S, disp_state, data);
        break;
      }
  }

  if (turbo_mode && millis() % 100 > 50)
    data[3] |= SEG_DP;

  display.setSegments(data, 4, 0);

  if (millis() < timer)
    timer = millis();
}

void set_state(int new_state)
{
  if (state != new_state)
    debugln("switching form state %d to state %d", state, new_state);

  state = new_state;
}

float run_state_machine()
{
  static int no_current_timeout = 0;

  switch (state)
  {
    case STATE_IDLE:
      {
        break;
      }
    case STATE_CHARGING:
      {
        if (battery_charge < 5)
          desired_current = 0.2;
        else if (battery_charge < 8)
          desired_current = 0.45;
        else if (battery_charge < 8.3)
          desired_current = 0.4;
        else if (battery_charge < 8.45)
          desired_current = 0.2;
        else if (battery_charge < 8.5)
          desired_current = 0.0;
        else
          desired_current = 0.0;

        float drone_current_consumption = 0.200;

        desired_current += drone_current_consumption;

        if (turbo_mode)
          desired_current += 0.20;

        if (battery_charge > 8.5)
          desired_current = 0.0;

        if (output > 254 && smoothed_current < 0.01)
          no_current_timeout++;
        else
          no_current_timeout = 0;

        if (battery_charge < 3.2 || millis() % 120000 < 1000 || no_current_timeout > 1000)
        {
          set_state(STATE_CHECK_CHARGE);
          desired_current = 0;
          return 300;
        }

        break;
      }
    case STATE_CHECK_CHARGE:
      {
        desired_current = 0;

        battery_charge = getVoltage();
        if (battery_charge == 0)
          battery_charge = 0.01;

        if (battery_charge < 3.2)
          return 1;

        set_state(STATE_CHARGING_PRECHECK);
        return 300;

        break;
      }
    case STATE_CHARGING_PRECHECK:
      {
        desired_current = 0;

        battery_charge = getVoltage();

        if (battery_charge < 3.2)
          set_state(STATE_CHECK_CHARGE);

        set_state(STATE_CHARGING);

        no_current_timeout = 0;

        break;
      }
  }

  return 1;
}

float getVoltage(bool reset)
{
  static float smoothed_voltage = 0;

  if (reset)
    smoothed_voltage = 0;

  float voltage = (analogRead(VOLTAGE_PIN) + 0.5) * 5.0 / 1024.0;
  voltage *= 2.05;

  static float calib_level = 0;
  calib_level = 0.99 * calib_level + 0.01 * float(analogRead(CALIB_PIN));
  float calib = (calib_level + 0.5) / 1024.0;
  calib -= 0.5;
  calib *= 0.1;
  voltage += calib;

  smoothed_voltage = 0.9 * smoothed_voltage + 0.1 * voltage;

  return smoothed_voltage;
}

float getCurrent()
{
  float voltage = analogRead(A0) / 1024.0 * 5.0;
  return voltage / 3.2;
}

void calcSmoothedCurrent()
{
  smoothed_current = smooth_value(0.01, getCurrent(), smoothed_current);
}

void set_current(float current_setpoint) {
  float error = (smoothed_current - current_setpoint);
  output -= error * 1.0;

  debugln("%d %d %d %d %d %d", int(smoothed_current * 100), int(100 * current_setpoint), -int(100 * error), (int)(output / 2.55), (int)(getVoltage() * 100), (int)(battery_charge * 100));

  if (current_setpoint == 0)
    output = 0;

  if (output > 255)
    output = 255;
  if (output < 0)
    output = 0;

  static int test = 1;
  test = 1 - test;

  analogWrite(PWM_PIN,  (int)output);
}
