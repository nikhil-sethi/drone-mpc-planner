   
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "TM1637Display.h"

#define debugln(msg, ...)  {char debug_buf[64]; sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}

#define PWM_PIN 11
#define VOLTAGE_PIN A2
#define CALIB_PIN A7
#define LED_PIN 6

#define CLK A4
#define DIO A5

#define SOFTWARE_VERSION 0.2

float getVoltage(bool reset = false);

const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};

enum State { STATE_IDLE, STATE_CHARGING, STATE_CHECK_CHARGE, STATE_FULL, STATE_TESTING, STATE_CHARGING_PRECHECK };

TM1637Display display(CLK, DIO);

int state = 0;

float desired_current = 0;
float output = 0;
float battery_charge = 0.0001;

bool test_mode = false;

float smoothed_current;

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);

  display.setBrightness(0x0f);

  getVoltage();

  state = STATE_CHECK_CHARGE;

  smoothed_current = 0;
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
    set_current(0);
  else
    set_current(desired_current);
    
  static long next_display_time = 0;

  if (millis() > next_display_time)
  {
    run_display();
    next_display_time = millis() + 100;
  }
  
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
        if (millis() % 1500 > 750)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
      }
      else
      {
        if (millis() % 1500 > 100)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
      }
       break;
    }
    case STATE_CHECK_CHARGE:
    {
        if (millis() % 1000 > 930)
          digitalWrite(LED_PIN, 1);
        else
          digitalWrite(LED_PIN, 0);
       break;
    }
    case STATE_CHARGING_PRECHECK:
    {
        if (millis() % 300 > 150)
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

void run_display(void)
{
  static long next_display_time = 0;
  static float smoothed_voltage = 0;
  smoothed_voltage = smooth_value(0.5, getVoltage(), smoothed_voltage);

  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00};

  static uint8_t vers[18] = { };

                        
  if (millis() < 100)
  {
    vers[4] = SEG_C | SEG_D | SEG_E;
    vers[5] = SEG_A | SEG_D | SEG_E | SEG_F | SEG_G;
    vers[6] = SEG_E | SEG_G;
    vers[7] = SEG_C | SEG_D;  
    vers[8] = SEG_C ;
    vers[9] = SEG_C | SEG_D | SEG_E | SEG_G;
    vers[10] = SEG_C | SEG_E | SEG_G;
  }
  if (millis() < 5000)
  {
    static int next_shift = 400;
    if (millis() > next_shift)
    {
      next_shift += 400;
      
      for (int i = 0; i < 17; i++) 
          vers[i] = vers[i + 1]; 
    }
    
    display.setSegments(vers, 4 , 0);
  }
  else if (millis() < 8000)
  {
    display.setDotPos(2);
    display.showNumberDec(SOFTWARE_VERSION * 10, false); 
    
    data[0] = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_DP ;
    display.setSegments(data, 1, 2);
  }
  else
  {
    next_display_time = millis() + 200;

    if (millis() % 7500 < 1500 || test_mode)
    {
      if (test_mode)
      {     
        display.setDotPos(0);
        display.showNumberDec(getVoltage() * 1000, false); 
      }
      else
      {      
        display.setDotPos(1);

        display.showNumberDec(getVoltage() * 100, false); 

        data[0] = SEG_B | SEG_C | SEG_D | SEG_E | SEG_F ;
        display.setSegments(data, 1, 0);
      }
    }
    else if (millis() % 7500 < 3000)
    {
      
      display.setDotPos(1);

      display.showNumberDec(battery_charge * 100, false);

      data[0] = SEG_D | SEG_E | SEG_F;
      display.setSegments(data, 1, 0);

    }
    else   if (millis() % 7500 < 4500)
    {
      display.setDotPos(0);

      display.showNumberDec(smoothed_current * 1000, false); 

      data[0] = SEG_A | SEG_D | SEG_E | SEG_F;
      display.setSegments(data, 1, 0);
    }
    else   if (millis() % 7500 < 6000)
    {
      display.setDotPos(-1);

      display.showNumberDec(output, false);

      data[0] = SEG_C | SEG_D | SEG_E | SEG_G;
      display.setSegments(data, 1, 0);
    }
    else   if (millis() % 7500 < 7500)
    {
      display.setDotPos(-1);
      display.showNumberDec(state, false); 

      data[0] = SEG_A | SEG_B | SEG_D | SEG_E ;
      display.setSegments(data, 1, 0);
    }
  }
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
        else if (battery_charge < 8.1)
          desired_current = 0.4;
        else if (battery_charge < 8.2)
          desired_current = 0.3;
        else if (battery_charge < 8.3)
          desired_current = 0.0;
        else
          desired_current = 0.0;

        float drone_current_consumption = 0.200;

        desired_current += drone_current_consumption;

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
          return 3000;
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
        return 3000;

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
  output -= error * 0.1;

  debugln("%d %d %d %d", int(smoothed_current*1000), int(1000*current_setpoint), int(1000*error), (int)(output));

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
