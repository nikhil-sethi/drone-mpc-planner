#pragma once

#include <WString.h>

#define VERSION 2
#define PATS_HEADER 'P'
struct SerialPackage {
    const char header = 'P';
    uint16_t version = VERSION;
    uint8_t led_state;
    uint8_t watchdog_state;
    uint32_t up_duration; // uint32_t = unsigned long in arduino
    uint8_t charging_state;
    float battery_volts;
    float charging_volts;
    float charging_amps;
    float setpoint_amp;
    float mah_charged;
    float charge_resistance;
    float drone_amps_burn;
    unsigned char charging_pwm;
    uint32_t charging_duration;
    const char ender = '\n';
};

#define debugln(msg, ...)                          \
  {                                                \
    char debug_buf[64];                            \
    sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); \
    Serial.write(debug_buf);                       \
  }

float moving_average(float alpha, float value, float smoothed_value);
bool match_command(char *input, const char *command);
void append_log_line(char *log_line, const char *name, int val, bool add_comma = true);
void reverse_strcat(char *src, char *dst);
bool serial_input(char *buf, int size);
