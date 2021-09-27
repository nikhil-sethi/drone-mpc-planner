#pragma once

#include <WString.h>

#define LOG_LINE_VERSION 003

#define debugln(msg, ...)                          \
  {                                                \
    char debug_buf[64];                            \
    sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); \
    Serial.write(debug_buf);                       \
  }

float moving_average(float alpha, float value, float smoothed_value);
bool match_command(char* input, const char* command);
void append_log_line(char* log_line, const char* name, int val, bool add_comma = true);
void reverse_strcat(char* src, char* dst);
bool serial_input(char* buf, int size);