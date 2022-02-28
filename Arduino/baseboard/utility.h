#pragma once

#include <WString.h>

#define MAX_PACKAGE_READ_SIZE 16

#define FIRMWARE_VERSION 8
#define BASEBOARD_PACKAGE_PRE_HEADER '@'
enum baseboard_package_headers {
    header_SerialBaseboard2NUCPackage = 'P',
    header_SerialNUC2BaseboardChargingPackage = 'C',
    header_SerialNUC2BaseboardLedPowerPackage = 'L',
    header_SerialNUC2BaseboardWatchdogPackage = 'W',
    header_SerialNUC2BaseboardFanPackage = 'F',
    header_SerialNUC2BaseboardNUCResetPackage = 'N',
    header_SerialNUC2BaseboardEEPROMPackage = 'E',
    header_SerialNUC2BaseboardRGBLEDPackage = 'R',
    header_SerialExecutor2BaseboardAllowChargingPackage = 'A',
    header_SerialExecutor2BaseboardStatePackage = 'S'

};

//copy from utility.h Arduino code
struct __attribute__((packed)) SerialBaseboard2NUCPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialBaseboard2NUCPackage;
    uint16_t hardware_version;
    uint16_t baseboard_boot_count;
    uint16_t watchdog_boot_count;
    uint32_t up_duration; // uint32_t = unsigned long in arduino
    uint8_t ir_led_state;
    uint8_t watchdog_state;
    uint8_t charging_state;
    float battery_volts;
    float charging_volts;
    float charging_amps;
    float setpoint_amps;
    float mah_charged;
    float charge_resistance;
    float drone_amps_burn;
    unsigned char charging_pwm;
    uint32_t charging_duration;
    uint16_t measured_fan_speed;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardEEPROMPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardEEPROMPackage;
    uint8_t clear_config_all;
    uint8_t clear_config_hard;
    uint8_t clear_config_log;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardChargingPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardChargingPackage;
    uint8_t enable_charging;
    uint8_t calibrate;
    float volts;
    uint8_t reset_calibration;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialExecutor2BaseboardAllowChargingPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialExecutor2BaseboardAllowChargingPackage;
    uint8_t allow_charging;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialExecutor2BaseboardStatePackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialExecutor2BaseboardStatePackage;
    uint8_t executor_state;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardLedPowerPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardLedPowerPackage;
    uint8_t led_power;
    const char ender = '\n';
};
struct __attribute__((packed)) SerialNUC2BaseboardWatchdogPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardWatchdogPackage;
    uint8_t watchdog_enabled;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardFanPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardFanPackage;
    uint8_t fan_pwm;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardNUCResetPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardNUCResetPackage;
    const char ender = '\n';
};

struct __attribute__((packed)) SerialNUC2BaseboardRGBLEDPackage {
    const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
    const uint16_t firmware_version = FIRMWARE_VERSION;
    const char header = header_SerialNUC2BaseboardRGBLEDPackage;
    unsigned char led1_state;
    bool internet_OK;
    bool daemon_OK;
    bool postprocessing;
    const char ender = '\n';
};

#define debugln(msg, ...)                          \
  {                                                \
    char debug_buf[64];                            \
    sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); \
    Serial.write(debug_buf);                       \
  }


float moving_average(float alpha, float value, float smoothed_value);
uint16_t serial_read_to_buf(unsigned char buf[MAX_PACKAGE_READ_SIZE]);
