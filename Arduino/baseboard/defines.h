#pragma once
#define CHARGING_AMPS_PIN A0
#define HARDWARE_VERSION_PIN A1
#define CHARGING_BATT_VOLT_PIN A2
#define FAN_RPM_PIN A3
#define CHARGING_VOLT_PIN A4
#define CHARGING_GND_PIN A5
#define LIGHT_SENSOR_PIN A6
#define RGB_LEDS_PIN 2
#define CHARGING_OVERLOAD_INTERRUPT_PIN 3
#define FAN_PWM_OUT 4
#define USB_ENABLE_PIN 5
#define BUTTON_PIN 6
#define IR_LED_ENABLE_PIN 7
#define NUC_ENABLE_PIN 8
#define CHARGING_ENABLE_PIN 9
#define CHARGING_PWM_PIN 10
#define LED_FLASH_PIN 10

#define SECOND_IN_MILLIS 1000L
#define MINUTE_IN_MILLIS (60L * SECOND_IN_MILLIS)
#define HOUR_IN_MILLIS (60L * MINUTE_IN_MILLIS)
#define DAY_IN_MILLIS (24L * HOUR_IN_MILLIS)
#define NUC_OFF_TIME (30L * SECOND_IN_MILLIS)
#define WATCHDOG_TIMEOUT HOUR_IN_MILLIS

#define LOOP_TIME_MS 15
#define LOOP_FREQ 66 // --> 1/LOOP_TIME_MS