#include "utility.h"
#include "charger.h"
#include "eeprom_settings.h"
#include <FastLED.h>

#define INDICATOR_LEDS_PIN 2
#define NUM_INDICATOR_LEDS 2
CRGB leds[NUM_INDICATOR_LEDS];

#define NUC_ENABLE_PIN 8
#define FAN_RPM_PIN 3

#define SECOND_IN_MILLIS 1000L
#define MINUTE_IN_MILLIS (60L * SECOND_IN_MILLIS)
#define HOUR_IN_MILLIS (60L * MINUTE_IN_MILLIS)
#define DAY_IN_MILLIS (24L * HOUR_IN_MILLIS)
#define NUC_OFF_TIME (30L * SECOND_IN_MILLIS)
#define WATCHDOG_TIMEOUT HOUR_IN_MILLIS
#define WATCHDOG_STATUS_LED 13
#define HARDWARE_VERSION_PIN A1
#define BUTTON_1_PIN 4
#define BUTTON_2_PIN 6
#define USB_ENABLE_PIN 5

Charger charger;

bool reset_nuc = false;
bool watchdog_enabled = false;
bool watchdog_trigger_imminent = false;
bool nuc_has_been_reset = false;
long last_nuc_reset = 0L;
unsigned long nuc_watchdog_timer = millis();
uint8_t watchdog_reset_count = 0;
uint16_t watchdog_boot_count = 0;
uint16_t baseboard_boot_count = 0;
uint16_t measured_fan_speed = 0;

unsigned char serial_input_buffer[MAX_PACKAGE_READ_SIZE] = {0};

void setup() {
    init_values_from_eeprom();
    write_baseboard_boot_count_eeprom(++baseboard_boot_count);

    // set pin high before and after setting pinmode to prevent NUC from resetting
    digitalWrite(NUC_ENABLE_PIN, 1);
    pinMode(NUC_ENABLE_PIN, OUTPUT);
    digitalWrite(NUC_ENABLE_PIN, 1);

    digitalWrite(USB_ENABLE_PIN, 1);
    pinMode(USB_ENABLE_PIN, OUTPUT);
    digitalWrite(USB_ENABLE_PIN, 1);

    Serial.begin(115200);
    Serial.setTimeout(5);

    pinMode(WATCHDOG_STATUS_LED, OUTPUT);
    digitalWrite(WATCHDOG_STATUS_LED, watchdog_enabled);

    pinMode(LED_ENABLE_PIN, OUTPUT);
    digitalWrite(LED_ENABLE_PIN, 1);

    pinMode(FAN_RPM_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT);
    pinMode(BUTTON_2_PIN, INPUT);
    pinMode(HARDWARE_VERSION_PIN, INPUT);

    charger.init();

    FastLED.addLeds<WS2812, INDICATOR_LEDS_PIN, GRB>(leds, NUM_INDICATOR_LEDS);

    set_indicator_leds({0, 0, 100 }, {0, 100, 0});
    hardware_version();
    //restart_usb()
    read_buttons();

    debugln("Baseboard started!");
}

void init_values_from_eeprom() {
    read_config_from_eeprom();
    watchdog_enabled = watchdog_state_eeprom();
    last_nuc_reset = millis() - watchdog_time_eeprom();
    watchdog_reset_count = watchdog_count_eeprom();
    watchdog_boot_count = watchdog_boot_count_eeprom();
    baseboard_boot_count = baseboard_boot_count_eeprom();
}

void apply_loop_delay(int loop_time) {
    static unsigned long timer = millis();
    int wait_time = min(loop_time, timer + loop_time - millis());
    timer = millis();
    if (wait_time > 0)
        delay(min(wait_time, 1));
}

void handle_serial_input() {
    auto n = serial_read_to_buf(serial_input_buffer);
    if (n > 2) {
        if (serial_input_buffer[0] == PACKAGE_PRE_HEADER && serial_input_buffer[1] == VERSION) {
            switch (serial_input_buffer[3])
            {
                case header_SerialNUC2BaseboardChargingPackage: {
                        SerialNUC2BaseboardChargingPackage *pkg = reinterpret_cast<SerialNUC2BaseboardChargingPackage * >(&serial_input_buffer);
                        charger.handle_serial_input_package(pkg);
                        break;
                } case header_SerialNUC2BaseboardLedPowerPackage: {
                        SerialNUC2BaseboardLedPowerPackage *pkg = reinterpret_cast<SerialNUC2BaseboardLedPowerPackage * >(&serial_input_buffer);
                        if (pkg->led_power) {
                            debugln("Enable LED");
                            digitalWrite(LED_ENABLE_PIN, 1);
                        } else {
                            debugln("Disable LED");
                            digitalWrite(LED_ENABLE_PIN, 0);
                        }
                        break;
                } case header_SerialNUC2BaseboardWatchdogPackage: {
                        SerialNUC2BaseboardWatchdogPackage *pkg = reinterpret_cast<SerialNUC2BaseboardWatchdogPackage * >(&serial_input_buffer);
                        nuc_watchdog_timer = millis();
                        watchdog_trigger_imminent = false;
                        nuc_has_been_reset = false;
                        write_nuc_has_been_reset_eeprom(nuc_has_been_reset);

                        if (pkg->watchdog_enabled && !watchdog_enabled) {
                            digitalWrite(WATCHDOG_STATUS_LED, 1);
                            write_watchdog_state_eeprom(watchdog_enabled);
                            watchdog_enabled = true;
                            debugln("Watchdog enabled");
                        } else if (!pkg->watchdog_enabled && watchdog_enabled) {
                            digitalWrite(WATCHDOG_STATUS_LED, 0);
                            write_watchdog_state_eeprom(watchdog_enabled);
                            watchdog_enabled = false;
                            debugln("Watchdog disabled");
                        }
                        break;
                } case header_SerialNUC2BaseboardFanPackage: {
                        SerialNUC2BaseboardFanPackage *pkg = reinterpret_cast<SerialNUC2BaseboardFanPackage * >(&serial_input_buffer);
                        debugln("Warning fan package not yet implemented");
                        break;
                } case header_SerialNUC2BaseboardNUCResetPackage: {
                        SerialNUC2BaseboardNUCResetPackage *pkg = reinterpret_cast<SerialNUC2BaseboardNUCResetPackage * >(&serial_input_buffer);
                        debugln("Reboot NUC");
                        reset_nuc = true;
                        break;
                } case header_SerialNUC2BaseboardEEPROMPackage: {
                        SerialNUC2BaseboardEEPROMPackage *pkg = reinterpret_cast<SerialNUC2BaseboardEEPROMPackage *>(&serial_input_buffer);
                        if (pkg->clear_config_all) {
                            debugln("clear config all");
                            clear_eeprom_all();
                            init_values_from_eeprom();
                        }
                        if (pkg->clear_config_log) {
                            debugln("clear config log");
                            clear_eeprom_exept_calib();
                            init_values_from_eeprom();
                        }
                        if (pkg->clear_config_hard) {
                            debugln("clear config hard");
                            clear_eeprom_hard();
                            init_values_from_eeprom();
                        }
                        break;
                } case header_SerialExecutor2BaseboardAllowChargingPackage: {
                        SerialExecutor2BaseboardAllowChargingPackage *pkg = reinterpret_cast<SerialExecutor2BaseboardAllowChargingPackage * >(&serial_input_buffer);
                        charger.handle_serial_input_package(pkg);
                        break;
                } default:
                    break;
            }
        }
    }
}

void loop() {
    apply_loop_delay(15);

    handle_serial_input();
    handle_watchdog();
    handle_nuc_reset();
    set_watchdog_indicator_led();
    // measured_fan_speed = pulseInLong(FAN_RPM_PIN, LOW);

    charger.run();
    if (!charger.calibrating())
        write_serial();
}

void write_serial() {
    static long value_update_time = 0;
    if (millis() - value_update_time < 500L)
        return;
    value_update_time = millis();

    uint8_t deci_hours_since_reset = (value_update_time - last_nuc_reset) * 10 / HOUR_IN_MILLIS;
    write_watchdog_time_eeprom(value_update_time - last_nuc_reset);

    SerialBaseboard2NUCPackage pkg;
    pkg.led_state = digitalRead(LED_ENABLE_PIN);
    pkg.watchdog_state = watchdog_enabled;
    pkg.up_duration = millis();
    pkg.measured_fan_speed = measured_fan_speed;
    charger.fill_serial_output_package(&pkg);
    Serial.write((char *)&pkg, sizeof(SerialBaseboard2NUCPackage));

    if (watchdog_trigger_imminent)
        debugln("Watchdog will trigger in %d seconds!", (int)((WATCHDOG_TIMEOUT + nuc_watchdog_timer - millis()) / 1000));
}

void handle_nuc_reset() {
    static unsigned long nuc_off_timer = millis();

    if (digitalRead(NUC_ENABLE_PIN) && reset_nuc) {
        debugln("Turn off NUC");
        reset_nuc = false;
        nuc_has_been_reset = true;
        write_nuc_has_been_reset_eeprom(nuc_has_been_reset);
        watchdog_trigger_imminent = false;
        nuc_off_timer = millis();
        last_nuc_reset = millis();
        nuc_watchdog_timer = millis();
        watchdog_reset_count++;
        write_watchdog_count_eeprom(watchdog_reset_count);
        write_watchdog_time_eeprom(millis() - last_nuc_reset);
        digitalWrite(NUC_ENABLE_PIN, 0);
    }

    if (millis() - nuc_off_timer > NUC_OFF_TIME && !digitalRead(NUC_ENABLE_PIN)) {
        debugln("Turn on NUC");
        watchdog_boot_count++;
        write_watchdog_boot_count_eeprom(watchdog_boot_count);
        digitalWrite(NUC_ENABLE_PIN, 1);
    }
}

void handle_watchdog() {
    if (!watchdog_enabled)
        return;
    else if (millis() - last_nuc_reset < 8 * HOUR_IN_MILLIS && nuc_has_been_reset)
        return;
    else if (millis() - nuc_watchdog_timer > WATCHDOG_TIMEOUT)
        reset_nuc = true;
    else if (millis() - nuc_watchdog_timer > WATCHDOG_TIMEOUT - 10000L)
        watchdog_trigger_imminent = true;
}

void hardware_version() {
    uint16_t analog_value = analogRead(A1);
    delay(10);
    analog_value = analogRead(A1);
    analog_value = ~analog_value;
    analog_value = analog_value >> 4;
    analog_value &= 0b11111;
    Serial.print("Hardware version ");
    Serial.println(analog_value);
}

void set_indicator_leds(CRGB led0, CRGB led1) {
    leds[0] = led0;
    leds[1] = led1;
    FastLED.show();
}

void read_buttons() {
    // button 2 does not work in hardware version 1
    Serial.print("button 1 is ");
    Serial.print(digitalRead(BUTTON_1_PIN) ? " on  " : " off ");
    Serial.print("button 2 is ");
    Serial.println(digitalRead(BUTTON_2_PIN) ? " on  " : " off ");
}

void restart_usb() {
    // also restarts usb hub (whoops)
    digitalWrite(USB_ENABLE_PIN, 0);
    delay(1000);
    digitalWrite(USB_ENABLE_PIN, 1);
}

void set_watchdog_indicator_led() {
    static int blink = 0;
    if (watchdog_enabled) {
        if ((!watchdog_trigger_imminent && millis() - nuc_watchdog_timer > 50L) || blink++ % 100 > 50)
            leds[0] = CRGB(0, 0, 100);
        else
            leds[0] = CRGB(0, 0, 0);
    }
    else
        leds[0] = CRGB(0, 0, 0);

    FastLED.show();

}

