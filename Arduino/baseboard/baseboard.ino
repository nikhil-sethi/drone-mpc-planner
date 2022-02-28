#include "utility.h"
#include "defines.h"
#include "charger.h"
#include "rgbleds.h"
#include "eeprom_settings.h"

Charger charger;
RGBLeds rgb_leds;

bool reset_nuc = false;
bool watchdog_enabled = false;
bool watchdog_trigger_imminent = false;
bool nuc_has_been_reset = false;
long last_nuc_reset = 0L;
unsigned long nuc_watchdog_timer = millis();
unsigned long nuc_last_serial_received_time = millis();
uint8_t watchdog_reset_count = 0;
uint16_t watchdog_boot_count = 0;
uint16_t baseboard_boot_count = 0;
uint16_t measured_fan_speed = 0;
uint16_t hardware_version = 0;

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

    pinMode(WATCHDOG_STATUS_LED_PIN, OUTPUT);
    digitalWrite(WATCHDOG_STATUS_LED_PIN, watchdog_enabled);

    pinMode(IR_LED_ENABLE_PIN, OUTPUT);
    digitalWrite(IR_LED_ENABLE_PIN, 1);

    pinMode(FAN_RPM_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);

    pinMode(HARDWARE_VERSION_PIN, OUTPUT);
    analogWrite(HARDWARE_VERSION_PIN, 0);
    delay(1);
    pinMode(HARDWARE_VERSION_PIN, INPUT);

    Serial.begin(115200);
    Serial.setTimeout(5);

    rgb_leds.init();

    charger.init(&rgb_leds);
    init_hardware_version();
    // restart_usb()
    // read_buttons();

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

void handle_serial_input() {
    auto n = serial_read_to_buf(serial_input_buffer);
    if (n > 3) {
        nuc_last_serial_received_time = millis();
        if (serial_input_buffer[0] == BASEBOARD_PACKAGE_PRE_HEADER && serial_input_buffer[1] == FIRMWARE_VERSION) {
            switch (serial_input_buffer[3])
            {
                case header_SerialNUC2BaseboardChargingPackage: {
                        SerialNUC2BaseboardChargingPackage *pkg = reinterpret_cast<SerialNUC2BaseboardChargingPackage * >(&serial_input_buffer);
                        if (hardware_version != 2) {
                            pkg->enable_charging = 0;
                            debugln("WARNING: HARDWARE REV MISMATCH --> DISABLED CHARGING")
                        }
                        charger.handle_serial_input_package(pkg);
                        break;
                } case header_SerialNUC2BaseboardLedPowerPackage: {
                        SerialNUC2BaseboardLedPowerPackage *pkg = reinterpret_cast<SerialNUC2BaseboardLedPowerPackage * >(&serial_input_buffer);
                        if (pkg->led_power) {
                            debugln("Enable IR LED");
                            digitalWrite(IR_LED_ENABLE_PIN, 1);
                        } else {
                            debugln("Disable IR LED");
                            digitalWrite(IR_LED_ENABLE_PIN, 0);
                        }
                        break;
                } case header_SerialNUC2BaseboardWatchdogPackage: {
                        SerialNUC2BaseboardWatchdogPackage *pkg = reinterpret_cast<SerialNUC2BaseboardWatchdogPackage * >(&serial_input_buffer);
                        nuc_watchdog_timer = millis();
                        watchdog_trigger_imminent = false;
                        nuc_has_been_reset = false;
                        write_nuc_has_been_reset_eeprom(nuc_has_been_reset);

                        if (pkg->watchdog_enabled && !watchdog_enabled) {
                            digitalWrite(WATCHDOG_STATUS_LED_PIN, 1);
                            write_watchdog_state_eeprom(watchdog_enabled);
                            watchdog_enabled = true;
                            debugln("Watchdog enabled");
                        } else if (!pkg->watchdog_enabled && watchdog_enabled) {
                            digitalWrite(WATCHDOG_STATUS_LED_PIN, 0);
                            write_watchdog_state_eeprom(watchdog_enabled);
                            watchdog_enabled = false;
                            debugln("Watchdog disabled");
                        }
                        break;
                } case header_SerialNUC2BaseboardFanPackage: {
                        SerialNUC2BaseboardFanPackage *pkg = reinterpret_cast<SerialNUC2BaseboardFanPackage * >(&serial_input_buffer);
                        debugln("Warning fan package not yet implemented");
                        break;
                } case header_SerialNUC2BaseboardRGBLEDPackage: {
                        SerialNUC2BaseboardRGBLEDPackage *pkg = reinterpret_cast<SerialNUC2BaseboardRGBLEDPackage * >(&serial_input_buffer);
                        // Serial.print("Received SerialNUC2BaseboardRGBLEDPackage with led1_state: ");
                        // Serial.print(pkg->led1_state);
                        // Serial.print(" and internet: ");
                        // Serial.print(pkg->internet_OK);
                        rgb_leds.led1_state(static_cast<RGBLeds::rgb_led_1_states>(pkg->led1_state));
                        rgb_leds.internet_OK(pkg->internet_OK);
                        rgb_leds.post_processing(pkg->postprocessing);
                        rgb_leds.daemon_OK(pkg->daemon_OK);
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


    if (millis() - nuc_last_serial_received_time > 10000L)
        rgb_leds.nuc_inresponsive();
}

void apply_loop_delay(int loop_time) {
    static unsigned long loop_time_start = millis();
    unsigned long fan_cnt = 0;
    bool fan_state_prev = analogRead(FAN_RPM_PIN) > 300;
    int dt = loop_time_start - millis() + loop_time;

    int fan_pin;
    while (millis() < loop_time_start + loop_time) {
        fan_pin = analogRead(FAN_RPM_PIN);
        bool fan_state = fan_pin > 300;
        if (fan_state != fan_state_prev) {
            fan_cnt++;
            fan_state_prev = fan_pin;
        }
    }
    measured_fan_speed = moving_average(0.01, (fan_cnt * 1000) / dt, measured_fan_speed);
    loop_time_start = millis();
}

void loop() {
    apply_loop_delay(15);

    handle_serial_input();
    handle_watchdog();
    handle_nuc_reset();
    rgb_leds.run();
    charger.run();
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
    pkg.hardware_version = hardware_version;
    pkg.baseboard_boot_count = baseboard_boot_count;
    pkg.ir_led_state = digitalRead(IR_LED_ENABLE_PIN);
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
        rgb_leds.watchdog_trigger_imminent(false);
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
    else if (millis() - nuc_watchdog_timer > WATCHDOG_TIMEOUT - 10000L) {
        watchdog_trigger_imminent = true;
        rgb_leds.watchdog_trigger_imminent(true);
    }
}

void init_hardware_version() {
    uint8_t l = 5;
    uint16_t analog_value = analogRead(HARDWARE_VERSION_PIN);
    delay(10);
    analog_value = analogRead(HARDWARE_VERSION_PIN);
    analog_value = ~analog_value;
    analog_value = analog_value >> l;
    analog_value &= 0b11111;
    uint16_t r = 0;
    for (int i = 0; i < l; i++)
        r |= (analog_value >> i & 0x1) << l - i - 1;
    analog_value = r;
    Serial.print("Hardware version ");
    Serial.println(analog_value);
    hardware_version = analog_value;
}

void read_buttons() {
    Serial.print("button is ");
    Serial.println(digitalRead(BUTTON_PIN) ? " on  " : " off ");
}

void restart_usb() {
    // also restarts usb hub (whoops)
    digitalWrite(USB_ENABLE_PIN, 0);
    delay(1000);
    digitalWrite(USB_ENABLE_PIN, 1);
}
