#include "eeprom_settings.h"
#include <EEPROM.h>
#include "Arduino.h"

struct {
    uint32_t eeprom_writes;
    bool voltage_calibration_done;
    float voltage_calibration_value;
    bool watchdog_enabled;
    bool charging_enabled;
    uint16_t watchdog_count;
    int32_t watchdog_time;
    uint16_t watchdog_boot_count;
    uint16_t baseboard_boot_count;
    bool nuc_has_been_reset;
} eeprom_values;

template<typename Type>
int get_address(Type &value) {
    return (int)((uint8_t *) &value - (uint8_t *) &eeprom_values);
}

void print_config() {
    Serial.println("");
    Serial.print("eeprom_writes ");
    Serial.println(eeprom_values.eeprom_writes);
    Serial.print("voltage_calibration_done ");
    Serial.println(eeprom_values.voltage_calibration_done);
    Serial.print("voltage_calibration_value ");
    Serial.println(eeprom_values.voltage_calibration_value);
    Serial.print("watchdog_enabled ");
    Serial.println(eeprom_values.watchdog_enabled);
    Serial.print("charging_enabled ");
    Serial.println(eeprom_values.charging_enabled);
    Serial.print("watchdog_count ");
    Serial.println(eeprom_values.watchdog_count);
    Serial.print("watchdog_time ");
    Serial.print(eeprom_values.watchdog_time / (1000L * 3600L));
    Serial.print("H (");
    Serial.print(eeprom_values.watchdog_time);
    Serial.println(")");
    Serial.print("watchdog_boot_count ");
    Serial.println(eeprom_values.watchdog_boot_count);
    Serial.print("baseboard_boot_count ");
    Serial.println(eeprom_values.baseboard_boot_count);

}

void read_config_from_eeprom() {
    EEPROM.get(0, eeprom_values);
    print_config();
}

void write_read_config_to_eeprom() {
    eeprom_values.eeprom_writes++;
    EEPROM.put(0, eeprom_values);
}

void clear_eeprom_hard() {
    eeprom_values = {};
    print_config();
    EEPROM.put(0, eeprom_values);
}

void clear_eeprom_all() {
    memset(&(eeprom_values.voltage_calibration_done), 0, sizeof(eeprom_values) - get_address(eeprom_values.voltage_calibration_done));
    print_config();
    EEPROM.put(0, eeprom_values);
}

void clear_eeprom_exept_calib() {
    memset(&(eeprom_values.watchdog_enabled), 0, sizeof(eeprom_values) - get_address(eeprom_values.watchdog_enabled));
    print_config();
    EEPROM.put(0, eeprom_values);
}

void reset_calibration_eeprom() {
    eeprom_values.voltage_calibration_done = 0;
    write_read_config_to_eeprom();
}

void write_charger_state_eeprom(bool charger_enabled) {
    if (charger_enabled != eeprom_values.charging_enabled) {
        eeprom_values.charging_enabled = charger_enabled;
        write_read_config_to_eeprom();
    }
}

void write_watchdog_count_eeprom(bool watchdog_count) {
    if (watchdog_count != eeprom_values.watchdog_count) {
        eeprom_values.watchdog_count = watchdog_count;
        write_read_config_to_eeprom();
    }
}

void write_watchdog_boot_count_eeprom(bool watchdog_boot_count) {
    if (watchdog_boot_count != eeprom_values.watchdog_boot_count) {
        eeprom_values.watchdog_boot_count = watchdog_boot_count;
        write_read_config_to_eeprom();
    }
}

void write_baseboard_boot_count_eeprom(bool baseboard_boot_count) {
    if (baseboard_boot_count != eeprom_values.baseboard_boot_count) {
        eeprom_values.baseboard_boot_count = baseboard_boot_count;
        write_read_config_to_eeprom();
    }
}

void write_watchdog_time_eeprom(int32_t watchdog_time) {
    static uint32_t last_update_time = 0;
    eeprom_values.watchdog_time = watchdog_time;

    if (millis() - last_update_time > 10L * 60L * 1000L) {
        write_read_config_to_eeprom();
        last_update_time = millis();
    }
}

void write_watchdog_state_eeprom(bool watchdog_enabled) {
    if (watchdog_enabled != eeprom_values.watchdog_enabled) {
        eeprom_values.watchdog_enabled = watchdog_enabled;
        write_read_config_to_eeprom();
    }
}

void write_calibration_eeprom(float voltage_calibration_value) {
    eeprom_values.voltage_calibration_done = 1;
    eeprom_values.voltage_calibration_value = voltage_calibration_value;
    write_read_config_to_eeprom();
}

void write_nuc_has_been_reset_eeprom(bool nuc_has_been_reset) {
    if (nuc_has_been_reset != eeprom_values.nuc_has_been_reset) {
        eeprom_values.watchdog_enabled = nuc_has_been_reset;
        write_read_config_to_eeprom();
    }
}

bool charger_state_eeprom() {
    return eeprom_values.charging_enabled;
}

bool calibration_mode_eeprom() {
    return !eeprom_values.voltage_calibration_done;
}

bool watchdog_state_eeprom() {
    return eeprom_values.watchdog_enabled;
}

bool nuc_has_been_reset_eeprom() {
    return eeprom_values.nuc_has_been_reset;
}

float voltage_calibration_value_eeprom() {
    return eeprom_values.voltage_calibration_value;
}

int32_t watchdog_time_eeprom() {
    return eeprom_values.watchdog_time;
}

uint16_t baseboard_boot_count_eeprom() {
    return eeprom_values.baseboard_boot_count;
}

uint16_t watchdog_boot_count_eeprom() {
    return eeprom_values.watchdog_boot_count;
}

uint16_t watchdog_count_eeprom() {
    return eeprom_values.watchdog_count;
}