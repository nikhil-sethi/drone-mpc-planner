#pragma once
#include <stdint.h>

bool calibration_mode_eeprom();
bool charger_state_eeprom();
bool watchdog_state_eeprom();
bool nuc_has_been_reset_eeprom();
float voltage_calibration_value_eeprom();
int32_t watchdog_time_eeprom();
uint16_t baseboard_boot_count_eeprom();
uint16_t watchdog_boot_count_eeprom();
uint16_t watchdog_count_eeprom();
void clear_eeprom_all();
void clear_eeprom_exept_calib();
void clear_eeprom_hard();
void read_config_from_eeprom();
void reset_calibration_eeprom();
void write_baseboard_boot_count_eeprom(bool baseboard_boot_count);
void write_calibration_eeprom(float voltage_calibration_value);
void write_charger_state_eeprom(bool charger_enabled);
void write_watchdog_boot_count_eeprom(bool watchdog_boot_count);
void write_watchdog_count_eeprom(bool watchdog_count);
void write_watchdog_state_eeprom(bool watchdog_enabled);
void write_watchdog_time_eeprom(int32_t watchdog_time);
void write_nuc_has_been_reset_eeprom(bool nuc_has_been_reset);