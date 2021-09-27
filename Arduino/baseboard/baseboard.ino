#include "utility.h"
#include "charger.h"
#include "eeprom_settings.h"

#define NUC_ENABLE_PIN 8

#define SECOND_IN_MILLIS 1000L
#define MINUTE_IN_MILLIS (60L * SECOND_IN_MILLIS)
#define HOUR_IN_MILLIS (60L * MINUTE_IN_MILLIS)
#define DAY_IN_MILLIS (24L * HOUR_IN_MILLIS)
#define NUC_OFF_TIME (30L * SECOND_IN_MILLIS)
#define WATCHDOG_TIMEOUT HOUR_IN_MILLIS
#define WATCHDOG_STATUS_LED 13

#define VERSION "1.0"
const char version_string[] = "vers_stmp: v = " VERSION;

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

void setup() {
    init_values_from_eeprom();
    write_baseboard_boot_count_eeprom(++baseboard_boot_count);

    // set pin high before and after setting pinmode to prevent NUC from resetting
    digitalWrite(NUC_ENABLE_PIN, 1);
    pinMode(NUC_ENABLE_PIN, OUTPUT);
    digitalWrite(NUC_ENABLE_PIN, 1);

    Serial.begin(115200);
    Serial.setTimeout(5);

    pinMode(WATCHDOG_STATUS_LED, OUTPUT);
    digitalWrite(WATCHDOG_STATUS_LED, watchdog_enabled);

    pinMode(LED_ENABLE_PIN, OUTPUT);
    digitalWrite(LED_ENABLE_PIN, 1);

    charger.init();

    debugln("baseboard started");
    Serial.println(version_string);
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
        delay(min(wait_time,1));
}

void handle_serial_commands() {
    const int size = 100;
    char input[size];

    if (serial_input(input, size)) {
        Serial.println(input);
        handle_watchdog_commands(input);
        handle_led_commands(input);
        handle_reboot_commands(input);
        handle_eeprom_commands(input);
        charger.handle_commands(input);
    }
}

void loop() {
    apply_loop_delay(15);

    handle_serial_commands();
    handle_watchdog();
    handle_nuc_reset();

    charger.run();
    if (!charger.calibrating())
        run_display();
}

void add_header(char* buf) {
    int header_length = 16;
    char log_line[header_length] = "";
    sprintf(log_line, "disp: len: %03d, ", min(header_length + strlen(buf), 999));
    reverse_strcat(log_line, buf);
}

void run_display() {
    static long value_update_time = 0;

    if (millis() - value_update_time < 500L)
        return;

    uint8_t deci_hours_since_reset = (millis() - last_nuc_reset) * 10 / HOUR_IN_MILLIS;
    write_watchdog_time_eeprom(millis() - last_nuc_reset);

    value_update_time = millis();

    char log_line[200] = "";
    append_log_line(log_line, "ver", LOG_LINE_VERSION);
    append_log_line(log_line, "led", digitalRead(LED_ENABLE_PIN));
    append_log_line(log_line, "upt", min(millis() / HOUR_IN_MILLIS, 999));
    append_log_line(log_line, "wdg", watchdog_enabled);
    if (watchdog_enabled) {
        append_log_line(log_line, "wdt", max((int)((WATCHDOG_TIMEOUT + nuc_watchdog_timer - millis())/10000),0));
        append_log_line(log_line, "tsr", min(deci_hours_since_reset, 999));
        append_log_line(log_line, "wrc", watchdog_reset_count);
        append_log_line(log_line, "wbc", watchdog_boot_count);
        append_log_line(log_line, "bbc", baseboard_boot_count);
    }

    charger.append_charging_log(log_line);
    add_header(log_line);
    Serial.println(log_line);

    if (watchdog_trigger_imminent)
        debugln("Watchdog will trigger in %d seconds!", (int)((WATCHDOG_TIMEOUT + nuc_watchdog_timer - millis())/1000));
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

void enable_watchdog() {
    debugln("Watchdog enabled");
    watchdog_enabled = true;
    digitalWrite(WATCHDOG_STATUS_LED, 1);
    write_watchdog_state_eeprom(watchdog_enabled);
}

void disable_watchdog() {
    debugln("Watchdog disabled");
    watchdog_enabled = false;
    digitalWrite(WATCHDOG_STATUS_LED, 0);
    write_watchdog_state_eeprom(watchdog_enabled);
}

void handle_watchdog_commands(char * input) {
    if (strlen(input) > 0) {
        nuc_watchdog_timer = millis();
        watchdog_trigger_imminent = false;
        nuc_has_been_reset = false;
        write_nuc_has_been_reset_eeprom(nuc_has_been_reset);
    }

    if (match_command(input, "enable watchdog"))
        enable_watchdog();
    else if (match_command(input, "disable watchdog"))
        disable_watchdog();
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

void handle_led_commands(char* input) {
    if (match_command(input, "enable led")) {
        debugln("Enable LED");
        digitalWrite(LED_ENABLE_PIN, 1);
    }
    else if (match_command(input, "disable led")) {
        debugln("Disable LED");
        digitalWrite(LED_ENABLE_PIN, 0);
    }
}

void handle_reboot_commands(char* input) {
    if (match_command(input, "reboot nuc")) {
        debugln("Reboot NUC");
        reset_nuc = true;
    }
}

void handle_eeprom_commands(char* input) {
    if (match_command(input, "clear config all")) {
        debugln("clear config all");
        clear_eeprom_all();
        init_values_from_eeprom();
    }
    if (match_command(input, "clear config log")) {
        debugln("clear config log");
        clear_eeprom_exept_calib();
        init_values_from_eeprom();
    }
    if (match_command(input, "clear config hard")) {
        debugln("clear config hard");
        clear_eeprom_hard();
        init_values_from_eeprom();
    }
}
