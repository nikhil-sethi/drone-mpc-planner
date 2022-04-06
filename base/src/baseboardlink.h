#pragma once
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <condition_variable>
#include "versions.h"
#include "common.h"
using namespace std::chrono_literals;

static const char *charging_state_names[] = {
    "disabled",
    "init",
    "drone_not_on_pad",
    "contact_problem",
    "bat_dead",
    "bat_does_not_charge",
    "revive_charging",
    "charging",
    "trickle",
    "discharge",
    "bat_measure",
    "wait_till_drone_ready",
    "bat_calibrating"
};
//copy from charging.h Arduino code
enum charging_states {
    state_disabled,
    state_init,
    state_drone_not_on_pad,
    state_contact_problem,
    state_bat_dead,
    state_bat_does_not_charge,
    state_revive_charging,
    state_normal_charging,
    state_trickle_charging,
    state_discharge,
    state_measure,
    state_wait_until_drone_ready,
    state_calibrating
};

#define BASEBOARD_PACKAGE_PRE_HEADER '@'
class BaseboardLink {
private:
    enum baseboard_package_headers {
        header_SerialBaseboard2NUCPackage = 'P',
        header_SerialExecutor2BaseboardAllowChargingPackage = 'A',
        header_SerialExecutor2BaseboardStatePackage = 'S',
    };

    //copy from utility.h Arduino code
    struct __attribute__((packed)) SerialBaseboard2NUCPackage {
        const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
        const uint16_t firmware_version = required_firmware_version_baseboard;
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
        float ground_volts;
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
    struct __attribute__((packed)) SerialExecutor2BaseboardStatePackage {
        const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
        const uint16_t firmware_version = required_firmware_version_baseboard;
        const char header = header_SerialExecutor2BaseboardStatePackage;
        uint8_t executor_state;
        const char ender = '\n';
    };
    struct __attribute__((packed)) SerialExecutor2BaseboardAllowChargingPackage {
        const char pre_header = BASEBOARD_PACKAGE_PRE_HEADER;
        const uint16_t firmware_version = required_firmware_version_baseboard;
        const char header = header_SerialExecutor2BaseboardAllowChargingPackage;
        uint8_t allow_charging;
        const char ender = '\n';
    };

    double _time = 0;
    bool _disabled = false;
    bool _replay_mode;
    bool _exit_now = false;
    std::thread thread_send;
    std::thread thread_receive;
    bool initialized = false;
    bool logger_initialized = false;
    bool exit_thread = false;
    int read_timeouts = 0;
    std::string disable_flag = "/home/pats/pats/flags/disable_baseboard";

    int sock;
    sockaddr_un deamon_address;
    std::ofstream baseboard_logger;

    std::condition_variable cv_to_baseboard;
    std::mutex cv_m_to_baseboard;
    bool update_allow_charging_pkg_to_baseboard = false;
    SerialExecutor2BaseboardAllowChargingPackage allow_charging_pkg_to_baseboard;
    bool update_executor_state_pkg_to_baseboard = false;
    SocketExecutorStatePackage executor_state_pkg_to_baseboard;

    charging_states _charging_state = state_drone_not_on_pad;
    float _bat_voltage = -1;
    bool _charging = false;
    float _uptime = 0;

    void worker_receive();
    void worker_send();
    void send_over_socket(unsigned char *data, ssize_t len);

public:

    void init(bool replay_mode);
    void init_logger();
    void time(double time) {_time = time;}
    void close();

    void inject_log() {
        // #1178
        // some default values for now:
        _charging_state  = state_trickle_charging;
        _bat_voltage = 4.2;
        _charging = true;
    }
    void inject_log(int s) {_charging_state  = static_cast<charging_states>(s);}

    bool disabled() {return _disabled;}
    bool exit_now() { return _exit_now;}
    bool battery_ready_for_flight() { return _charging_state == state_trickle_charging || (_charging_state == state_normal_charging && _bat_voltage >= dparams.min_hunt_cell_v);}
    bool charging() {return _charging_state == state_normal_charging || _charging_state == state_trickle_charging || _charging_state == state_revive_charging;};
    bool charging_waits_until_drone_ready() {return _charging_state == state_wait_until_drone_ready;};
    bool contact_problem() {return _charging_state == state_contact_problem;}
    bool drone_on_pad() {return _charging_state != state_drone_not_on_pad;}
    bool charging_problem() {return _charging_state == state_bat_dead || _charging_state == state_bat_does_not_charge || _charging_state == state_calibrating;};
    charging_states charging_state() {return _charging_state;};
    std::string charging_state_str() {
        if (!_disabled)
            return charging_state_names[_charging_state];
        else
            return "baseboard disabled";
    }
    float uptime() { return _uptime;}
    float drone_battery_voltage() { return _bat_voltage;}
    void allow_charging(bool b) {
        allow_charging_pkg_to_baseboard.allow_charging = b;
        update_allow_charging_pkg_to_baseboard = true;
        cv_to_baseboard.notify_all();
    }
    void executor_state(executor_states s) {
        executor_state_pkg_to_baseboard.executor_state = s;
        update_executor_state_pkg_to_baseboard  = true;
        cv_to_baseboard.notify_one();
    }
};
