#pragma once
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "versions.h"


class Baseboard {
public:
    //copy from charging.h Arduino code
    enum ChargingState {
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
        state_calibrating
    };
private:
    //copy from utility.h Arduino code
    struct __attribute__((packed)) SerialPackage {
        const char header = 'P';
        uint16_t version = required_firmware_version_baseboard;
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


    bool _replay_mode;
    std::thread thread_send;
    std::thread thread_receive;
    bool initialized = false;
    bool exit_thread = false;
    int read_timeouts = 0;

    int sock;
    sockaddr_un deamon_address;

    ChargingState _charging_state = state_drone_not_on_pad;
    bool _charging = false;
    float _uptime = 0;

    void worker_receive();
    void worker_send();

public:

    void init(bool replay_mode);
    void close();

    bool charging() {return _charging;};
    bool contact_problem() {return _charging_state == state_contact_problem;}
    bool drone_on_pad() {return _charging_state != state_drone_not_on_pad;}
    bool charging_problem() {return _charging_state == state_bat_dead || _charging_state == state_bat_does_not_charge  || _charging_state == state_calibrating;};
    ChargingState charging_state() {return _charging_state;};
    float uptime() { return _uptime;}

};
