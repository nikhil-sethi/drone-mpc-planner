#pragma once
#include "rc.h"
#include "common.h"

class ReplayRC : public RC {
private:
    std::string logger_rfn;
    ifstream infile;
    std::string _replay_dir;
    int _drone_id_rxnum = 0;

    std::string next_line;
public:
    ReplayRC(std::string replay_dir) {
        _replay_dir = replay_dir;
        port_unopened = 0;
    }
    void init(int id);
    bool connect() { return true; }
    void close();
    void bind(bool __attribute__((unused)));
    void init_logger();
    void send_rc_data(void);
    void queue_commands(int, int, int, int);
    void telemetry_from_log(double time);
    int drone_id() { return _drone_id_rxnum; }

    // value_betaflight = value_here + 1000
    int bf_headless_enabled() {return 16;}
    int bf_headless_disabled() {return 47;}
    int bf_yaw_reset() {return 79;}
    int bf_PID_loop_disabled() {return 110;}
    int bf_spin_motor() {return 142;}
    int bf_spin_motor_reversed() {return 173;}
    int bf_airmode() {return 204;}
    int bf_sleep() {return  236;}
};
