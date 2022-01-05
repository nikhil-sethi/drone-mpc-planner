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
        notconnected = 0;
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
};
