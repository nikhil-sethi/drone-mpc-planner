#pragma once
#include "rc.h"
#include "common.h"

class ReplayRc : public Rc {
private:
    std::string logger_rfn;
    ifstream infile;
    std::string _replay_dir;

    std::string next_line;
public:
    ReplayRc(std::string replay_dir) {
        _replay_dir = replay_dir;
    }
    void init(int __attribute__((unused)));
    bool connect() { return true; }
    void close();
    void bind(bool __attribute__((unused)));
    void init_logger();
    void send_rc_data(void);
    void queue_commands(int, int, int, int);
    void telemetry_from_log(double time);
    int drone_id() { return 0; }
};
