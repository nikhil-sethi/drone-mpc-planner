#pragma once
#include "logging.h"

namespace logging
{

class DroneReader {

public:
    void init(std::string file);
    int current_frame_number(unsigned long long rs_id) {
        if (rs_id < _start_rs_id)
            return -1;
        else if (rs_id > last_rs_id)
            return 1;

        _rs_id = rs_id;

        if (log.find(rs_id) != log.end()) {
            current_entry = log.at(rs_id);
            return 0;
        } else {
            return 2;
        }
    }
    bool increase_frame_number() {
        while (last_rs_id > _rs_id) {
            _rs_id ++;
            if (log.find(_rs_id) != log.end()) {
                current_entry = log.at(_rs_id);
                return false;
            }
        }
        _done = true;
        return true;
    }
    bool initialized() { return _initialized; }
    bool done() { return _done; }
    unsigned long long start_rs_id() { return _start_rs_id; }
    int video_start_rs_id() { return _video_start_rs_id; }
    int flight_id() { return _flight_id; }

    LogEntryDrone current_entry;
private:

    LogEntryDrone create_log_entry(std::string line);
    std::tuple<std::map<int, LogEntryDrone>, std::map<std::string, int>> read_log(std::string file);

    std::map<int, LogEntryDrone> log;
    std::map<std::string, int> headmap;
    unsigned long long last_rs_id{0};
    unsigned long long _start_rs_id{0};
    bool _done{false};
    unsigned long long _rs_id{0};
    bool _initialized{false};
    int _flight_id{-1};
    int _video_start_rs_id{-1};
};

}
