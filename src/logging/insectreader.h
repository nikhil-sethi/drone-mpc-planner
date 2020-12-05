#pragma once
#include "logging.h"

namespace logging
{

class InsectReader {

public:
    void init(std::string file);
    bool current_frame_number(unsigned long long rs_id) {
        _rs_id = rs_id;
        if (log.find( rs_id ) != log.end()) {
            current_entry = log.at(rs_id);
            return false;
        } else {
            return true;
        }
    }
    bool increase_frame_number() {
        while(last_rs_id > _rs_id) {
            _rs_id ++;
            if (log.find( _rs_id ) != log.end()) {
                current_entry = log.at(_rs_id);
                return false;
            }
        }
        _done = true;
        return true;
    }
    bool done() {
        return _done;
    }
    bool replay_moth() {
        return _replay_moth;
    }

    LogEntryInsect current_entry;
private:

    LogEntryInsect create_log_entry(std::string line);
    std::tuple<std::map<int, LogEntryInsect>,std::map<std::string, int>> read_log(std::string file);

    std::map<int, LogEntryInsect> log;
    std::map<std::string, int> headmap;
    unsigned long long last_rs_id{0};
    bool _done{false};
    bool _replay_moth{false};
    unsigned long long _rs_id{0};
};

}
