#pragma once
#include "logging.h"

namespace logging
{

class InsectReader {

public:
    void init(std::string file);
    bool current_frame_number(unsigned long long RS_id) {
        _RS_id = RS_id;
        if (log.find( RS_id ) != log.end()) {
            current_entry = log.at(RS_id);
            return false;
        } else {
            return true;
        }
    }
    bool increase_frame_number() {
        while(last_RS_id > _RS_id) {
            _RS_id ++;
            if (log.find( _RS_id ) != log.end()) {
                current_entry = log.at(_RS_id);
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
    unsigned long long last_RS_id{0};
    bool _done{false};
    bool _replay_moth{false};
    unsigned long long _RS_id{0};
};

}
