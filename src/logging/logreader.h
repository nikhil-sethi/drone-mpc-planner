#pragma once

#include <cmath>
#include "logging.h"
#include "insectreader.h"

namespace logging
{

class LogReader{

public:
    void init(std::string path);
    void read_insect_replay_log(std::string path);
    void current_frame_number(unsigned long long RS_id);
    bool set_next_frame_number();
    double first_takeoff_time() {return _takeoff_time;}

    LogEntryMain current_entry;
    std::vector<InsectReader> replay_moths() {
        std::vector<InsectReader> replay_logs;
        for (auto log : log_insects) {
            if (log.replay_moth()){
                replay_logs.push_back(log);
            }
        }
        return replay_logs;
    }

private:
    void read_multi_insect_logs(std::string path);
    LogEntryMain create_log_entry(std::string line,std::map<std::string, int> headmap);
    LogEntryInsect create_insect_log_entry(std::string line,std::map<std::string, int> headmap);

    std::tuple<std::map<int, LogEntryMain>,std::map<std::string, int>> read_log(std::string file);

    std::map<int, LogEntryMain> log_main;
    std::map<std::string, int> headmap_main;
    std::vector<InsectReader> log_insects;

    double _takeoff_time = INFINITY;

};

}
