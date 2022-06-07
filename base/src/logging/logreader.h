#pragma once
#include <cmath>
#include "logging.h"
#include "insectreader.h"
#include "dronereader.h"
#include <limits.h>

namespace logging
{

class LogReader {

public:
    void init(std::string path);
    void init(std::string path, std::string drone_log_fn);
    void read_insect_replay_log(std::string path);
    int current_frame_number(unsigned long long rs_id);
    unsigned long long retrieve_rs_id_from_frame_id(uint frame_number) { // reverse lookup for filecam
        if (rs_ids.size() > frame_number + 1)
            return rs_ids.at(frame_number);
        else
            return ULONG_MAX;
    }
    unsigned long long start_rs_id() {return _start_rs_id;}


    LogEntryMain current_entry;
    DroneReader * log_drone() {return &_log_drone;};
    std::vector<InsectReader> replay_moths() {
        std::vector<InsectReader> replay_logs;
        for (auto &log : log_insects) {
            if (log.replay_moth()) {
                replay_logs.push_back(log);
            }
        }
        return replay_logs;
    }

private:
    void read_drone_log(std::string drone_log_fn);
    void read_multi_insect_logs(std::string path);
    LogEntryMain create_log_entry(std::string line, std::map<std::string, int> headmap);
    LogEntryInsect create_insect_log_entry(std::string line, std::map<std::string, int> headmap);

    std::tuple<std::map<int, LogEntryMain>, std::map<std::string, int>> read_log(std::string file);

    std::map<int, LogEntryMain> log_main;
    std::map<std::string, int> headmap_main;
    std::vector<InsectReader> log_insects;
    DroneReader _log_drone;

    std::vector<unsigned long long> rs_ids;
    unsigned long long _start_rs_id = 0;
};

}
