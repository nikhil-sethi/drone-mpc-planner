#include "logreader.h"
#include "common.h"
#include "navigation.h"

#include <experimental/filesystem>

using namespace std;
namespace logging
{

void LogReader::init(string path) {
    if (!file_exist(path)) {
        throw std::runtime_error("log file not found!");
    }
    cout << "Opening log folder: " << path << endl;

    string file = path + "/log.csv";
    tie(log_main, headmap_main) = read_log(file);
    read_multi_insect_logs(path);

}
void LogReader::init(string path, string drone_log_fn) {
    read_drone_log(drone_log_fn);
    init(path);
    for (auto &insect : log_insects) {
        insect.current_frame_number(_log_drone.video_start_rs_id());
    }
    current_frame_number(_log_drone.video_start_rs_id());
}

tuple<map<int, LogEntryMain>, map<string, int>> LogReader::read_log(string file) {
    ifstream infile(file);
    string heads;
    getline(infile, heads);
    map<string, int> headmap = read_head_map(heads);
    string line;
    map<int, LogEntryMain> log;
    while (getline(infile, line)) {
        try {
            LogEntryMain entry = create_log_entry(line, headmap);
            map<const int, LogEntryMain>::value_type item(entry.rs_id, entry);
            rs_ids.push_back(entry.rs_id);
            log.insert(item);
        } catch (exception &exp) {
            string next_line;
            if (getline(infile, next_line))
                throw std::runtime_error("Could not read log! File: " + file + '\n' + "Err: " + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();
    return make_tuple(log, headmap);

}

void LogReader::read_multi_insect_logs(string path) {
    //read multi insect logs
    vector<string> ins_logs;
    for (auto entry : experimental::filesystem::directory_iterator(path)) {

        if (!entry.path().extension().string().compare(".csv") && (
                    entry.path().filename().string().rfind("log_i", 0) == 0 || entry.path().filename().string().rfind("log_r", 0) == 0))
            ins_logs.push_back(entry.path().string());
    }
    for (auto &f : ins_logs) {
        InsectReader ir;
        ir.init(f);
        log_insects.push_back(ir);
    }
}

void LogReader::read_drone_log(std::string drone_log_fn) {
    if (!file_exist(drone_log_fn))
        throw std::runtime_error("Could not find drone log file: " + drone_log_fn);
    std::cout << "Opening flight log: " << drone_log_fn << std::endl;
    _log_drone.init(drone_log_fn);
}

LogEntryMain LogReader::create_log_entry(string line, map<string, int> headmap) {

    auto line_data = split_csv_line(line);

    LogEntryMain entry;
    entry.id = stoi(line_data.at(headmap["ID"]));
    entry.rs_id = stoi(line_data.at(headmap["rs_id"]));
    entry.elapsed = stod(line_data.at(headmap["elapsed"]));
    entry.exposure = stoi(line_data.at(headmap["exposure"]));
    entry.gain = stoi(line_data.at(headmap["gain"]));
    entry.trackers_state = stoi(line_data.at(headmap["trackers_state"]));
    entry.charging_state = stoi(line_data.at(headmap["charging_state"]));

    if (!_start_rs_id)
        _start_rs_id = entry.rs_id ;

    return entry;
}

int LogReader::current_frame_number(unsigned long long rs_id) {
    current_entry = log_main[rs_id];

    if (current_entry.rs_id != rs_id)
        return 1;

    if (_log_drone.initialized())
        _log_drone.current_frame_number(rs_id);

    //Currently log playback based on the insect logs is not used.
    //I'd rather recalculate the insect based on the vision.
    //(Replay logs are inserted directly into the trackermanager from somewhere else)
    if (false) { // so, disable to slightly speed up log playback
        vector<LogEntryInsect> ins_entries;
        for (auto &ins : log_insects) {
            if (!ins.current_frame_number(rs_id)) {
                ins_entries.push_back(ins.current_entry);
            }
        }
    }
    return 0;
}

} // namespace logging
