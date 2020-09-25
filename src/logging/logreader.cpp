#include "logreader.h"
#include "common.h"

#include <experimental/filesystem>

using namespace std;
namespace logging
{

void LogReader::init(string path) {
    if (!file_exist(path)) {
        throw my_exit("log file not found!");
    }
    cout << "Opening log folder: " << path << endl;

    string file = path + "/log.csv";
    tie (log_main,headmap_main) = read_log(file);
    read_multi_insect_logs(path);

}

tuple<map<int, LogEntryMain>,map<string, int>> LogReader::read_log(string file) {
    ifstream infile(file);
    string heads;
    getline(infile, heads);
    map<string, int> headmap = read_head_map(heads);
    string line;
    map<int, LogEntryMain> log;
    while (getline(infile, line)) {
        try {
            LogEntryMain entry = create_log_entry(line,headmap);
            map<const int, LogEntryMain>::value_type item(entry.RS_id,entry);
            RS_IDs.push_back(entry.RS_id);
            log.insert(item);
        } catch (exception& exp ) {
            string next_line;
            if (getline(infile, next_line))
                throw my_exit("Could not read log! File: " +file + '\n' + "Line: " + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();
    return make_tuple(log,headmap);

}

void LogReader::read_multi_insect_logs(string path) {
    //read multi insect logs
    vector<string> ins_logs;
    for (auto entry : experimental::filesystem::directory_iterator(path)) {
        if (!entry.path().extension().string().compare(".csv") && entry.path().filename().string().compare("log.csv"))
            ins_logs.push_back(entry.path().string());
    }
    for (auto & f : ins_logs) {
        InsectReader ir;
        ir.init(f);
        log_insects.push_back(ir);
    }
}

LogEntryMain LogReader::create_log_entry(string line, map<string, int> headmap) {

    auto line_data = split_csv_line(line);

    LogEntryMain entry;
    entry.id = stoi(line_data.at(headmap["ID"]));
    entry.RS_id = stoi(line_data.at(headmap["RS_ID"]));
    entry.elapsed = stod(line_data.at(headmap["elapsed"]));
    auto iid = headmap["insect_log"];
    if (iid)
        entry.insect_replay_log = stoi(line_data.at(iid));
    else
        entry.insect_replay_log = false;
    entry.valid = stoi(line_data.at(headmap["valid"]));
    entry.joyThrottle = stoi(line_data.at(headmap["joyThrottle"]));
    entry.joyRoll = stoi(line_data.at(headmap["joyRoll"]));
    entry.joyPitch = stoi(line_data.at(headmap["joyPitch"]));
    entry.joyYaw = stoi(line_data.at(headmap["joyYaw"]));
    entry.joyArmSwitch = stoi(line_data.at(headmap["joyArmSwitch"]));
    entry.joyModeSwitch = stoi(line_data.at(headmap["joyModeSwitch"]));
    entry.joyTakeoffSwitch = stoi(line_data.at(headmap["joyTakeoffSwitch"]));
    entry.trkrs_state = stoi(line_data.at(headmap["trkrs_state"]));

    entry.imLx_drone = stof(line_data.at(headmap["imLx_drone"]));
    entry.imLy_drone = stof(line_data.at(headmap["imLy_drone"]));
    entry.disparity_drone = stof(line_data.at(headmap["disparity_drone"]));

    if (_takeoff_time > static_cast<double>(entry.RS_id)/pparams.fps && entry.valid && entry.joyModeSwitch > 0 && entry.auto_throttle > dparams.spinup_throttle_non3d)
        _takeoff_time = static_cast<double>(entry.RS_id)/pparams.fps - 1;

    entry.auto_throttle = stoi(line_data.at(headmap["autoThrottle"]));
    entry.auto_roll = stoi(line_data.at(headmap["autoRoll"]));
    entry.auto_pitch = stoi(line_data.at(headmap["autoPitch"]));

    return entry;
}

int LogReader::current_frame_number(unsigned long long RS_id) {
    current_entry = log_main[RS_id];

    if (current_entry.RS_id != RS_id)
        return 1;

    vector<LogEntryInsect> ins_entries;
    for (auto & ins : log_insects) {
        if (!ins.current_frame_number(RS_id)) {
            ins_entries.push_back(ins.current_entry);
        }
    }
    current_entry.insects  = ins_entries;
    return 0;
}

} // namespace logging

