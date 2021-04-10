#include "logreader.h"
#include "common.h"
#include "navigation.h"

#include <experimental/filesystem>

using namespace std;
namespace logging
{

void LogReader::init(string path) {
    if (!file_exist(path)) {
        throw MyExit("log file not found!");
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
            map<const int, LogEntryMain>::value_type item(entry.rs_id,entry);
            rs_ids.push_back(entry.rs_id);
            log.insert(item);
        } catch (exception& exp ) {
            string next_line;
            if (getline(infile, next_line))
                throw MyExit("Could not read log! File: " +file + '\n' + "Err: " + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();
    return make_tuple(log,headmap);

}

void LogReader::read_multi_insect_logs(string path) {
    //read multi insect logs
    vector<string> ins_logs;
    for (auto entry : experimental::filesystem::directory_iterator(path)) {

        if (!entry.path().extension().string().compare(".csv") && (
                    entry.path().filename().string().rfind("log_i", 0) == 0 || entry.path().filename().string().rfind("log_r", 0) == 0))
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
    entry.rs_id = stoi(line_data.at(headmap["RS_ID"]));
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
    entry.nav_state = stoi(line_data.at(headmap["nav_state"]));

    entry.imLx_drone = stof(line_data.at(headmap["imLx_drone"]));
    entry.imLy_drone = stof(line_data.at(headmap["imLy_drone"]));
    entry.disparity_drone = stod(line_data.at(headmap["disparity_drone"]));

    entry.auto_throttle = stoi(line_data.at(headmap["autoThrottle"]));
    entry.auto_roll = stoi(line_data.at(headmap["autoRoll"]));
    entry.auto_pitch = stoi(line_data.at(headmap["autoPitch"]));

    if (isinf(_takeoff_time) && (entry.nav_state == navigation::ns_chasing_insect_ff || entry.nav_state == navigation::ns_chasing_insect || entry.nav_state == navigation::ns_set_waypoint || entry.nav_state == navigation::ns_taking_off))
        _takeoff_time = entry.elapsed;
    if (isinf(_drone_ready_time) && entry.nav_state == navigation::ns_calibrating_motion_done)
        _drone_ready_time = entry.elapsed;
    if (isinf(_yaw_reset_time) && entry.nav_state == navigation::ns_reset_headless_yaw)
        _yaw_reset_time = entry.elapsed;
    if (isinf(_drone_problem_time) && entry.nav_state == navigation::ns_drone_problem)
        _drone_problem_time = entry.elapsed;
    if (isinf(_first_blink_time) && entry.nav_state == navigation::ns_wait_locate_drone)
        _first_blink_time = entry.elapsed;

    return entry;
}

int LogReader::current_frame_number(unsigned long long rs_id) {
    current_entry = log_main[rs_id];

    if (current_entry.rs_id != rs_id)
        return 1;

    //Currently log playback based on the insect logs is not used.
    //I'd rather recalculate the insect based on the vision.
    //(Replay logs are inserted directly into the trackermanager from somewhere else)
    if (false) { // so, disable to slightly speed up log playback
        vector<LogEntryInsect> ins_entries;
        for (auto & ins : log_insects) {
            if (!ins.current_frame_number(rs_id)) {
                ins_entries.push_back(ins.current_entry);
            }
        }
        current_entry.insects  = ins_entries;
    }
    return 0;
}

} // namespace logging
