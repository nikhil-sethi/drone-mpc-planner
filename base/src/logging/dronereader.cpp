#include "dronereader.h"
#include "common.h"
#include "navigation.h"

#include <experimental/filesystem>

using namespace std;

namespace logging
{

void DroneReader::init(string file) {
    if (!file_exist(file)) {
        throw std::runtime_error("log file not found!");
    }
    read_log(file);
    current_entry = log.begin()->second;
    _rs_id = current_entry.rs_id;
    std::string flight_id_str  = std::experimental::filesystem::path(file).filename().string();
    flight_id_str = flight_id_str.substr(10, flight_id_str.size() - 14);
    _flight_id = stoi(flight_id_str);

    std::string video_rs_id_start_fn = std::experimental::filesystem::path(file).replace_filename("flight" + flight_id_str + ".txt").string();
    if (!file_exist(video_rs_id_start_fn))
        throw runtime_error("Video_start_rs_id file not found: " + video_rs_id_start_fn);
    ifstream video_rs_id_start__file(video_rs_id_start_fn);
    string offset;
    getline(video_rs_id_start__file, offset);
    _video_start_rs_id = stoi(offset);

    _initialized = true;
}

tuple<map<int, LogEntryDrone>, map<string, int>> DroneReader::read_log(string file) {
    ifstream infile(file);
    string heads;
    getline(infile, heads);
    headmap = read_head_map(heads);
    string line;
    while (getline(infile, line)) {
        try {
            LogEntryDrone entry = create_log_entry(line);
            map<const int, LogEntryDrone>::value_type item(entry.rs_id, entry);
            log.insert(item);
        } catch (exception &exp) {
            string next_line;
            if (getline(infile, next_line))
                throw std::runtime_error("Could not read drone log! \n" + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();

    return make_tuple(log, headmap);

}

LogEntryDrone DroneReader::create_log_entry(string line) {
    auto line_data = split_csv_line(line);

    LogEntryDrone entry;
    entry.elapsed = stod(line_data.at(headmap["elapsed"]));
    entry.rs_id = stod(line_data.at(headmap["rs_id"]));
    entry.pos_x = stod(line_data.at(headmap["posX"]));
    entry.pos_y = stod(line_data.at(headmap["posY"]));
    entry.pos_z = stod(line_data.at(headmap["posZ"]));

    entry.spos_x = stod(line_data.at(headmap["sposX"]));
    entry.spos_y = stod(line_data.at(headmap["sposY"]));
    entry.spos_z = stod(line_data.at(headmap["sposZ"]));

    entry.svel_x = stod(line_data.at(headmap["svelX"]));
    entry.svel_y = stod(line_data.at(headmap["svelY"]));
    entry.svel_z = stod(line_data.at(headmap["svelZ"]));

    entry.sacc_x = stod(line_data.at(headmap["saccX"]));
    entry.sacc_y = stod(line_data.at(headmap["saccY"]));
    entry.sacc_z = stod(line_data.at(headmap["saccZ"]));

    entry.im_x = stod(line_data.at(headmap["imLx"]));
    entry.im_y = stod(line_data.at(headmap["imLy"]));
    entry.disparity = stod(line_data.at(headmap["disparity"]));
    entry.pred_im_x = stod(line_data.at(headmap["imLx_pred"]));
    entry.pred_im_y = stod(line_data.at(headmap["imLy_pred"]));

    entry.n_frames_lost = stoi(line_data.at(headmap["n_frames_lost"]));
    entry.n_frames_tracking = stoi(line_data.at(headmap["n_frames_tracking"]));
    entry.foundL = stoi(line_data.at(headmap["foundL"]));

    entry.nav_flight_mode = stoi(line_data.at(headmap["nav_flight_mode"]));
    entry.insect_id = stoi(line_data.at(headmap["insect_id"]));
    entry.charging_state = stoi(line_data.at(headmap["charging_state"]));

    entry.target_pos_x = stoi(line_data.at(headmap["target_pos_x"]));
    entry.target_pos_y = stoi(line_data.at(headmap["target_pos_y"]));
    entry.target_pos_z = stoi(line_data.at(headmap["target_pos_z"]));

    entry.auto_roll = stoi(line_data.at(headmap["auto_roll"]));
    entry.auto_pitch = stoi(line_data.at(headmap["auto_pitch"]));
    entry.auto_yaw = stoi(line_data.at(headmap["auto_yaw"]));
    entry.auto_throttle = stoi(line_data.at(headmap["auto_throttle"]));

    entry.joy_roll = stoi(line_data.at(headmap["joy_roll"]));
    entry.joy_pitch = stoi(line_data.at(headmap["joy_pitch"]));
    entry.joy_yaw = stoi(line_data.at(headmap["joy_yaw"]));
    entry.joy_throttle = stoi(line_data.at(headmap["joy_throttle"]));

    entry.joy_arm_switch = stoi(line_data.at(headmap["joy_arm_switch"]));
    entry.joy_mode_switch = stoi(line_data.at(headmap["joy_mode_switch"]));
    entry.joy_takeoff_switch = stoi(line_data.at(headmap["joy_takeoff_switch"]));

    if (_start_rs_id <= 0)
        _start_rs_id = entry.rs_id;;
    if (entry.rs_id > last_rs_id)
        last_rs_id = entry.rs_id;

    return entry;
}
}
