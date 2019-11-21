#include "logreader.h"
#include "common.h"

#include <iostream>
#include <string>

#include <algorithm>
#include <cctype>
#include <locale>

#include <experimental/filesystem>

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
                return !std::isspace(ch);
            }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
                return !std::isspace(ch);
            }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

void LogReader::init(std::string path) {
    if (!file_exist(path)) {
        throw my_exit("log file not found!");
    }
    std::cout << "Opening log folder: " << path << std::endl;

    std::string file = path + "/log.csv";
    std::tie (log_main,headmap_main) = read_main_log(file);
    read_multi_insect_logs(path);

}

void LogReader::read_insect_replay_log(std::string path) {


}

std::tuple<std::map<int, LogReader::Log_Entry_Main>,std::map<std::string, int>> LogReader::read_main_log(std::string file) {
    std::ifstream infile(file);
    std::string heads;
    std::getline(infile, heads);
    std::map<std::string, int> headmap = read_head_map(heads);
    std::string line;
    std::map<int, Log_Entry_Main> log;
    while (std::getline(infile, line)) {
        try {
            Log_Entry_Main entry = create_main_log_entry(line,headmap);
            std::map<const int, Log_Entry_Main>::value_type item(entry.RS_ID,entry);
            log.insert(item);
        } catch (std::exception& exp ) {
            throw my_exit("Could not read log! \n" + std::string(exp.what()) + " at: " + line);
        }
    }

    return std::make_tuple(log,headmap);

}
std::tuple<std::map<int, LogReader::Log_Entry_Insect>,std::map<std::string, int>> LogReader::read_insect_log(std::string file) {
    std::ifstream infile(file);
    std::string heads;
    std::getline(infile, heads);
    std::map<std::string, int> headmap = read_head_map(heads);
    std::string line;
    std::map<int, Log_Entry_Insect> log;
    while (std::getline(infile, line)) {
        try {
            Log_Entry_Insect entry = create_insect_log_entry(line,headmap);
            std::map<const int, Log_Entry_Insect>::value_type item(entry.RS_ID,entry);
            log.insert(item);
        } catch (std::exception& exp ) {
            throw my_exit("Could not read log! \n" + std::string(exp.what()) + " at: " + line);
        }
    }

    return std::make_tuple(log,headmap);

}

void LogReader::read_multi_insect_logs(std::string path) {
    //read multi insect logs
    std::vector<string> ins_logs;
    for (auto entry : std::experimental::filesystem::directory_iterator(path)) {
        if (!entry.path().extension().string().compare(".csv") && entry.path().filename().string().compare("log.csv"))
            ins_logs.push_back(entry.path().string());
    }
    for (auto f : ins_logs) {
        auto [log,headmap] = read_insect_log(f);
        log_insects.push_back(std::make_tuple(log,headmap));
    }
}

std::vector<std::string> split_csv_line(std::string line){
    std::stringstream liness(line);
    std::vector<std::string> line_data;
    while (!liness.eof()) {
        std::string tmp;
        std::getline(liness,tmp , ';');
        line_data.push_back(tmp);
    }
    return line_data;
}

std::map<std::string, int> LogReader::read_head_map(std::string heads) {
    std::stringstream heads_ss(heads);
    int cnt=0;

    std::map<std::string, int> res;
    while (!heads_ss.eof()) {
        std::string tmp;
        std::getline(heads_ss,tmp , ';');
        trim(tmp);
        std::map<const std::string, int>::value_type head(tmp,cnt);
        res.insert(head);
        cnt++;
    }
    return res;
}

LogReader::Log_Entry_Main LogReader::create_main_log_entry(std::string line, std::map<std::string, int> headmap) {

    auto line_data = split_csv_line(line);

    Log_Entry_Main entry;
    entry.ID = std::stoi(line_data.at(headmap["ID"]));
    entry.RS_ID = std::stoi(line_data.at(headmap["RS_ID"]));
    auto iid = headmap["insect_log"];
    if (iid)
        entry.insect_replay_log = std::stoi(line_data.at(iid));
    else
        entry.insect_replay_log = false;
    entry.valid = std::stoi(line_data.at(headmap["valid"]));
    entry.joyThrottle = std::stoi(line_data.at(headmap["joyThrottle"]));
    entry.joyRoll = std::stoi(line_data.at(headmap["joyRoll"]));
    entry.joyPitch = std::stoi(line_data.at(headmap["joyPitch"]));
    entry.joyYaw = std::stoi(line_data.at(headmap["joyYaw"]));
    entry.joyArmSwitch = std::stoi(line_data.at(headmap["joyArmSwitch"]));
    entry.joyModeSwitch = std::stoi(line_data.at(headmap["joyModeSwitch"]));
    entry.joyTakeoffSwitch = std::stoi(line_data.at(headmap["joyTakeoffSwitch"]));

    if (_takeoff_time > static_cast<double>(entry.RS_ID)/pparams.fps && entry.valid && entry.joyModeSwitch > 0 && entry.insects.size()>0)
        _takeoff_time = static_cast<double>(entry.RS_ID)/pparams.fps;

    entry.auto_throttle = std::stoi(line_data.at(headmap["autoThrottle"]));
    entry.auto_roll = std::stoi(line_data.at(headmap["autoRoll"]));
    entry.auto_pitch = std::stoi(line_data.at(headmap["autoPitch"]));

    return entry;
}

LogReader::Log_Entry_Insect LogReader::create_insect_log_entry(std::string line, std::map<std::string, int> headmap) {
    auto line_data = split_csv_line(line);

    Log_Entry_Insect entry;
    entry.time = std::stod(line_data.at(headmap["time"]));
    entry.replay = std::stoi(line_data.at(headmap["replay"]));
    entry.RS_ID = std::stod(line_data.at(headmap["RS_ID"]));
    entry.ins_pos_x = std::stod(line_data.at(headmap["posX_insect"]));
    entry.ins_pos_y = std::stod(line_data.at(headmap["posY_insect"]));
    entry.ins_pos_z = std::stod(line_data.at(headmap["posZ_insect"]));

    entry.ins_spos_x = std::stod(line_data.at(headmap["sposX_insect"]));
    entry.ins_spos_y = std::stod(line_data.at(headmap["sposY_insect"]));
    entry.ins_spos_z = std::stod(line_data.at(headmap["sposZ_insect"]));

    entry.ins_svel_x = std::stod(line_data.at(headmap["svelX_insect"]));
    entry.ins_svel_y = std::stod(line_data.at(headmap["svelY_insect"]));
    entry.ins_svel_z = std::stod(line_data.at(headmap["svelZ_insect"]));

    entry.ins_sacc_x = std::stod(line_data.at(headmap["saccX_insect"]));
    entry.ins_sacc_y = std::stod(line_data.at(headmap["saccY_insect"]));
    entry.ins_sacc_z = std::stod(line_data.at(headmap["saccZ_insect"]));

    entry.ins_im_x = std::stod(line_data.at(headmap["imLx_insect"]));
    entry.ins_im_y = std::stod(line_data.at(headmap["imLy_insect"]));
    entry.ins_disparity = std::stod(line_data.at(headmap["disparity_insect"]));
    entry.ins_pred_im_x = std::stod(line_data.at(headmap["imLx_pred_insect"]));
    entry.ins_pred_im_y = std::stod(line_data.at(headmap["imLy_pred_insect"]));

    entry.ins_n_frames_lost = std::stoi(line_data.at(headmap["n_frames_lost_insect"]));
    entry.ins_n_frames_tracking = std::stoi(line_data.at(headmap["n_frames_tracking_insect"]));
    entry.ins_foundL = std::stoi(line_data.at(headmap["foundL_insect"]));

    return entry;
}


bool LogReader::set_next_frame_number() {

}

void LogReader::current_frame_number(unsigned long long RS_id) {
    current_entry = log_main[RS_id];

    if (current_entry.RS_ID != RS_id) {
        std::cout << "Warning, frame not found in log" << std::endl;
        while(current_entry.RS_ID != RS_id){
            RS_id++;
            current_entry = log_main[RS_id];
        }
    }

    std::vector<Log_Entry_Insect> ins_entries;
    for (auto ins : log_insects) {
        auto [log,hm]  = ins;
        hm.clear(); // dummy to kill warning
        auto entry = log[RS_id];
        if (entry.RS_ID == RS_id)
            ins_entries.push_back(entry);
    }

    current_entry.insects  = ins_entries;

    if (current_entry.insect_replay_log) {
        current_replay_insect_entry = ins_entries[0];
        if (!current_replay_insect_entry.replay) // possibly using the log, fix this when it becomes necessary
            std::cout << "Warning: log does not indicate a replay moth!" << std::endl;
    }

}
