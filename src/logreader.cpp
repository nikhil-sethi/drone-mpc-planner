#include "logreader.h"
#include "common.h"

#include <iostream>
#include <string>

#include <algorithm>
#include <cctype>
#include <locale>

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

void LogReader::init(std::string file) {

    if (!checkFileExist(file)) {
        std::cout << "Error: log file not found!" <<std::endl;
        throw my_exit(1);
    }
    //read the whole log here, and process it into a table that can easily be searched
    std::ifstream infile(file);

    std::string heads;
    std::getline(infile, heads);
    setHeadMap(heads);
    std::string line;
    while (std::getline(infile, line)) {
//        std::cout << line << std::endl;
        std::istringstream iss(line);
        Log_Entry entry = createLogEntry(line);
        std::map<const int, Log_Entry>::value_type item(entry.RS_ID,entry);
        log.insert(item);
    }

}


void LogReader::setHeadMap(std::string heads) {
    std::stringstream heads_ss(heads);
    int cnt=0;

    while (!heads_ss.eof()) {
        std::string tmp;
        std::getline(heads_ss,tmp , ';');
        trim(tmp);
        std::map<const std::string, int>::value_type head(tmp,cnt);
        headmap.insert(head);
        cnt++;
        std::string grr = head.first;
    }

}

LogReader::Log_Entry LogReader::createLogEntry(std::string line) {
    std::stringstream liness(line);
    std::vector<std::string> linedata;
    while (!liness.eof()) {
        std::string tmp;
        std::getline(liness,tmp , ';');
        linedata.push_back(tmp);
    }

    Log_Entry entry;
    entry.ID = std::stoi(linedata.at(headmap["ID"]));
    entry.RS_ID = std::stoi(linedata.at(headmap["RS_ID"]));
    entry.valid = std::stoi(linedata.at(headmap["valid"]));
    entry.joyThrottle = std::stoi(linedata.at(headmap["joyThrottle"]));
    entry.joyRoll = std::stoi(linedata.at(headmap["joyRoll"]));
    entry.joyPitch = std::stoi(linedata.at(headmap["joyPitch"]));
    entry.joyYaw = std::stoi(linedata.at(headmap["joyYaw"]));
    entry.joyArmSwitch = std::stoi(linedata.at(headmap["joyArmSwitch"]));
    entry.joyModeSwitch = std::stoi(linedata.at(headmap["joyModeSwitch"]));
    entry.joyTakeoffSwitch = std::stoi(linedata.at(headmap["joyTakeoffSwitch"]));

    entry.auto_throttle = std::stoi(linedata.at(headmap["autoThrottle"]));
    entry.auto_roll = std::stoi(linedata.at(headmap["autoRoll"]));
    entry.auto_pitch = std::stoi(linedata.at(headmap["autoPitch"]));

    entry.ins_pos_x = std::stof(linedata.at(headmap["posX_insect"]));
    entry.ins_pos_y = std::stof(linedata.at(headmap["posY_insect"]));
    entry.ins_pos_z = std::stof(linedata.at(headmap["posZ_insect"]));

    entry.ins_spos_x = std::stof(linedata.at(headmap["sposX_insect"]));
    entry.ins_spos_y = std::stof(linedata.at(headmap["sposY_insect"]));
    entry.ins_spos_z = std::stof(linedata.at(headmap["sposZ_insect"]));

    entry.ins_svel_x = std::stof(linedata.at(headmap["velX_insect"]));
    entry.ins_svel_y = std::stof(linedata.at(headmap["velY_insect"]));
    entry.ins_svel_z = std::stof(linedata.at(headmap["velZ_insect"]));

    entry.ins_sacc_x = std::stof(linedata.at(headmap["accX_insect"]));
    entry.ins_sacc_y = std::stof(linedata.at(headmap["accY_insect"]));
    entry.ins_sacc_z = std::stof(linedata.at(headmap["accZ_insect"]));

    entry.ins_im_x = std::stof(linedata.at(headmap["imLx_insect"]));
    entry.ins_im_y = std::stof(linedata.at(headmap["imLy_insect"]));
    entry.ins_disparity = std::stof(linedata.at(headmap["disparity_insect"]));
    entry.ins_pred_im_x = std::stof(linedata.at(headmap["imLx_pred_insect"]));
    entry.ins_pred_im_y = std::stof(linedata.at(headmap["imLy_pred_insect"]));

    entry.ins_n_frames_lost = std::stoi(linedata.at(headmap["n_frames_lost_insect"]));
    entry.ins_n_frames_tracking = std::stoi(linedata.at(headmap["n_frames_tracking_insect"]));
    entry.ins_foundL = std::stoi(linedata.at(headmap["foundL_insect"]));
    entry.camera_angle_y = std::stoi(linedata.at(headmap["camera_angle_y"]));

    return entry;
}

void LogReader::set_current_frame_number(int _RS_frame_number) {
    current_item = log[_RS_frame_number];
    if (current_item.RS_ID != _RS_frame_number) {
        std::cout << "Warning, frame not found in log" << std::endl;
        while(current_item.RS_ID != _RS_frame_number){
            _RS_frame_number++;
            current_item = log[_RS_frame_number];
        }
    }
}
