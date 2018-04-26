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
        exit(1);
    }
    //read the whole log here, and process it into a table that can easily be searched
    std::ifstream infile(file);

    std::string heads;
    std::getline(infile, heads);
    setHeadMap(heads);
    //        int lastframe = 0;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        Log_Entry entry = createLogEntry(line);
//        std::pair<const int, Log_Entry> item(entry.RS_ID,entry);
        std::map<const int, Log_Entry>::value_type item(entry.RS_ID,entry);
        log.insert(item);
    }

}


void LogReader::setHeadMap(std::string heads) {
    std::stringstream heads_ss(heads);
//    std::map<std::string, int> headmap;
//    std::map<std::string, int> headmap_fix;
    int cnt=0;
    int cnt_fix=0;
    while (!heads_ss.eof()) {
        std::string tmp;
        std::getline(heads_ss,tmp , ';');
        trim(tmp);
//        std::pair<const std::string, int> head(tmp,cnt);
        std::map<const std::string, int>::value_type head(tmp,cnt);
        headmap.insert(head);
        cnt++;
        std::string grr = head.first;
        if((grr.compare("imLx")!=0) && (grr.compare("imLy")!=0) && (grr.compare("disparity")!=0)) {
            std::map<const std::string, int>::value_type head_fix(tmp,cnt_fix);
            headmap_fix.insert(head_fix);
            cnt_fix++;
        }

    }

    std::cout << heads<< std::endl;

}

LogReader::Log_Entry LogReader::createLogEntry(std::string line) {
    std::stringstream liness(line);
    std::vector<std::string> linedata;
    while (!liness.eof()) {
        std::string tmp;
        std::getline(liness,tmp , ';');
        linedata.push_back(tmp);
    }



    std::map<std::string, int> headmap_grr;
    if (linedata.size() == 26)
        headmap_grr = headmap_fix;
    else
        headmap_grr = headmap;


    Log_Entry entry;
    entry.ID = std::stoi(linedata.at(headmap_grr["ID"]));
    entry.RS_ID = std::stoi(linedata.at(headmap_grr["RS_ID"]));
    if (linedata.size() != 26) {
        entry.imLx = std::stoi(linedata.at(headmap_grr["imLx"]));
        entry.imLy = std::stoi(linedata.at(headmap_grr["imLy"]));
        entry.disparity = std::stoi(linedata.at(headmap_grr["disparity"]));
    }
    entry.valid = std::stoi(linedata.at(headmap_grr["valid"]));
    entry.posErrX = std::stof(linedata.at(headmap_grr["posErrX"]));
    entry.posErrY = std::stof(linedata.at(headmap_grr["posErrY"]));
    entry.posErrZ = std::stof(linedata.at(headmap_grr["posErrZ"]));
    entry.velX = std::stof(linedata.at(headmap_grr["velX"]));
    entry.velY = std::stof(linedata.at(headmap_grr["velY"]));
    entry.velZ = std::stof(linedata.at(headmap_grr["velZ"]));
    entry.hoverthrottle = std::stoi(linedata.at(headmap_grr["hoverthrottle"]));
    entry.autoThrottle = std::stoi(linedata.at(headmap_grr["autoThrottle"]));
    entry.autoRoll = std::stoi(linedata.at(headmap_grr["autoRoll"]));
    entry.autoPitch = std::stoi(linedata.at(headmap_grr["autoPitch"]));
    entry.autoYaw = std::stoi(linedata.at(headmap_grr["autoYaw"]));
    entry.joyThrottle = std::stoi(linedata.at(headmap_grr["joyThrottle"]));
    entry.joyRoll = std::stoi(linedata.at(headmap_grr["joyRoll"]));
    entry.joyPitch = std::stoi(linedata.at(headmap_grr["joyPitch"]));
    entry.joyYaw = std::stoi(linedata.at(headmap_grr["joyYaw"]));
    entry.joySwitch = std::stoi(linedata.at(headmap_grr["joySwitch"]));
    entry.throttleP = std::stoi(linedata.at(headmap_grr["throttleP"]));
    entry.throttleI = std::stoi(linedata.at(headmap_grr["throttleI"]));
    entry.throttleD = std::stoi(linedata.at(headmap_grr["throttleD"]));
    entry.dt = std::stof(linedata.at(headmap_grr["dt"]));
    entry.dx = std::stof(linedata.at(headmap_grr["dx"]));
    entry.dy = std::stof(linedata.at(headmap_grr["dy"]));
    entry.dz = std::stof(linedata.at(headmap_grr["dz"]));

    return entry;

}

LogReader::Log_Entry LogReader::getItem(int _RS_frame_number) {
    return log[_RS_frame_number];
}


