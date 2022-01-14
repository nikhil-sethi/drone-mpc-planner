#include "replayrc.h"

void ReplayRC::init(int drone_id) {
    _drone_id_rxnum = drone_id;
    logger_rfn = _replay_dir  + "/log_telem.txt";
    infile = ifstream(logger_rfn);
    getline(infile, next_line);
    initialized = true;
}

void ReplayRC::init_logger() {
    logger_initialized = true;
}

std::tuple<double, std::string, std::string>  split_telem_line(std::string line) {
    auto lp = split_csv_line(line);
    if (lp.size() > 2) {
        auto log_time_str = lp.at(0);
        double log_time = std::stod(log_time_str);
        return std::make_tuple(log_time, lp.at(1), lp.at(2));
    }
    return std::make_tuple(INFINITY, "", "");
}

void ReplayRC::telemetry_from_log(double time) {
    auto [log_time, id_str, val_str] = split_telem_line(next_line);
    while (log_time < time) {
        try {
            if (!strcmp(id_str.c_str(), "FSSP_DATAID_ROLL")) { // also from FSSP_DATAID_ROLL_PITCH
                telemetry.roll = std::stof(val_str);
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_PITCH")) { // also from FSSP_DATAID_ROLL_PITCH
                telemetry.pitch = std::stof(val_str);
                telemetry.roll_pitch_package_id++;
            } else  if (!strcmp(id_str.c_str(), "FSSP_DATAID_A4")) {
                telemetry.batt_cell_v = std::stof(val_str);
                telemetry.batt_cell_v_package_id++;
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_VFAS")) {
                telemetry.batt_cell_v = std::stof(val_str);
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_RSSI")) {
                telemetry.rssi = std::stoi(val_str);
                telemetry.rssi_last_received = time;
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_BF_VERSION")) {
                auto v_str = split_csv_line(val_str, '.');
                telemetry.bf_major = std::stoi(v_str.at(0));
                telemetry.bf_minor = std::stoi(v_str.at(1));
                telemetry.bf_patch = std::stoi(v_str.at(2));
                _bf_version_error = -1;
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_BF_UID")) {
                telemetry.bf_uid_str = val_str;
                _bf_uid_error = -1;
            } else if (!strcmp(id_str.c_str(), "FSSP_DATAID_ARMING")) {
                telemetry.arming_state = static_cast<arming_states>(std::stoi(val_str));
                telemetry.arming_state_package_id++;
            }
        } catch (std::invalid_argument const &err) {
            std::cout << "Warning: telemetry logging parsing problem..." << std::endl;
        }

        if (!getline(infile, next_line))
            break;
        std::tie(log_time, id_str, val_str) = split_telem_line(next_line);
    }
}

void ReplayRC::close() {
    initialized = false;
}
