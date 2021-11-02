#include "insectreader.h"
#include "common.h"

#include <experimental/filesystem>

using namespace std;

namespace logging
{

void InsectReader::init(string file) {
    if (!file_exist(file)) {
        throw std::runtime_error("log file not found!");
    }
    _replay_moth = experimental::filesystem::path(file).filename().string().find("itrk") == string::npos;
    if (_replay_moth)
        cout << "Opening replay log: " << file << endl;
    else
        cout << "Opening insect log: " << file << endl;
    read_log(file);
    current_entry = log.begin()->second;
    _rs_id = current_entry.rs_id;
}

tuple<map<int, LogEntryInsect>, map<string, int>> InsectReader::read_log(string file) {
    ifstream infile(file);
    string heads;
    getline(infile, heads);
    headmap = read_head_map(heads);
    string line;
    while (getline(infile, line)) {
        try {
            LogEntryInsect entry = create_log_entry(line);
            map<const int, LogEntryInsect>::value_type item(entry.rs_id, entry);
            log.insert(item);
        } catch (exception &exp) {
            string next_line;
            if (getline(infile, next_line))
                throw std::runtime_error("Could not read insect log! \n" + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();

    return make_tuple(log, headmap);

}

LogEntryInsect InsectReader::create_log_entry(string line) {
    auto line_data = split_csv_line(line);

    LogEntryInsect entry;
    entry.time = stod(line_data.at(headmap["time"]));
    entry.replay = stoi(line_data.at(headmap["replay"]));
    entry.rs_id = stod(line_data.at(headmap["RS_ID"]));
    entry.ins_pos_x = stod(line_data.at(headmap["posX"]));
    entry.ins_pos_y = stod(line_data.at(headmap["posY"]));
    entry.ins_pos_z = stod(line_data.at(headmap["posZ"]));

    entry.ins_spos_x = stod(line_data.at(headmap["sposX"]));
    entry.ins_spos_y = stod(line_data.at(headmap["sposY"]));
    entry.ins_spos_z = stod(line_data.at(headmap["sposZ"]));

    entry.ins_svel_x = stod(line_data.at(headmap["svelX"]));
    entry.ins_svel_y = stod(line_data.at(headmap["svelY"]));
    entry.ins_svel_z = stod(line_data.at(headmap["svelZ"]));

    entry.ins_sacc_x = stod(line_data.at(headmap["saccX"]));
    entry.ins_sacc_y = stod(line_data.at(headmap["saccY"]));
    entry.ins_sacc_z = stod(line_data.at(headmap["saccZ"]));

    entry.ins_im_x = stod(line_data.at(headmap["imLx"]));
    entry.ins_im_y = stod(line_data.at(headmap["imLy"]));
    entry.ins_disparity = stod(line_data.at(headmap["disparity"]));
    entry.ins_pred_im_x = stod(line_data.at(headmap["imLx_pred"]));
    entry.ins_pred_im_y = stod(line_data.at(headmap["imLy_pred"]));

    entry.ins_n_frames_lost = stoi(line_data.at(headmap["n_frames_lost"]));
    entry.ins_n_frames_tracking = stoi(line_data.at(headmap["n_frames_tracking"]));
    entry.ins_foundL = stoi(line_data.at(headmap["foundL"]));
    if (entry.rs_id > last_rs_id)
        last_rs_id = entry.rs_id;

    return entry;
}
}
