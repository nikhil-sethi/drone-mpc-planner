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
    entry.elapsed = stod(line_data.at(headmap["elapsed"]));
    entry.replay = stoi(line_data.at(headmap["replay"]));
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
    if (entry.rs_id > last_rs_id)
        last_rs_id = entry.rs_id;

    return entry;
}
}
