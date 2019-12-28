#pragma once
#include "insecttracker.h"
#include "insectreader.h"
/*
 * This class will replay a insect log
 *
 */
class ReplayTracker : public InsectTracker {

private:
    int16_t _id{-1};
    logging::InsectReader logreader;
    void start_new_log_line(double time, unsigned long long frame_number);
protected:
    void init_logger();
public:
    void init(int id, string file, VisionData *_visdat);
    void init(int id,logging::InsectReader, VisionData *visdat);
    void track(double time);
    void update_from_log(unsigned long long frame_number, double time);
    bool tracking(){return _tracking;}
    int16_t id(){return _id;}

    bool check_ignore_blobs(BlobProps * pbs [[maybe_unused]], double time [[maybe_unused]]) {
        return false;
    }
    void calc_world_item(BlobProps * pbs, double time [[maybe_unused]]){
        pbs->world_props.valid = false;
    }

    bool delete_me(){
        if (logreader.done()) {
            _logger->close();
            return true;
        }
        return false;
    }
};
