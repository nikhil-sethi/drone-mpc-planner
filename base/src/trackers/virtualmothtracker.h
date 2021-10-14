#pragma once
#include "insecttracker.h"
#include "insectreader.h"
#include "dronecontroller.h"

namespace tracking {

class VirtualMothTracker : public InsectTracker {
public:
    enum mothbehavior {
        diving,
        escape_turn
    };

private:
    int16_t _id{-1};
    logging::InsectReader logreader;
    void start_new_log_line(double time, unsigned long long frame_number);
    cv::Point3f insect_pos;
    cv::Point3f insect_vel;
    DroneController *_dctrl;
    bool _delete_me = false;
    double start_time = 0;
    bool escape_triggered = false;
    mothbehavior behavior_type;

public:
    void init(int id, mothbehavior behavior_type, VisionData *visdat, DroneController *dctrl);
    void init_logger();
    bool check_ignore_blobs(BlobProps *pbs [[maybe_unused]]) {return false;}
    void calc_world_item(BlobProps *pbs, double time [[maybe_unused]]) {pbs->world_props.valid = false;}
    void update(double time);
    void update_behavior_based(unsigned long long frame_number, double time);
    bool delete_me();

    bool tracking() {return _tracking;}
    int16_t id() {return _id;}
    tracker_type type() {return tt_virtualmoth;}
};

}
