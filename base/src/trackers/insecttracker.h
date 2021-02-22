#pragma once
#include "itemtracker.h"

namespace tracking {

class InsectTracker : public ItemTracker {

private:
    int16_t _insect_trkr_id{-1};
    uint _fp_cnt = 0;
    float dist_integrator_fp =0;
    std::ofstream insectlogger;

    void start_new_log_line(double time, unsigned long long frame_number);
    void close_log_line();
    void check_false_positive();
public:
    static constexpr float new_tracker_drone_ignore_zone_size_world = 0.25f;
    static constexpr float new_tracker_drone_ignore_zone_size_im = 10; // pixels

    void init(int id, VisionData *_visdat, int motion_thresh, int16_t viz_id);
    void init_logger();
    void append_log(double time, unsigned long long frame_number);
    void calc_world_item(tracking::BlobProps * pbs, double time);
    bool check_ignore_blobs(tracking::BlobProps * pbs);
    void update(double time);
    bool delete_me();

    int16_t insect_trkr_id() {return _insect_trkr_id;}
    tracking::false_positive_type false_positive();
    tracker_type type() {return tt_insect;}
};

}
