#pragma once
#include "itemtracker.h"
#include "insectreader.h"

namespace tracking {

class InsectTracker : public ItemTracker {
public: tracker_type type() { return tt_insect;}

private:
    int16_t _insect_trkr_id{-1};
    void start_new_log_line(double time, unsigned long long frame_number);
protected:
    void update_insect_prediction();
public:
    static constexpr float new_tracker_drone_ignore_zone_size = 0.3f;
    void init(int id, VisionData *_visdat, int16_t viz_id);
    void track(double time);
    bool tracking() {return _tracking;}
    bool properly_tracking() {
        return n_frames_tracking > n_frames_lost && tracking();
    }
    void append_log(double time, unsigned long long frame_number);
    int16_t insect_trkr_id() {return _insect_trkr_id;}

    void calc_world_item(tracking::BlobProps * pbs, double time);
    bool check_ignore_blobs(tracking::BlobProps * pbs);
    bool delete_me() {
        if ((n_frames_lost > n_frames_lost_threshold && _insect_trkr_id > 0)) {
            _logger->close();
            initialized = false;
            return true;
        } else
            return false;
    }
};

}
