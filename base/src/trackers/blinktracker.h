#pragma once
#include "itemtracker.h"

namespace tracking {

static const char *blinking_drone_state_names[] = { "bds_start",
                                                    "bds_failed",
                                                    "bds_failed_delete_me",
                                                    "bds_restart_searching",
                                                    "bds_searching",
                                                    "bds_1_blink_off",
                                                    "bds_1_blink_on",
                                                    "bds_2_blink_off",
                                                    "bds_2_blink_on",
                                                    "bds_3_blink_off",
                                                    "bds_3_blink_on",
                                                    "bds_4_blink_off",
                                                    "bds_4_blink_on",
                                                    "bds_5_blink_off",
                                                    "bds_5_blink_on",
                                                    "bds_6_blink_off",
                                                    "bds_6_blink_on",
                                                    "bds_found"
                                                  };

class BlinkTracker : public ItemTracker {
public:
    enum blinking_drone_states {
        bds_start = 0,
        bds_failed,
        bds_failed_delete_me,
        bds_restart_search,
        bds_searching,
        bds_1_blink_off,
        bds_1_blink_on,
        bds_2_blink_off,
        bds_2_blink_on,
        bds_3_blink_off,
        bds_3_blink_on,
        bds_4_blink_off,
        bds_4_blink_on,
        bds_5_blink_off,
        bds_5_blink_on,
        bds_6_blink_off,
        bds_6_blink_on,
        bds_found
    };

private:
    int16_t _blink_trkr_id{-1};
    blinking_drone_states _blinking_drone_status = bds_start;
    int attempts = 0;
    double blink_time_start = 0;
    double fail_time_start = 0;
    double manual_calib_time_start = 0;
    std::ofstream blinklogger;

    cv::Point3f last_known_valid_pos = {0};
    bool last_known_valid_pos_valid = false;

    blinking_drone_states detect_blink(double time, bool found);

    void close_log_line();
    void clean_ignore_blobs(double time);

public:
    bool init(int id, VisionData *_visdat, int motion_thresh, int16_t viz_id);
    void init_logger();
    void update(double time);
    void calc_world_item(tracking::BlobProps *pbs, double time);
    [[maybe_unused]] void match_template() {};
    bool check_ignore_blobs(tracking::BlobProps *pbs);

    bool delete_me();

    tracker_type type() { return tt_blink;}
    std::string state_str() {return blinking_drone_state_names[_blinking_drone_status];}
    blinking_drone_states state() {return _blinking_drone_status;}
    float smoothed_size_image() {return smoother_im_size.latest();}
    float score(tracking::BlobProps *blob);
    bool blinking_drone_located() {return _blinking_drone_status >= bds_found;}

};

}
