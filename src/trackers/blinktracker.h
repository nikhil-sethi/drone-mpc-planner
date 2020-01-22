#pragma once
#include "itemtracker.h"

namespace tracking {

static const char* blinking_drone_state_names[] = { "",
                                                    "bds_start",
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
                                                    "bds_6_blink_off_calib",
                                                    "bds_6_blink_off",
                                                    "bds_6_blink_on",
                                                    "bds_found"
                                                  };

class BlinkTracker : public ItemTracker {
public: tracker_type type() { return tt_blink;}

public:
    enum blinking_drone_states {
        bds_start=1,
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
        bds_6_blink_off_calib,
        bds_6_blink_off,
        bds_6_blink_on,
        bds_found
    };

private:
    blinking_drone_states _blinking_drone_status = bds_start;
    int attempts = 0;
    double blink_time_start = 0;
    double fail_time_start = 0;
    double manual_calib_time_start = 0;

    blinking_drone_states detect_blink(double time, bool found);

    void clean_ignore_blobs(double time);

public:
    std::string state_str() {return blinking_drone_state_names[_blinking_drone_status];}
    blinking_drone_states state() {return _blinking_drone_status;}

    bool init(VisionData *_visdat, int16_t viz_id);
    void track(double time);

    void calc_world_item(tracking::BlobProps * pbs, double time);
    bool check_ignore_blobs(tracking::BlobProps * pbs);
    bool blinking_drone_located() {return _blinking_drone_status >= bds_found;}

    bool delete_me() {
        return (n_frames_lost > n_frames_lost_threshold) || _blinking_drone_status == bds_failed_delete_me;
    }

    float smoothed_size_image() {return smoother_im_size.latest();}

    float score(tracking::BlobProps blob) {
        if (path.size()>0) {
            tracking::ImageItem first = path.at(0).iti;
            return ItemTracker::score(blob,first);
        } else {
            return ItemTracker::score(blob,_image_item);
        }
    }

};

}
