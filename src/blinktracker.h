#ifndef BLINKTRACKER_H
#define BLINKTRACKER_H

#include "itemtracker.h"

/*
 * This class will track a micro drone with leds
 *
 */
static const char* blinking_drone_state_names[] = { "",
                                                    "bds_start",
                                                    "bds_reset_bkg",
                                                    "bds_searching",
                                                    "bds_1_blink_off",
                                                    "bds_1_blink_on",
                                                    "bds_2_blink_off",
                                                    "bds_2_blink_on",
                                                    "bds_3_blink_off_calib",
                                                    "bds_3_blink_off",
                                                    "bds_3_blink_on",
                                                    "bds_found" };

class BlinkTracker : public ItemTracker {


public:
    enum blinking_drone_states {
        bds_start=1,
        bds_reset_bkg,
        bds_searching,
        bds_1_blink_off,
        bds_1_blink_on,
        bds_2_blink_off,
        bds_2_blink_on,
        bds_3_blink_off_calib,
        bds_3_blink_off,
        bds_3_blink_on,
        bds_found
    };
private:
    blinking_drone_states _blinking_drone_status = bds_start;
    double blink_time_start = 0;
    double manual_calib_time_start = 0;

    blinking_drone_states detect_blink(double time, bool found);

    void clean_ignore_blobs(double time);

public:
    std::string state_str() {return blinking_drone_state_names[_blinking_drone_status];}
    blinking_drone_states state() {return _blinking_drone_status;}

    bool init(VisionData *_visdat);
    void track(double time);

    BlobWorldProps calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs, double time);
    bool blinking_drone_located() {return _blinking_drone_status >= bds_found;}

    bool delete_me(){
        return n_frames_lost > n_frames_lost_threshold;;
    }

    float smoothed_size_image(){return smoother_im_size.latest();}

};




#endif //BLINKTRACKER_H
