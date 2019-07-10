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
                                                    "bds_dedicated_calib",
                                                    "bds_calib_wait",
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
        bds_dedicated_calib,
        bds_calib_wait,
        bds_found
    };
private:
    blinking_drone_states _blinking_drone_status = bds_start;
    double blink_time_start = 0;

    double manual_calib_time_start = 0;

    blinking_drone_states detect_blink(double time, bool found);
    cv::KeyPoint blink_location;

    cv::Point2f _drone_blink_image_location;
    cv::Point3f _drone_blink_world_location;

protected:
    void init_settings();
public:
    std::string state_str() {return blinking_drone_state_names[_blinking_drone_status];}
    blinking_drone_states state() {return _blinking_drone_status;}
    cv::Point2f drone_startup_im_location(){ return _drone_blink_image_location; }

    bool init(VisionData *_visdat);
    void track(double time);
    void drone_location(cv::Point p){_drone_blink_image_location = p/IMSCALEF;}

    bool blinking_drone_located() {return _blinking_drone_status >= bds_found;}
    cv::Point3f drone_startup_location() {return _drone_blink_world_location;}

    bool delete_me(){
        return n_frames_lost > n_frames_lost_threshold;;
    }

};




#endif //BLINKTRACKER_H
