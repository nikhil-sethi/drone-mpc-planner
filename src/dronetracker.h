#ifndef DRONETRACKER_H
#define DRONETRACKER_H

#include "itemtracker.h"

/*
 * This class will track a micro drone with leds
 *
 */
class DroneTracker : public ItemTracker {

    const float bind_blink_time = 0.5f;
private:
    enum blinking_drone_state {
        bds_none = 0,
        bds_start=1,
        bds_resetting_background=2,
        bds_searching=3,
        bds_blink_off=4,
        bds_blink_on=5,
        bds_2nd_blink_off=6,
        bds_2nd_blink_on=7,
        bds_3th_blink_off=8,
        bds_3th_blink_on=9,
        bds_found=10
    };
    blinking_drone_state _blinking_drone_located = bds_none;
    float blink_time_start = 0;

    enum drone_tracking_state {
        dts_initialize_blink_locater = 0,
        dts_blinking = 1,
        dts_inactive = 2,
        dts_active_detecting = 3,
        dts_found_after_takeoff=4
    };
    drone_tracking_state _drone_tracking_state = dts_inactive;

    blinking_drone_state detect_blink(float time, bool found);
    cv::KeyPoint blink_location;

protected:
    cv::Mat get_probability_cloud(cv::Point size);
    void init_settings();
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size);
public:

    cv::Mat _cir;
    float drone_max_border_z = MAX_BORDER_Z_DEFAULT;
    float drone_max_border_y = MAX_BORDER_Y_DEFAULT;

    bool init(std::ofstream *logger, VisionData *_visdat);
    void track(float time, std::vector<track_item> ignore, bool drone_is_active);

    void Locate_Startup_Location() {
        _drone_tracking_state = dts_blinking;
        _blinking_drone_located = bds_start;
    }
    bool blinking_drone_located() {
        return _blinking_drone_located == bds_found;
    }
};




#endif //DRONETRACKER_H
