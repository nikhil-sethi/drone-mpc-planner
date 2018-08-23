#ifndef DRONETRACKER_H
#define DRONETRACKER_H

#include "itemtracker.h"

/*
 * This class will track a micro drone with leds
 *
 */
class DroneTracker : public ItemTracker {


private:

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

};




#endif //DRONETRACKER_H
