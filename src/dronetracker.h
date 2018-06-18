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
public:       

    cv::Mat cir;
    float drone_max_border_z = MAX_BORDER_Z_DEFAULT;
    float drone_max_border_y = MAX_BORDER_Y_DEFAULT;

    bool init(std::ofstream *logger, VisionData *visdat);
    void track(float time, cv::Point3f setpoint_world, std::vector<track_item> ignore);

};




#endif //DRONETRACKER_H
