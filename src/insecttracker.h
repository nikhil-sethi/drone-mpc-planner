#ifndef INSECTTRACKER_H
#define INSECTTRACKER_H

#include "itemtracker.h"
/*
 * This class will track insects
 *
 */
class InsectTracker : public ItemTracker {


private:

protected:
    void init_settings();
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size);
public:
    void init(std::ofstream *logger, VisionData *_visdat);
    void track(float time, cv::Point3f setpoint_world, std::vector<track_item>  ignore, bool drone_is_active);

};




#endif //INSECTTRACKER_H
