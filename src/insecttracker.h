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
public:
    bool init(std::ofstream *logger, VisionData *visdat);
    bool track(float time, cv::Point3f setpoint_world, std::vector<track_item>  ignore);

};




#endif //INSECTTRACKER_H
