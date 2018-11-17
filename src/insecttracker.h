#ifndef INSECTTRACKER_H
#define INSECTTRACKER_H

#include "itemtracker.h"
#include "logreader.h"
/*
 * This class will track insects
 *
 */
class InsectTracker : public ItemTracker {


private:
    uint drone_still_active;
protected:
    void init_settings();
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size);

public:
    void init(std::ofstream *logger, VisionData *_visdat);
    void track(float time, std::vector<track_item>  ignore, bool drone_is_active);
    void update_from_log(LogReader::Log_Entry entry, int frame_number);

};




#endif //INSECTTRACKER_H
