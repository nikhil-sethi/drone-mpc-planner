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
    cv::Mat Qfi;

    bool enable_viz_diff = false;
protected:
    void init_settings();
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL_small, cv::Point size);
    void update_insect_prediction();

public:
    void init(std::ofstream *logger, VisionData *_visdat);
    void track(float time, std::vector<track_item>  ignore, std::vector<cv::Point2f> additional_ignores);
    void update_from_log(LogReader::Log_Entry entry, int frame_number);

    cv::Mat diff_viz;
};




#endif //INSECTTRACKER_H
