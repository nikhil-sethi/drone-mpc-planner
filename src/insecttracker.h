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
    cv::Mat Qfi;

protected:
    void init_settings();
    void update_insect_prediction();

public:
    void init(std::ofstream *logger, VisionData *_visdat);
    void track(double time);
    void update_from_log(LogReader::Log_Entry entry, int frame_number);

    bool delete_me(){
        return false;
    }
};




#endif //INSECTTRACKER_H
