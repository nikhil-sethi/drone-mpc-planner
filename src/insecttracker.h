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

    void make_image_item_size_invald() {_image_item.size = -1;} // flag the smoother_im_size to not use this value
    bool check_ignore_blobs(BlobProps * pbs, uint id);
    bool delete_me(){
        return false;
    }
};




#endif //INSECTTRACKER_H
