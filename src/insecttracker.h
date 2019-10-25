#ifndef INSECTTRACKER_H
#define INSECTTRACKER_H

#include "itemtracker.h"
#include "logreader.h"
/*
 * This class will track insects
 *
 */
class InsectTracker : public ItemTracker {

protected:
    void update_insect_prediction();
public:
    void init(std::ofstream *logger, VisionData *_visdat);
    void track(double time);
    void update_from_log(LogReader::Log_Entry entry, int frame_number, double time);
    void reset_after_log(){
        _tracking = false;
        _world_item.valid = false;
        _image_item.valid = false;
    }
    bool tracking(){return _tracking;}
    bool properly_tracking(){
        return n_frames_tracking > n_frames_lost && tracking();
    }

    BlobWorldProps calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs, double time);
    bool delete_me(){
        return false;
    }
};

#endif //INSECTTRACKER_H
