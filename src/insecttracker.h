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
    }

    BlobWorldProps calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs, uint id);
    bool delete_me(){
        return false;
    }
};

#endif //INSECTTRACKER_H
