#ifndef REPLAYTRACKER_H
#define REPLAYTRACKER_H

#include "itemtracker.h"
#include "logreader.h"
/*
 * This class will track insects
 *
 */
class ReplayTracker : public ItemTracker {

private:
    int16_t _id{-1};
    void start_new_log_line(double time, unsigned long long frame_number,bool replay_insect);
protected:
public:
    void init(int id, VisionData *_visdat);
    void track(double time);
    void update_from_log(LogReader::Log_Entry_Insect *entry, unsigned long long frame_number, double time);
    bool tracking(){return _tracking;}
    int16_t id(){return _id;}


    bool delete_me(){

    }
};

#endif //REPLAYTRACKER_H
