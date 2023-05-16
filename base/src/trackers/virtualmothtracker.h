#pragma once
#include "insecttracker.h"
#include "insectreader.h"
#include "dronecontroller.h"

namespace tracking {

class VirtualMothTracker : public InsectTracker {
public:

    enum trigger_type {
        drone_spinup,
        hunt_error,
        number_trigger_types, //must be last element
    };

    enum evasion_type {
        diving,
        u_turn,
        no_reaction,
        number_evasion_types, //must be last element

    };

    struct moth_behavior_type {
        trigger_type trigger;
        evasion_type evasion;

        moth_behavior_type(): trigger(drone_spinup), evasion(diving) {}
        moth_behavior_type(trigger_type tr, evasion_type ev): trigger(tr), evasion(ev) {};
    };

private:
    int16_t _id{-1};
    logging::InsectReader logreader;
    void start_new_log_line(double time, unsigned long long frame_number);
    cv::Point3f insect_pos;
    cv::Point3f insect_vel;
    cv::Point3f insect_acc;
    DroneController *_dctrl;
    bool _delete_me = false;
    double start_time = 0;
    bool escape_triggered = false;
    moth_behavior_type _moth_behavior;
    double time_escape_triggered = -1;
    double duration_escape_turn = 0.27;


public:
    moth_behavior_type init(int id, moth_behavior_type mothbehavior, VisionData *visdat, DroneController *dctrl);
    void init_logger();
    bool check_ignore_blobs(BlobProps *pbs [[maybe_unused]]) {return false;}
    [[maybe_unused]] void match_template() {};
    void calc_world_item(BlobProps *pbs, double time [[maybe_unused]]) {pbs->world_props.valid = false;}
    void update(double time);
    void update_behavior_based(unsigned long long frame_number, double time);
    bool delete_me();

    bool tracking() {return _tracking;}
    int16_t id() {return _id;}
    tracker_type type() {return tt_virtualmoth;}
};

}
