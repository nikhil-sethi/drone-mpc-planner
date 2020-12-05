#pragma once
#include "common.h"
#include "tracking.h"
#include "itemtracker.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "replaytracker.h"
#include "virtualmothtracker.h"
#include "blinktracker.h"
#include "visiondata.h"
#include "common.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace tracking {

static const char* trackermanager_mode_names[] = {"tm_idle",
                                                  "tm_locate_drone",
                                                  "tm_wait_for_insect",
                                                  "tm_drone_only",
                                                  "tm_hunt",
                                                  "tm_hunt_replay_moth"
                                                 };

struct ProcessedBlob {
    ProcessedBlob(BlobProps *blob, uint16_t new_id) {
        props = blob;
        id = new_id;
    }
    cv::Point2f pt() { // scaled with pparams.imscalef
        return cv::Point2f(props->x,props->y);
    }
    float size() { // scaled with pparams.imscalef
        return props->size;
    }
    float pixel_max() {
        return props->pixel_max;
    }
    uint16_t id;
    cv::Mat mask;
    BlobProps *props;
    std::vector<ItemTracker *> trackers;
    bool tracked() { return trackers.size()>0;}
    bool ignored = false;
    cv::Scalar color() {
        if (trackers.size() == 0 ) {
            if (ignored || props->world_props.takeoff_reject)
                return cv::Scalar(0,128,0); // dark green
            else if (props->in_overexposed_area)
                return cv::Scalar(128,128,128);  // gray
            else
                return cv::Scalar(255,255,55); // light blue
        } else if (trackers.size()>1)
            return cv::Scalar(0,128,255); // orange
        ItemTracker *trkr = trackers.at(0);
        if (trkr->type() == tt_drone)
            return cv::Scalar(0,255,0); // green
        else if (trkr->type() == tt_insect)
            return cv::Scalar(0,0,255); // red
        else if (trkr->type() == tt_replay)
            return cv::Scalar(0,0,180); // dark red
        else if (trkr->type() == tt_blink)
            return cv::Scalar(255,0,255); // pink
        return cv::Scalar(0,0,0);
    }
    std::string prefix() {
        if (trackers.size() == 0 ) {
            if (ignored || props->world_props.takeoff_reject)
                return "";
            else if (props->in_overexposed_area)
                return "E";
            else
                return "";
        } else if (trackers.size()>1)
            return "";
        ItemTracker *trkr = trackers.at(0);
        if (trkr->type() == tt_drone)
            return "D ";
        else if (trkr->type() == tt_insect)
            return "M ";
        else if (trkr->type() == tt_replay)
            return "";
        else if (trkr->type() == tt_blink)
            return "B ";
        return "";
    }
};

class TrackerManager {

public: cv::Scalar tracker_color( ItemTracker *trkr) {
        if (trkr->type() == tt_drone)
            return cv::Scalar(0,255,0);
        else if (trkr->type() == tt_insect)
            return cv::Scalar(0,0,255);
        else if (trkr->type() == tt_blink)
            return cv::Scalar(255,0,255);
        return cv::Scalar(0,0,0);
    }


private:
    class TrackerManagerParameters: public xmls::Serializable
    {
    public:
        xmls::xInt max_points_per_frame,roi_radius,motion_thresh;

        TrackerManagerParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("TrackerManagerParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("max_points_per_frame",&max_points_per_frame);
            Register("roi_radius",&roi_radius);
            Register("motion_thresh",&motion_thresh);
        }
    };

    int max_points_per_frame;
    int roi_radius;
    int motion_thresh;

    string settings_file = "../../xml/trackermanager.xml";
    void deserialize_settings();
    void serialize_settings();

    const string false_positive_fn = "false_positives.csv";
    void read_false_positives();
    void save_false_positives();
    void save_false_positives(std::string false_positive_wfn);

public:
    enum detection_mode {
        mode_idle,
        mode_locate_drone,
        mode_wait_for_insect,
        mode_drone_only,
        mode_hunt
    };


private:
    std::vector<ItemTracker *> _trackers;
    std::ofstream *_logger;
    VisionData *_visdat;
    CameraView *_camview;
    bool initialized = false;
    std::string replay_dir;

    std::vector<FalsePositive> false_positives;

    bool enable_viz_blobs = false;
    bool enable_viz_trackers = false;
    bool enable_viz_motion = false;
    std::vector<cv::Mat> vizs_maxs;
    const float viz_max_points_resizef = 4.0;

    const float chance_multiplier_pixel_max = 0.5f;
    const float chance_multiplier_dist = 3;
    const float chance_multiplier_total = chance_multiplier_dist + chance_multiplier_pixel_max;

    std::vector<tracking::BlobProps> _blobs;

    void update_trackers(double time, long long frame_number, bool drone_is_active);
    void find_blobs();
    void collect_static_ignores();
    void collect_static_ignores(ItemTracker *trkr);
    void check_match_conflicts(std::vector<ProcessedBlob> *pbs,double time);
    void flag_used_static_ignores(std::vector<ProcessedBlob> *pbs);
    void prep_blobs(std::vector<ProcessedBlob> *pbs,double time);
    void erase_dissipated_fps(double time);
    void match_existing_trackers(std::vector<ProcessedBlob> *pbs,bool drone_is_active, double time);
    void rematch_drone_tracker(std::vector<ProcessedBlob> *pbs,bool drone_is_active, double time);
    void rematch_blink_tracker(std::vector<ProcessedBlob> *pbs, double time);
    void create_new_insect_trackers(std::vector<ProcessedBlob> *pbs, double time);
    void create_new_blink_trackers(std::vector<ProcessedBlob> *pbs, double time);
    void draw_viz(std::vector<ProcessedBlob> *pbs, double time);
    void match_blobs_to_trackers(bool drone_is_active, double time);
    void find_cog_and_remove(cv::Point maxt, double max, cv::Mat diff,bool enable_insect_drone_split, float drn_ins_split_thresh,cv::Mat bkg_frame);
    bool tracker_active(ItemTracker *trkr, bool drone_is_active);
    void prep_vizs();
    std::tuple<cv::Rect,cv::Rect,int> calc_trkr_viz_roi(ImageItem iti);
    void finish_vizs();
    cv::Scalar color_of_blob(ProcessedBlob blob);
    std::vector<BlinkTracker *> blinktrackers();
    std::vector<InsectTracker *> insecttrackers();
    std::vector<ReplayTracker *> replaytrackers();
    std::vector<VirtualmothTracker*> virtualmothtrackers();

    void reset_trkr_viz_ids();

    uint16_t next_insecttrkr_id = 1;
    uint16_t next_blinktrkr_id = 1;
    detection_mode _mode;

    DroneTracker *_dtrkr;
    unsigned long long target_insecttracker_frameid_updated = 0;
    InsectTracker *_target_insecttracker;

    std::vector<logging::InsectReader> replay_logs;
    cv::Mat diff_viz;
public:
    cv::Mat viz_max_points,diff_viz_buf,viz_trkrs_buf;

    void enable_blob_viz() { enable_viz_blobs = true;}
    void enable_trkr_viz() { enable_viz_trackers = true;}
    void mode(detection_mode m) {_mode = m;}
    detection_mode mode ()  {return _mode;}
    std::string mode_str() {
        if (_mode == mode_locate_drone) {
            int best_state = 0;
            for (auto trkr : _trackers)   {
                if (trkr->type() == tt_blink)   {
                    BlinkTracker *btrkr = static_cast<BlinkTracker *>(trkr);
                    if (btrkr->state() > best_state)
                        best_state = btrkr->state();
                }
            }
            if (best_state>0)
                return blinking_drone_state_names[best_state];
            else
                return trackermanager_mode_names[_mode];
        } else
            return trackermanager_mode_names[_mode];
    }
    bool too_many_false_positives() {return false_positives.size()>20;}

    std::tuple<bool, BlinkTracker *> blinktracker_best();

    std::vector<ItemTracker *>all_target_trackers();
    InsectTracker *target_insecttracker();
    bool tracking_a_target();
    double target_last_detection();
    TrackData target_last_trackdata();
    DroneTracker *dronetracker() { return _dtrkr; }
    void init(ofstream *logger,string replay_dir, VisionData *visdat, CameraView *camview);
    void update(double time, bool drone_is_active);
    void close();

    void init_replay_moth(std::string file) {
        ReplayTracker *rt;
        rt = new ReplayTracker();
        rt->init(next_insecttrkr_id,file,_visdat);
        _trackers.push_back(rt);
        next_insecttrkr_id++;
    }
    void init_replay_moth(int id) {
        ReplayTracker *rt;
        rt = new ReplayTracker();
        if(pparams.fps == 90)
            rt->init(next_insecttrkr_id,"../insect_logs/" + std::to_string(id) + "-90fps.csv",_visdat);
        else
            rt->init(next_insecttrkr_id,"../insect_logs/" + std::to_string(id) + ".csv",_visdat);
        _trackers.push_back(rt);
        next_insecttrkr_id++;
    }
    void init_replay_moth(std::vector<logging::InsectReader> logs) {
        replay_logs = logs;
    }
    void process_replay_moth(unsigned long long rs_id) {

        replay_logs.erase(
            std::remove_if(
                replay_logs.begin(),
                replay_logs.end(),
        [&](auto log) {
            if ( log.current_entry.rs_id == rs_id) {
                ReplayTracker *rt;
                rt = new ReplayTracker();
                rt->init(next_insecttrkr_id,log,_visdat);
                _trackers.push_back(rt);
                next_insecttrkr_id++;
                return true;
            }
            return false;
        }
            ),
        replay_logs.end()
        );
    }

    void init_virtualmoth_moth(tracking::mothbehavior behavior_type, DroneController*dctrl) {
        VirtualmothTracker *vt;
        vt = new VirtualmothTracker();
        vt->init(next_insecttrkr_id, behavior_type, _visdat, dctrl);
        _trackers.push_back(vt);
        next_insecttrkr_id++;
    }
};

}
