#pragma once
#include "common.h"
#include "tracking.h"
#include "itemtracker.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "replaytracker.h"
#include "virtualmothtracker.h"
#include "blinktracker.h"
#include "visiondata.h"
#include "common.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
class Interceptor;
namespace tracking {

static const char *trackermanager_mode_names[] = {"tm_idle",
                                                  "tm_locate_drone",
                                                  "tm_wait_for_insect",
                                                  "tm_drone_only",
                                                  "tm_hunt",
                                                  "tm_hunt_replay_moth"
                                                 };

class TrackerManager {

public:
    enum detection_mode {
        mode_idle,
        mode_locate_drone,
        mode_wait_for_insect,
        mode_drone_only,
        mode_hunt
    };

private:
    enum preselector_roi_states {
        roi_state_drone = 0,
        roi_state_blink,
        roi_state_prior_insects,
        roi_state_no_prior
    };

    struct ProcessedBlob {
        ProcessedBlob(BlobProps *blob, uint16_t new_id) {
            props = blob;
            id = new_id;
        }
        cv::Point2f pt() {return cv::Point2f(props->x, props->y);} // scaled with pparams.imscalef
        float size() {return props->size;}  // scaled with pparams.imscalef
        uint8_t pixel_max() {return props->pixel_max;}
        uint8_t motion_noise() {return props->motion_noise;}
        uint16_t id;
        cv::Mat mask;
        BlobProps *props;
        std::vector<ItemTracker *> trackers;
        bool tracked() { return trackers.size() > 0;}
        bool ignored = false;
        cv::Scalar color() {
            if (trackers.size() == 0) {
                if (ignored || props->world_props.takeoff_reject)
                    return cv::Scalar(0, 128, 0); // dark green
                else if (props->in_overexposed_area)
                    return cv::Scalar(128, 128, 128); // gray
                else
                    return cv::Scalar(255, 255, 55); // light blue
            } else if (trackers.size() > 1)
                return cv::Scalar(0, 128, 255); // orange
            ItemTracker *trkr = trackers.at(0);
            if (trkr->type() == tt_drone)
                return cv::Scalar(0, 255, 0); // green
            else if (trkr->type() == tt_insect) {
                if (static_cast<InsectTracker *>(trkr)->false_positive())
                    return cv::Scalar(255, 0, 0); //blue
                else
                    return cv::Scalar(0, 0, 255); // red
            } else if (trkr->type() == tt_replay)
                return cv::Scalar(0, 0, 180); // dark red
            else if (trkr->type() == tt_blink)
                return cv::Scalar(255, 0, 255); // pink
            return cv::Scalar(0, 0, 0);
        }
        std::string prefix() {
            if (trackers.size() == 0) {
                if (ignored || props->world_props.takeoff_reject)
                    return "";
                else if (props->in_overexposed_area)
                    return "e";
                else
                    return "";
            } else if (trackers.size() > 1)
                return "";
            ItemTracker *trkr = trackers.at(0);
            if (trkr->type() == tt_drone)
                return "d ";
            else if (trkr->type() == tt_insect)
                return "m ";
            else if (trkr->type() == tt_replay)
                return "r";
            else if (trkr->type() == tt_blink)
                return "b ";
            return "";
        }
    };


    class TrackerManagerParameters: public xmls::Serializable
    {
    public:
        xmls::xInt max_points_per_frame, roi_radius, motion_thresh;

        TrackerManagerParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("TrackerManagerParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("max_points_per_frame", &max_points_per_frame);
            Register("roi_radius", &roi_radius);
            Register("motion_thresh", &motion_thresh);
        }
    };

    const string settings_file = "../xml/trackermanager.xml";
    const string false_positive_fn = "false_positives.csv";

    bool initialized = false;
    detection_mode _mode = mode_idle;
    std::string replay_dir;

    std::ofstream *_logger;
    VisionData *_visdat;
    FlightArea *_flight_area;
    Interceptor *_iceptor;

    std::vector<FalsePositive> false_positives;
    std::vector<tracking::BlobProps> _blobs;

    std::vector<ItemTracker *> _trackers;
    DroneTracker *_dtrkr;
    std::vector<BlinkTracker *> blinktrackers();
    std::vector<InsectTracker *> insecttrackers();
    std::vector<ReplayTracker *> replaytrackers();
    std::vector<VirtualMothTracker *> virtualmothtrackers();
    std::vector<logging::InsectReader> replay_logs;

    uint16_t next_insecttrkr_id = 1;
    uint16_t next_blinktrkr_id = 1;

    cv::Mat diff_viz;
    bool _enable_viz_blob = false;
    bool _enable_viz_tracker = false;
    bool enable_viz_motion = false;
    bool _enable_draw_stereo_viz = false;
    double time_since_tracking_nothing = 0;
    double time_since_monsters = 0;
    bool _monster_alert = false;
    int _fp_monsters_count = 0;
    int _fp_statics_count = 0;
    int _fp_shorts_count = 0;
    int _insects_count = 0;

    std::vector<cv::Mat> vizs_blobs;
    const float viz_blobs_resizef = 4.0;

    int max_points_per_frame;
    int default_roi_radius = 15; //this is the expected im size of the drone at the charging pad
    int motion_thresh;
    const float chance_multiplier_pixel_max = 0.5f;
    const float chance_multiplier_dist = 3;

    void update_trackers(double time, long long frame_number, bool drone_is_active);
    void find_blobs();
    void collect_static_ignores();
    void collect_static_ignores(ItemTracker *trkr);
    void check_match_conflicts(std::vector<ProcessedBlob> *pbs, double time);
    void flag_used_static_ignores(std::vector<ProcessedBlob> *pbs);
    void prep_blobs(std::vector<ProcessedBlob> *pbs, double time);
    void erase_dissipated_fps(double time);
    void match_existing_trackers(std::vector<ProcessedBlob> *pbs, bool drone_is_active, double time);
    void rematch_drone_tracker(std::vector<ProcessedBlob> *pbs, bool drone_is_active, double time);
    void rematch_blink_tracker(std::vector<ProcessedBlob> *pbs, double time);
    void create_new_insect_trackers(std::vector<ProcessedBlob> *pbs, double time);
    void create_new_blink_trackers(std::vector<ProcessedBlob> *pbs, double time);
    void match_blobs_to_trackers(bool drone_is_active, double time);
    std::tuple<float, bool, float> tune_detection_radius(cv::Point maxt);
    void floodfind_and_remove(cv::Point maxt, uint8_t max, uint8_t motion_noise, cv::Mat diff, cv::Mat motion_filtered_noise_mapL);

    bool tracker_active(ItemTracker *trkr, bool drone_is_active);

    void prep_vizs();
    void draw_motion_viz(std::vector<ProcessedBlob> *pbs, double time);
    void draw_blob_viz(std::vector<ProcessedBlob> *pbs);
    std::tuple<cv::Rect, cv::Rect, int> calc_trkr_viz_roi(ImageItem image_item);
    void reset_trkr_viz_ids();
    void draw_trackers_viz();
    void finish_vizs();
    cv::Scalar color_of_blob(ProcessedBlob blob);

    void read_false_positives();
    void save_false_positives();
    void save_false_positives(std::string false_positive_wfn);
    void deserialize_settings();
    void serialize_settings();

public:
    cv::Mat viz_max_points, diff_viz_buf, viz_trkrs_buf;

    void init(ofstream *logger, string replay_dir, VisionData *visdat, Interceptor *iceptor);

    void init_replay_moth(std::string file) {
        ReplayTracker *rt;
        rt = new ReplayTracker();
        rt->init(next_insecttrkr_id, file, _visdat);
        _trackers.push_back(rt);
        next_insecttrkr_id++;
    }
    void init_replay_moth(int id) {
        ReplayTracker *rt;
        rt = new ReplayTracker();
        if (pparams.fps == 90)
            rt->init(next_insecttrkr_id, "../replay_insects/" + std::to_string(id) + "-90fps.csv", _visdat);
        else
            rt->init(next_insecttrkr_id, "../replay_insects/" + std::to_string(id) + ".csv", _visdat);
        _trackers.push_back(rt);
        next_insecttrkr_id++;
    }
    void init_replay_moth(std::vector<logging::InsectReader> logs) { replay_logs = logs; }
    void process_replay_moth(unsigned long long rs_id) {
        replay_logs.erase(
            std::remove_if(
                replay_logs.begin(),
                replay_logs.end(),
        [&](auto log) {
            if (log.current_entry.rs_id == rs_id) {
                ReplayTracker *rt;
                rt = new ReplayTracker();
                rt->init(next_insecttrkr_id, log, _visdat);
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
    void init_virtual_moth(tracking::VirtualMothTracker::mothbehavior behavior_type, DroneController *dctrl) {
        VirtualMothTracker *vt;
        vt = new VirtualMothTracker();
        vt->init(next_insecttrkr_id, behavior_type, _visdat, dctrl);
        _trackers.push_back(vt);
        next_insecttrkr_id++;
    }
    void update(double time, bool drone_is_active);
    void close();

    void mode(detection_mode m) {_mode = m;}
    detection_mode mode()  {return _mode;}
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
            if (best_state > 0)
                return blinking_drone_state_names[best_state];
            else
                return trackermanager_mode_names[_mode];
        } else
            return trackermanager_mode_names[_mode];
    }
    bool too_many_false_positives() {return false_positives.size() > 20;}
    int detections_count() {return next_insecttrkr_id - 1;}
    int fp_monsters_count() {return _fp_monsters_count;}
    int fp_statics_count() {return _fp_statics_count;}
    int fp_shorts_count() {return _fp_shorts_count;}
    int insects_count() {return _insects_count;}
    bool monster_alert() { return _monster_alert;}

    std::tuple<bool, BlinkTracker *> blinktracker_best();
    std::vector<ItemTracker *>all_target_trackers();
    DroneTracker *dronetracker() { return _dtrkr; }
    float tracking_anything_duration(double time) {
        for (auto trkr : _trackers) {
            if (trkr->n_frames_tracking() > 0)
                return static_cast<float>(time - time_since_tracking_nothing);
        }
        time_since_tracking_nothing = time;
        return 0;
    }

    void enable_draw_stereo_viz() {
        _enable_draw_stereo_viz = true;
        for (auto trkr : _trackers) {
            trkr->enable_draw_stereo_viz = true;
        }
    }
    void enable_blob_viz() { _enable_viz_blob = true;}
    void enable_trkr_viz() { _enable_viz_tracker = true;}
    cv::Scalar tracker_color(ItemTracker *trkr) {
        if (trkr->type() == tt_drone)
            return cv::Scalar(0, 255, 0);
        else if (trkr->type() == tt_insect)
            return cv::Scalar(0, 0, 255);
        else if (trkr->type() == tt_blink)
            return cv::Scalar(255, 0, 255);
        return cv::Scalar(0, 0, 0);
    }
};
}
