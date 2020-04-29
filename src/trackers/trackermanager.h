#pragma once
#include "defines.h"
#include "common.h"
#include "tracking.h"
#include "itemtracker.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "replaytracker.h"
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

struct processed_blobs {
    processed_blobs(BlobProps * blob, uint16_t new_id) {
        props = blob;
        id = new_id;
    }
    cv::Point2f pt() {
        return cv::Point2f(props->x,props->y);
    }
    float size() {
        return props->size;
    }
    float pixel_max() {
        return props->pixel_max;
    }
    uint16_t id;
    cv::Mat mask;
    BlobProps * props;
    std::vector<ItemTracker *> trackers;
    bool tracked() { return trackers.size()>0;}
    bool ignored = false;
    cv::Scalar color() {
        if (trackers.size() == 0 ) {
            if (ignored || props->world_props.takeoff_reject)
                return cv::Scalar(0,128,0); // dark green
            else
                return cv::Scalar(255,255,55); // light blue
        } else if (trackers.size()>1)
            return cv::Scalar(0,128,255); // orange
        ItemTracker * trkr = trackers.at(0);
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
};

/*
 * This class appoints the found image points to the drone and insect item tracker(s)
 *
 */
class TrackerManager {

public: cv::Scalar tracker_color( ItemTracker * trkr) {
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

    bool enable_viz_max_points = false; // flag for enabling the maxs visiualization
    std::vector<cv::Mat> vizs_maxs;
    bool enable_viz_diff = false; // flag for enabling the diff visiualization
    const float viz_max_points_resizef = 4.0;

    const float chance_multiplier_pixel_max = 0.5f;
    const float chance_multiplier_dist = 3;
    const float chance_multiplier_total = chance_multiplier_dist + chance_multiplier_pixel_max;

    std::vector<tracking::BlobProps> _blobs;

    void update_trackers(double time, long long frame_number, bool drone_is_active);
    void update_max_change_points();
    void update_static_ignores();
    void match_blobs_to_trackers(bool drone_is_active, double time);
    bool tracker_active(ItemTracker * trkr, bool drone_is_active);
    cv::Scalar color_of_blob(processed_blobs blob);
    std::vector<BlinkTracker *> blinktrackers();
    std::vector<InsectTracker *> insecttrackers();
    std::vector<ReplayTracker *> replaytrackers();

    void reset_trkr_viz_ids();

    uint16_t next_insecttrkr_id = 1;
    detection_mode _mode;

    DroneTracker * _dtrkr;
    InsectTracker * default_itrkr;

    std::vector<logging::InsectReader> replay_logs;
    cv::Mat diff_viz;
public:
    cv::Mat viz_max_points,diff_viz_buf;
    void mode(detection_mode m) {_mode = m;}
    detection_mode mode ()  {return _mode;}
    std::string mode_str() {
        if (_mode == mode_locate_drone) {
            int best_state = 0;
            for (auto trkr : _trackers)   {
                if (trkr->type() == tt_blink)   {
                    BlinkTracker * btrkr = static_cast<BlinkTracker * >(trkr);
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

    std::tuple<bool, BlinkTracker *> blinktracker_best();
    InsectTracker *insecttracker_best();
    DroneTracker * dronetracker() { return _dtrkr; }
    void init(ofstream *logger, VisionData *visdat, CameraView *camview);
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
        rt->init(next_insecttrkr_id,"../insect_logs/" + std::to_string(id) + ".csv",_visdat);
        _trackers.push_back(rt);
        next_insecttrkr_id++;
    }
    void init_replay_moth(std::vector<logging::InsectReader> logs) {
        replay_logs = logs;
    }
    void process_replay_moth(unsigned long long RS_id) {
        std::vector<logging::InsectReader> replay_logs_updated;
        for (auto log : replay_logs) {
            if(log.current_entry.RS_id == RS_id) {
                ReplayTracker *rt;
                rt = new ReplayTracker();
                rt->init(next_insecttrkr_id,log,_visdat);
                _trackers.push_back(rt);
                next_insecttrkr_id++;
            } else {
                replay_logs_updated.push_back(log);
            }
        }
        replay_logs = replay_logs_updated;
    }
};

}
