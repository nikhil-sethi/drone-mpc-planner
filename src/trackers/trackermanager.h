#pragma once
#include "defines.h"
#include "common.h"
#include "itemtracker.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "replaytracker.h"
#include "blinktracker.h"
#include "visiondata.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char* trackermanager_mode_names[] = {"tm_idle",
                                                  "tm_locate_drone",
                                                  "tm_wait_for_insect",
                                                  "tm_drone_only",
                                                  "tm_hunt",
                                                  "tm_hunt_replay_moth"};

/*
 * This class appoints the found image points to the drone and insect item tracker(s)
 *
 */
class TrackerManager {

public: cv::Scalar tracker_color( ItemTracker * trkr) {
        if (typeid(*trkr) == typeid(DroneTracker))
            return cv::Scalar(0,255,0);
        else if (typeid(*trkr) == typeid(InsectTracker))
            return cv::Scalar(0,0,255);
        else if (typeid(*trkr) == typeid(BlinkTracker))
            return cv::Scalar(255,0,255);
        return cv::Scalar(0,0,0);
    }

    struct processed_blobs {
        processed_blobs(ItemTracker::BlobProps * blob){
            props = blob;
        }
        cv::Point2f pt() {
            return cv::Point2f(props->x,props->y);
        }
        float size() {
            return props->radius;
        }
        float pixel_max() {
            return props->pixel_max;
        }
        cv::Mat mask;
        ItemTracker::BlobProps * props;
        std::vector<ItemTracker *> trackers;
        bool tracked() { return trackers.size()>0;}
        bool ignored = false;
        cv::Scalar color() {
            if (trackers.size() == 0 ){
                if (ignored)
                    return cv::Scalar(0,128,0); // dark green
                else
                    return cv::Scalar(255,255,55); // light blue
            } else if (trackers.size()>1)
                return cv::Scalar(200,255,250);
            ItemTracker * trkr = trackers.at(0);
            if (typeid(*trkr) == typeid(DroneTracker))
                return cv::Scalar(0,255,0); // green
            else if (typeid(*trkr) == typeid(InsectTracker))
                return cv::Scalar(0,0,255); // red
            else if (typeid(*trkr) == typeid(ReplayTracker))
                return cv::Scalar(0,0,180); // dark red
            else if (typeid(*trkr) == typeid(BlinkTracker))
                return cv::Scalar(255,0,255); // pink
            return cv::Scalar(0,0,0);
        }
    };

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
    enum detection_mode{
        mode_idle,
        mode_locate_drone,
        mode_wait_for_insect,
        mode_drone_only,
        mode_hunt
    };

private:
    std::vector<ItemTracker *> _trackers;
    std::ofstream *_logger;
    VisionData * _visdat;
    bool initialized = false;

    bool enable_viz_max_points = false; // flag for enabling the maxs visiualization
    std::vector<cv::Mat> vizs_maxs;
    bool enable_viz_diff = false; // flag for enabling the diff visiualization

    const float chance_multiplier_pixel_max = 0.5f;
    const float chance_multiplier_dist = 3;
    const float chance_multiplier_total = chance_multiplier_dist + chance_multiplier_pixel_max;

    std::vector<ItemTracker::BlobProps> _blobs;

    void update_trackers(double time, long long frame_number, bool drone_is_active);
    void update_max_change_points();
    void update_static_ignores();
    void match_blobs_to_trackers(bool drone_is_active, double time);
    bool tracker_active(ItemTracker * trkr, bool drone_is_active);
    std::vector<InsectTracker *> insecttrackers();
    std::vector<ReplayTracker *> replaytrackers();

    uint16_t next_insecttrkr_id = 1;
    detection_mode _mode;

    DroneTracker * _dtrkr;
    InsectTracker * default_itrkr;

    std::vector<logging::InsectReader> replay_logs;
public:
    cv::Mat viz_max_points,diff_viz;
    void mode(detection_mode m){_mode = m;}
    detection_mode mode ()  {return _mode;}
    std::string mode_str() {
        if (_mode == mode_locate_drone) {
            int best_state = 0;
            for (uint i = 0; i < _trackers.size(); i++)   {
                if (typeid(*_trackers.at(i)) == typeid(BlinkTracker))   {
                    BlinkTracker * btrkr = static_cast<BlinkTracker * >(_trackers.at(i));
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

    InsectTracker *insecttracker_best();
    DroneTracker * dronetracker(){ return _dtrkr; }
    void init(ofstream *logger, VisionData *visdat);
    void update(double time, bool drone_is_active);
    void close();

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
