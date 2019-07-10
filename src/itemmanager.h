#ifndef ITEMMANAGER_H
#define ITEMMANAGER_H
#include "defines.h"
#include "common.h"
#include "itemtracker.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "blinktracker.h"
#include "visiondata.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char* itemmanager_mode_names[] = { "im_idle",
                                                "im_locate_drone",
                                                "im_wait_for_insect",
                                                "im_drone_only",
                                                "im_hunt",
                                                "im_hunt_replay_moth"};

/*
 * This class appoints the found image points to the drone and insect item tracker(s)
 *
 */
class ItemManager {


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
        processed_blobs(ItemTracker::BlobProps blob){
            pt.x = blob.x;
            pt.y = blob.y;
            size = blob.size;
            pixel_max = blob.pixel_max;
        }
        cv::Point2f pt;
        float size, pixel_max;

        std::vector<ItemTracker *> trackers;
        bool tracked() { return trackers.size()>0;}
        bool ignored = false;
        cv::Scalar color() {
            if (ignored)
                cv::Scalar(0,128,0);
            if (trackers.size() == 0 )
               return cv::Scalar(255,255,55);
            else if (trackers.size()>1)
               return cv::Scalar(200,255,250);
            ItemTracker * trkr = trackers.at(0);
            if (typeid(*trkr) == typeid(DroneTracker))
                return cv::Scalar(0,255,0);
            else if (typeid(*trkr) == typeid(InsectTracker))
                return cv::Scalar(0,0,255);
            else if (typeid(*trkr) == typeid(BlinkTracker))
                return cv::Scalar(255,0,255);
            return cv::Scalar(0,0,0);
        }

    };

    struct item_manager_settings{

        int min_disparity=1;
        int max_disparity=20;

        int static_ignores_dist_thresh = 15; // in res/IMSCALEF resolution

        int max_points_per_frame = 10;
        int radius = 15;
        int motion_thresh = 10;
        int motion_thresh_blink_detect = 30;

        int background_subtract_zone_factor = 90;
        float version = 1.0f;

        template <class Archive>
        void serialize( Archive & ar ) {
            ar(version,min_disparity,max_disparity,
               background_subtract_zone_factor,
               max_points_per_frame,radius,motion_thresh_blink_detect,
               motion_thresh,static_ignores_dist_thresh
               );
        }

    };
    std::string _settingsFile;

public:
    enum detection_mode{
        mode_idle,
        mode_locate_drone,
        mode_wait_for_insect,
        mode_drone_only,
        mode_hunt,
        mode_hunt_replay_moth
    };

private:
    std::vector<ItemTracker *> _trackers;
    std::ofstream *_logger;
    VisionData * _visdat;
    bool initialized = false;
    item_manager_settings settings;
    bool _enable_motion_background_check = true;

    bool enable_viz_max_points = false; // flag for enabling the maxs visiualization
    std::vector<cv::Mat> vizs_maxs;
    bool enable_viz_diff = false; // flag for enabling the diff visiualization

    std::vector<ItemTracker::BlobProps> _blobs;

    void update_trackers(double time, LogReader::Log_Entry log_entry, bool drone_is_active);
    void update_max_change_points();
    void update_static_ignores();
    void match_blobs_to_trackers(bool drone_is_active);
    bool tracker_active(ItemTracker * trkr, bool drone_is_active);
    bool check_ignore_blobs(processed_blobs pbs, ItemTracker * trkr);
    detection_mode _mode;

    InsectTracker * _itrkr;   //tmp
    DroneTracker * _dtrkr;   //tmp
public:

    cv::Mat viz_max_points,diff_viz;
    void mode(detection_mode m){ _mode = m;}
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
            return blinking_drone_state_names[best_state];
        } else
            return itemmanager_mode_names[_mode];
    }

    //tmp:
    InsectTracker * insecttracker(){
        return _itrkr;
    }
    DroneTracker * dronetracker(){
        return _dtrkr;
    }
    void init(ofstream *logger, VisionData *visdat);
    void update(double time, LogReader::Log_Entry log_entry, bool drone_is_active);
    void close();

};

#endif //ITEMMANAGER_H
