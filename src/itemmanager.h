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

    struct processed_max_point {
        processed_max_point(cv::KeyPoint k){
            pt.x = k.pt.x;
            pt.y = k.pt.y;
            size = k.size;
        }
        cv::Point2f pt;
        float size;
        bool tracked = false;
        cv::Scalar color = {255,0,255};
    };

    struct item_manager_settings{

        int min_disparity=1;
        int max_disparity=20;

        int roi_min_size = 200;
        int roi_max_grow = 160;
        int roi_grow_speed = 64;

        int exclude_min_distance = 5; // in res/IMSCALEF resolution
        int exclude_max_distance = 50; // in res/IMSCALEF resolution

        int static_ignores_dist_thresh = 15; // in res/IMSCALEF resolution

        int max_points_per_frame = 10;
        int radius = 15;
        int motion_thresh = 10;
        int motion_thresh_blink_detect = 30;

        int background_subtract_zone_factor = 90;

        //only for dronetracker:
        int pixel_dist_landing_spot = 0;
        int pixel_dist_seperation_min = 2;
        int pixel_dist_seperation_max = 5;

        float version = 2.0f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version,min_disparity,max_disparity,roi_min_size,
               roi_max_grow,roi_grow_speed,exclude_min_distance,
               exclude_max_distance,background_subtract_zone_factor,
               max_points_per_frame,radius,motion_thresh_blink_detect,
               motion_thresh,static_ignores_dist_thresh,
               pixel_dist_landing_spot,pixel_dist_seperation_min,
               pixel_dist_seperation_max);
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
    bool enable_viz_diff = false;

    std::vector<cv::KeyPoint> _kps;

    void update_trackers(double time, LogReader::Log_Entry log_entry, bool drone_is_active);
    void update_max_change_points();
    void update_static_ignores();
    void match_image_points_to_trackers(bool drone_is_active);
    bool tracker_active(ItemTracker * trkr, bool drone_is_active);
    cv::Scalar tracker_color(ItemTracker * trkr);
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
