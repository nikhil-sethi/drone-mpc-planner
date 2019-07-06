#ifndef ITEMTRACKER_H
#define ITEMTRACKER_H
#include "defines.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "smoother2.h"
#include "smoother.h"

#include "common.h"
#include "visiondata.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>


/*
 * This class tracks one item
 *
 */
class ItemTracker {

public:
    struct image_track_item {
        cv::KeyPoint k;
        int frame_id;
        float tracking_certainty;
        float distance;
        float distance_background;
        float min_distance_static_ignore;
        float min_distance_moving_ignore;
        float distance_to_prediction;

        image_track_item() {

        }
        image_track_item(cv::KeyPoint kp, int frameid,float trackingCertainty){
            k = kp;
            frame_id = frameid;
            tracking_certainty = trackingCertainty;
        }
        image_track_item(image_track_item const &t){
            k = t.k;
            frame_id = t.frame_id;
            tracking_certainty = t.tracking_certainty;
            distance = t.distance;
            distance_background = t.distance_background;
        }
        float x() {return k.pt.x;}
        float y() {return k.pt.y;}
    };
    struct world_track_item {
        cv::Point3f world_coordinates;
        image_track_item  ti;
        cv::Point2f image_coordinates(){
            return ti.k.pt;
        }
        float disparity = 0;
        bool background_check_ok = false;
        bool disparity_in_range = false;

    };
    struct additional_ignore_point {
        additional_ignore_point() {
        }
        additional_ignore_point(cv::Point2f location, float timeout){
            p = location;
            invalid_after = timeout;
        }
        cv::Point2f p;
        float invalid_after;
    };
    cv::Point3f predicted_locationL_last = {0};
    cv::Point3f predicted_locationL_prev = {0};

private:
    struct TrackerSettings{

        int min_disparity=1;
        int max_disparity=20;

        int roi_min_size = 200;
        int roi_max_grow = 160;
        int roi_grow_speed = 64;

        int exclude_min_distance = 5; // in res/IMSCALEF resolution
        int exclude_max_distance = 50; // in res/IMSCALEF resolution

        int exclude_additional_max_distance = 15; // in res/IMSCALEF resolution

        int max_points_per_frame = 10;
        int radius = 15;
        int motion_thresh = 10;

        int background_subtract_zone_factor = 90;

        //only for dronetracker:
        int pixel_dist_landing_spot = 0;
        int pixel_dist_seperation_min = 2;
        int pixel_dist_seperation_max = 5;

        float version = 1.9f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version,min_disparity,max_disparity,roi_min_size,
               roi_max_grow,roi_grow_speed,exclude_min_distance,
               exclude_max_distance,background_subtract_zone_factor,
               max_points_per_frame,radius,
               motion_thresh,exclude_additional_max_distance,
               pixel_dist_landing_spot,pixel_dist_seperation_min,
               pixel_dist_seperation_max);
        }

    };
    std::string _settingsFile;
    struct Find_result {
        std::vector<image_track_item> image_track_items;
        std::vector<image_track_item> image_track_item_filtered;
        std::vector<image_track_item> excludes;
        cv::KeyPoint best_image_locationL;
        cv::Rect roi_offset;
        float disparity;
    };

    cv::Point3f predict(float dt, int frame_id);
    virtual cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size) = 0;
    uint match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<image_track_item> keypointsL);

    float estimate_sub_disparity(int disparity);
    void check_consistency(cv::Point3f previous_location,cv::Point3f measured_world_coordinates);
    void update_disparity(float disparity, float dt);
    void update_prediction_state(cv::Point3f p, float blob_size);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, float time, image_track_item *best_match, float disparity);
    void find(std::vector<image_track_item> exclude, std::vector<additional_ignore_point> additional_ignores);
    void select_best_world_candidate();
    std::vector<ItemTracker::image_track_item> select_best_image_candidates(std::vector<image_track_item> keypoints, std::vector<image_track_item> exclude_path, std::vector<additional_ignore_point> additional_ignores);
    void find_max_change_points(cv::Point prev, cv::Point roi_size, cv::Mat diff, std::vector<cv::KeyPoint> *scored_points);
    float calc_certainty(cv::KeyPoint item);
    void init_kalman();

    // Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    int type = CV_32F;
    cv::KalmanFilter kfL;
    cv::Mat stateL;
    cv::Mat _measL;

    cv::Mat _blurred_circle;

    float t_prev_tracking = 0;
    float t_prev_predict = 0;
    std::string _name;

    float prevX,prevY,prevZ =0;
    Smoother smoother_posX, smoother_posY, smoother_posZ;
    Smoother2 smoother_velX2,smoother_velY2,smoother_velZ2;
    Smoother smoother_velX, smoother_velY, smoother_velZ;
    Smoother2 smoother_accX2,smoother_accY2,smoother_accZ2;
    Smoother smoother_accX, smoother_accY, smoother_accZ;
    const int smooth_width_vel = 10;
    const int smooth_width_pos = 10;
    const int smooth_width_acc = 45;
    Smoother disp_smoothed;
    Smoother2 disp_rate_smoothed2;
    bool reset_filters;
    bool reset_disp = false;

    int detected_after_take_off = 0;
protected:

    std::vector<world_track_item> wti;

    int n_frames_lost = 100;
    const int n_frames_lost_threshold = 10;
    std::ofstream *_logger;
    VisionData * _visdat;
    int roi_size_cnt = 0;
    bool initialized = false;

    float blob_size_last = DRONE_IM_START_SIZE;

    const float certainty_factor = 1.1f; // TODO: tune
    const float certainty_init = 0.1f; // TODO: tune
    const int path_buf_size = 30;

    bool _enable_roi = true;
    bool _enable_depth_background_check = true;
    bool _enable_motion_background_check = true;

    bool enable_viz_roi = false; // flag for enabling the roi visiualization
    bool enable_viz_max_points = false; // flag for enabling the maxs visiualization

    float stereo_match(cv::Point closestL, cv::Mat diffL, cv::Mat diffR, float prev_disparity);
    void reset_tracker_ouput(float time);
    virtual cv::Mat get_probability_cloud(cv::Point size);
    virtual void init_settings() = 0;
public:

    TrackerSettings settings;
    cv::Mat _cir,_dif,_approx;
    Find_result find_result;
    std::vector<image_track_item> pathL;
    std::vector<image_track_item> predicted_pathL;
    std::vector<image_track_item> ignores; // keeps the item locations that should be ignored by other itemtrackers

    cv::Mat viz_max_points,viz_roi;

    int n_frames_tracking =0;
    float last_sighting_time = 0;

    bool foundL = false;

    void close (void);
    void init(std::ofstream *logger, VisionData *_visdat, std::string name);
    virtual void track(float time, std::vector<image_track_item> ignore, std::vector<additional_ignore_point> additional_ignores);
    void append_log();

    uint track_history_max_size = VIDEOFPS;
    std::vector<track_data> track_history;
    track_data Last_track_data() {
        if (track_history.empty())
            return track_data();
        return track_history.back();
    }

    int err [100];
    int cor_16 [100];

    float sub_disparity;
    float disparity_smoothed;
    float disp_rate;
    float disp_prev;

    float posX_smoothed = 0;
    float posY_smoothed = 0;
    float posZ_smoothed = 0;
};

#endif //ITEMTRACKER_H
