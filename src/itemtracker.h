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
 * This class will track items
 *
 */
class ItemTracker {

public:
    struct track_item {
        cv::KeyPoint k;
        cv::KeyPoint k_void;
        int frame_id;
        float tracking_certainty;
        float distance;
        float distance_background;

        track_item(cv::KeyPoint kp, int frameid,float trackingCertainty){
            k = kp;
            frame_id = frameid;
            tracking_certainty = trackingCertainty;
        }
        track_item(track_item const &t){
            k = t.k;
            k_void = t.k_void;
            frame_id = t.frame_id;
            tracking_certainty = t.tracking_certainty;
            distance = t.distance;
            distance_background = t.distance_background;
        }
        float x() {return k.pt.x;}
        float y() {return k.pt.y;}
    };

private:
    cv::SimpleBlobDetector::Params params;
    struct TrackerSettings{

        //thresh params
        int iLowH1r = 10;
        int iHighH1r = 255;
        int iLowS1r = 0;
        int iHighS1r = 255;
        int iLowV1r = 188;
        int iHighV1r = 255;
        int iOpen1r =0;
        int iClose1r =2;

        //blob params

        // Change thresholds
        int minThreshold = 10;
        int maxThreshold = 91;

        // Filter by Area.
        int filterByArea = 1;
        int minArea = 1;
        int maxArea = 40;

        // Filter by Circularity
        int filterByCircularity = 0;
        int minCircularity = 10;
        int maxCircularity = 100;

        // Filter by Convexity
        int filterByConvexity = 0;
        int minConvexity = 87;
        int maxConvexity = 100;

        // Filter by Inertia
        int filterByInertia = 0;
        int minInertiaRatio = 1;
        int maxInertiaRatio = 100;

        int min_disparity=0;
        int max_disparity=20;

        int roi_min_size = 200;
        int roi_max_grow = 160;
        int roi_grow_speed = 64;

        int appear_void_max_distance = 3;
        int void_void_max_distance = 10;
        int exclude_min_distance = 5; // in res/IMSCALEF resolution
        int exclude_max_distance = 50; // in res/IMSCALEF resolution

        int max_points_per_frame = 10;
        int ignore_circle_r_around_motion_max = 15;
        int motion_thresh = 10;

        int background_subtract_zone_factor = 90;
        float version = 1.3f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, iLowH1r,iHighH1r,iLowS1r,iHighS1r,
               iLowV1r,iHighV1r,iOpen1r,iClose1r,minThreshold,
               maxThreshold,filterByArea,minArea,maxArea,
               filterByCircularity,minCircularity,maxCircularity,
               filterByConvexity,minConvexity,maxConvexity,
               filterByInertia,minInertiaRatio,maxInertiaRatio,
               min_disparity,max_disparity,roi_min_size,
               roi_max_grow,roi_grow_speed,appear_void_max_distance,
               void_void_max_distance,appear_void_max_distance,
               exclude_max_distance,background_subtract_zone_factor,
               max_points_per_frame,ignore_circle_r_around_motion_max,motion_thresh);
        }

    };
    std::string _settingsFile;
    struct Find_result {
        cv::Mat treshL;
        std::vector<track_item> keypointsL;
        std::vector<track_item> keypointsL_wihout_voids;
        std::vector<track_item> excludes;
        cv::KeyPoint best_image_locationL;
        cv::Rect roi_offset;
        float disparity;
        float smoothed_disparity;
    };

    void updateParams();
    cv::Mat segment(cv::Mat diffL, cv::Point previous_imageL_location, cv::Point roi_size);
    cv::Point3f predict(float dt, int frame_id);
    virtual cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size) = 0;
    uint match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<track_item> keypointsL);
    float stereo_match(cv::KeyPoint closestL, cv::Mat frameL_prev, cv::Mat prevFrameR_big, cv::Mat frameL, cv::Mat frameR, float disparity);
    float estimate_sub_disparity(int disparity);
    void check_consistency(cv::Point3f previous_location,cv::Point3f measured_world_coordinates);
    float update_disparity(float disparity, float dt);
    void update_prediction_state(cv::Point3f p, float blob_size);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, track_item *best_match, float disparity);
    void find(std::vector<track_item> exclude);
    std::vector<ItemTracker::track_item> remove_excludes(std::vector<track_item> keypoints, std::vector<track_item> exclude_path);
    std::vector<ItemTracker::track_item> remove_excludes_improved(std::vector<track_item> keypoints, std::vector<track_item> exclude_path);
    cv::Mat show_uncertainty_map_in_image(cv::Point p, cv::Mat resframeL);
    std::vector<track_item> remove_voids(std::vector<track_item> keyps, std::vector<track_item> keyps_prev);
    void find_max_change(cv::Point prev, cv::Point roi_size, cv::Mat diff, std::vector<cv::KeyPoint> *scored_points);
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
    int n_frames_lost = 100;
    const int n_frames_lost_threshold = 10;
    std::ofstream *_logger;
    VisionData * _visdat;
    int roi_size_cnt = 0;

    cv::Point3f predicted_locationL_last = {0};
    float blob_size_last = DRONE_IM_START_SIZE;

    const float certainty_factor = 1.1f; // TODO: tune
    const float certainty_init = 0.1f; // TODO: tune
    const int path_buf_size = 30;

    void reset_tracker_ouput();
    virtual cv::Mat get_probability_cloud(cv::Point size);
    virtual void init_settings() = 0;
public:

    TrackerSettings settings;
    cv::Mat _cir,_bkg,_dif,_treshL,_approx;
    Find_result find_result;
    std::vector<track_item> pathL;
    std::vector<track_item> predicted_pathL;

    int n_frames_tracking =0;

    bool foundL = false;

    void close (void);
    void init(std::ofstream *logger, VisionData *_visdat, std::string name);
    virtual void track(float time, std::vector<track_item> ignore, float drone_max_border_y, float drone_max_border_z);

//    trackData data;
    std::vector<trackData> track_history; // TODO: this will build up indefenitely.... and only last track data is used,
    std::vector<trackData> track_prediction_history;
    trackData get_last_track_data() {
        if (track_history.empty())
            return trackData();
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
