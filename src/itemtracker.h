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
    struct ImageTrackItem {
        uint frame_id;
        uint keypoint_id;
        float distance_to_old_prediction,certainty,x,y,size,score,disparity = 0;
        bool valid = false;

        cv::KeyPoint k(){
            return cv::KeyPoint(x,y,size);
        }
        cv::Point2f pt(){
            return cv::Point2f(x,y);
        }
        ImageTrackItem() {}
        ImageTrackItem(float x_, float y_, int frameid){
            //read from log
            x = x_;
            y = y_;
            size  = -1;
            distance_to_old_prediction = -1;
            score = -1;
            keypoint_id = -1;
            frame_id = frameid;
            valid = true; //todo: implement when the log is not valid
        }
        ImageTrackItem(cv::KeyPoint kp, int frameid,float d2p, float matching_score, uint keypointid){
            x = kp.pt.x;
            y = kp.pt.y;
            size = kp.size;
            distance_to_old_prediction = d2p;
            score = matching_score;
            frame_id = frameid;
            keypoint_id = keypointid;
            valid = true;
        }
    };
    struct ImagePredictItem {
        uint frame_id;
        float x,y,size,certainty;
        float x_measured=-1,y_measured=-1, prediction_error = -1;
        cv::KeyPoint k(){
            return cv::KeyPoint(x,y,size);
        }
        cv::Point2f pt(){
            return cv::Point2f(x,y);
        }
        ImagePredictItem() {}
        ImagePredictItem(cv::Point2f p, float certainty_, float size_, int frameid){
            x = p.x;
            y = p.y;
            size = size_;
            certainty = certainty_;
            frame_id = frameid;
        }
        ImagePredictItem(float x_, float y_, float certainty_, float size_, int frameid){
            x = x_;
            y = y_;
            size = size_;
            certainty = certainty_;
            frame_id = frameid;
        }
    };
    struct WorldTrackItem {
        cv::Point3f pt;
        ImageTrackItem  iti;
        cv::Point2f image_coordinates(){
            return cv::Point2f(iti.x,iti.y);
        }
        float distance, distance_background;
        bool background_check_ok = false;
        bool disparity_in_range = false;
        bool valid = false;

        uint frame_id(){
            return iti.frame_id;
        }
        float size_in_image() {
            return iti.size;
        }

    };
    struct StaticIgnorePoint {
        StaticIgnorePoint() {}
        StaticIgnorePoint(cv::Point2f location, double timeout){
            p = location;
            invalid_after = timeout;
        }
        cv::Point2f p;
        double invalid_after;
        bool was_used = true;
    };

private:
    struct TrackerSettings{

        int min_disparity=1;
        int max_disparity=20;

        int roi_min_size = 200;
        int roi_max_grow = 160;
        int roi_grow_speed = 64;

        int score_threshold = 66;
        int background_subtract_zone_factor = 90;

        //only for dronetracker:
        int pixel_dist_landing_spot = 0;
        int pixel_dist_seperation_min = 2;
        int pixel_dist_seperation_max = 5;



        float version = 2.0f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version,
               min_disparity,
               max_disparity,
               roi_min_size,
               roi_max_grow,
               roi_grow_speed,
               score_threshold,
               background_subtract_zone_factor,
               pixel_dist_landing_spot,
               pixel_dist_seperation_min,
               pixel_dist_seperation_max);
        }

    };
    std::string _settingsFile;
    struct Find_result {
        std::vector<ImageTrackItem> image_track_items;
        std::vector<ImageTrackItem> image_track_item_filtered;
        std::vector<ImageTrackItem> excludes;
        cv::KeyPoint best_image_locationL;
        cv::Rect roi_offset;
        float disparity;
    };

    void predict(float dt, int frame_id);

    float estimate_sub_disparity(int disparity);
    void check_consistency(cv::Point3f previous_location,cv::Point3f measured_world_coordinates);
    void update_disparity(float disparity, float dt);
    void update_prediction_state(cv::Point2f image_location, float disparity, float size);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, double time, ImageTrackItem *best_match, float disparity);

    void update_world_candidate();

    float calc_certainty(cv::KeyPoint item);
    void init_kalman();

    // Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    int type = CV_32F;
    cv::KalmanFilter kfL;
    cv::Mat stateL;

    double t_prev_tracking = 0;
    double t_prev_predict = 0;
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
    int n_frames_lost_threshold = 10;
    std::ofstream *_logger;
    VisionData * _visdat;
    int roi_size_cnt = 0;
    bool initialized = false;

    const float certainty_factor = 1.1f; // TODO: tune
    const float certainty_init = 0.1f; // TODO: tune
    const int path_buf_size = 30;

    bool _enable_roi = true;
    bool _enable_depth_background_check = true;

    bool _tracking = false;
    bool _active = false;

    ImageTrackItem  _image_track_item;
    WorldTrackItem  _world_track_item;



    float stereo_match(cv::Point closestL, cv::Mat diffL, cv::Mat diffR, float prev_disparity);
    void reset_tracker_ouput(double time);
    virtual void init_settings() = 0;
public:

    virtual ~ItemTracker() {}

    TrackerSettings settings;
    Find_result find_result; // TODO: remove
    std::vector<WorldTrackItem> path;
    std::vector<ImagePredictItem> predicted_image_path;
    std::vector<StaticIgnorePoint> static_ignores_points_for_other_trkrs;
    std::vector<StaticIgnorePoint> static_ignores_points_for_me;

    int n_frames_tracking =0;
    double last_sighting_time = 0;

    bool tracking(){return _tracking;}

    void close (void);
    void init(std::ofstream *logger, VisionData *_visdat, std::string name);
    virtual void track(double time);
    void append_log();

    uint track_history_max_size = VIDEOFPS;
    std::vector<track_data> track_history;
    track_data Last_track_data() {
        if (track_history.empty())
            return track_data();
        return track_history.back();
    }
    virtual bool delete_me() = 0;

    ImageTrackItem image_track_item(){return _image_track_item;}
    WorldTrackItem world_track_item(){return _world_track_item;}
    void image_track_item(ImageTrackItem t){_image_track_item = t;}

    //TODO: make this private:
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
