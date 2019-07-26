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

    struct BlobProps {
        float x,y,radius,pixel_max;
        BlobProps(cv::Point2f pt, float area,float blob_pixel_max){
            x = pt.x;
            y = pt.y;
            radius = sqrtf(area/static_cast<float>(M_PI));
            pixel_max = blob_pixel_max;
        }
    };
    struct BlobWorldProps {
        float x,y,z,distance,distance_bkg;
        float disparity; // not really a world prop, but OK.
        bool disparity_in_range = false,bkg_check_ok = false,valid = false;
    };

    struct ImageItem {
        uint frame_id;
        uint keypoint_id;
        float x,y,size,pixel_max,score,disparity = 0;
        bool valid = false;
        bool blob_is_fused = false;

        cv::KeyPoint k(){
            return cv::KeyPoint(x,y,size);
        }
        cv::Point2f pt(){
            return cv::Point2f(x,y);
        }
        ImageItem() {}
        ImageItem(float x_, float y_, float disparity_, int frameid){
            //read from log
            x = x_;
            y = y_;
            disparity = disparity_;
            size  = -1;
            score = -1;
            pixel_max = -1;
            keypoint_id = 666;
            frame_id = frameid;
            valid = true; //todo: implement when the log is not valid
        }
        ImageItem(BlobProps blob, float disparity_, int frameid, float matching_score, uint keypointid){
            x = blob.x;
            y = blob.y;
            size = blob.radius;
            pixel_max = blob.pixel_max;
            score = matching_score;
            frame_id = frameid;
            keypoint_id = keypointid;
            disparity = disparity_;
            valid = true;
        }
    };
    struct ImagePredictItem {
        uint frame_id;
        float x,y,size,certainty;
        float x_measured=-1,y_measured=-1, prediction_error = -1;
        float pixel_max;
        bool valid;
        cv::KeyPoint k(){
            return cv::KeyPoint(x,y,size);
        }
        cv::Point2f pt(){
            return cv::Point2f(x,y);
        }
        ImagePredictItem() {}
        ImagePredictItem(cv::Point2f p, float certainty_, float size_, float pixel_max_, int frameid){
            x = p.x;
            y = p.y;
            size = size_;
            pixel_max = pixel_max_;
            certainty = certainty_;
            frame_id = frameid;
            valid = true;
        }
    };
    struct WorldItem {
        WorldItem(){}
        WorldItem(ImageItem new_iti, BlobWorldProps wbp){
            iti = new_iti;
            distance = wbp.distance;
            distance_bkg = wbp.distance_bkg;
            pt.x = wbp.x;
            pt.y = wbp.y;
            pt.z = wbp.z;
            valid = wbp.valid;
        }
        cv::Point3f pt;
        ImageItem  iti;
        cv::Point2f image_coordinates(){
            return cv::Point2f(iti.x,iti.y);
        }
        float distance, distance_bkg;
        bool valid = false;

        uint frame_id(){
            return iti.frame_id;
        }
        float size_in_image() {
            return iti.size;
        }

    };
    struct IgnoreBlob {
        enum IgnoreType{
            landing_spot,
            blink_spot,
            drone_taking_off
        };
        IgnoreBlob() {}
        IgnoreBlob(cv::Point2f location, double timeout, IgnoreType type){
            p = location;
            ignore_type = type ;
            invalid_after = timeout;
        }
        cv::Point2f p;
        double invalid_after;
        bool was_used = true;
        IgnoreType ignore_type;

    };


private:
    struct TrackerSettings{

        int min_disparity=1;
        int max_disparity=20;

        int static_ignores_dist_thresh = 5;

        int score_threshold = 66;
        int background_subtract_zone_factor = 97;


        float version = 2.1f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version,
               min_disparity,
               max_disparity,
               score_threshold,
               background_subtract_zone_factor);
        }

    };
    std::string _settingsFile;

    void predict(float dt, int frame_id);

    float estimate_sub_disparity(int disparity);
    void check_consistency();
    void update_disparity(float disparity, float dt);
    void update_prediction_state(cv::Point2f image_location, float disparity);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, double time, float disparity);

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

    //disparity stuff:
    int err [100];
    int cor_16 [100];

    float disparity_smoothed;
    float disp_rate;
    float disp_prev; // TODO: there's two disp(arity)_prevs...?
    float disparity_prev = 0;

    Smoother smoother_score;
    Smoother smoother_brightness;

    int detected_after_take_off = 0;
protected:

    Smoother smoother_posX, smoother_posY, smoother_posZ;
    Smoother2 smoother_velX2,smoother_velY2,smoother_velZ2;
    Smoother smoother_velX, smoother_velY, smoother_velZ;
    Smoother2 smoother_accX2,smoother_accY2,smoother_accZ2;
    Smoother smoother_accX, smoother_accY, smoother_accZ;
    const int smooth_width_vel = 10;
    const int smooth_width_pos = 10;
    const int smooth_width_acc = 45;
    const int smooth_blob_props_width = 10;
    Smoother disp_smoothed;
    Smoother2 disp_rate_smoothed2;
    bool reset_filters;
    bool reset_disp = false;

    Smoother smoother_im_size;

    int n_frames_lost = 100;
    int n_frames_lost_threshold = 10;
    std::ofstream *_logger;
    VisionData * _visdat;
    bool initialized = false;

    const float certainty_factor = 1.1f; // TODO: tune
    const float certainty_init = 0.1f; // TODO: tune
    const uint path_buf_size = 30;

    bool _tracking = false;

    ImageItem  _image_item;
    ImagePredictItem _image_predict_item;
    WorldItem  _world_item;

    float stereo_match(cv::Point closestL, cv::Mat diffL, cv::Mat diffR);
    void reset_tracker_ouput(double time);
    BlobWorldProps calc_world_props_blob_generic(BlobProps * pbs);
    bool check_ignore_blobs_generic(BlobProps * pbs);
    void cleanup_paths();
    virtual void init_settings() = 0;
public:

    virtual ~ItemTracker() {}

    TrackerSettings settings;
    std::vector<WorldItem> path;
    std::vector<ImagePredictItem> predicted_image_path;
    std::vector<IgnoreBlob> ignores_for_other_trkrs;
    std::vector<IgnoreBlob> ignores_for_me;

    void blobs_are_fused(){
        _world_item.iti.blob_is_fused = true;
        _image_item.blob_is_fused = true;
    }

    int n_frames_tracking =0;
    double last_sighting_time = 0;

    bool tracking(){return _tracking;}

    void close (void);
    void init(std::ofstream *logger, VisionData *_visdat, std::string name);
    virtual void track(double time);
    virtual bool check_ignore_blobs(BlobProps * pbs, uint id) = 0;
    virtual ItemTracker::BlobWorldProps calc_world_item(BlobProps * pbs, double time) = 0;
    void append_log();

    uint track_history_max_size = VIDEOFPS;
    std::vector<track_data> track_history;
    track_data Last_track_data() {
        if (track_history.empty())
            return track_data();
        return track_history.back();
    }
    virtual bool delete_me() = 0;

    ImageItem image_item(){return _image_item;}
    ImagePredictItem image_predict_item(){return _image_predict_item;}
    WorldItem world_item(){return _world_item;}
    void world_item(WorldItem world_item){
        _world_item = world_item;
        _image_item = _world_item.iti;
    }
    void item_invalidize(){
        _image_item.valid = false;
        _world_item.valid = false;
    }

    float score_threshold() {return static_cast<float>(settings.score_threshold) / 1e3f;}

    float score(BlobProps blob) {
        float dist = sqrtf(powf(_image_item.x-blob.x,2)+powf(_image_item.y-blob.y,2));
        float im_size_diff = fabs(_image_item.size - blob.radius) / (blob.radius + _image_item.size);
        float score = 1.f / (dist + 5.f*im_size_diff); // TODO: certainty

        if (_image_predict_item.valid){
            float dist_pred = sqrtf(powf(_image_predict_item.x-blob.x,2)+powf(_image_predict_item.y-blob.y,2));
            float ps = smoother_im_size.latest();
            float im_size_diff_pred = fabs(ps - blob.radius) / (blob.radius+ps);
            float score_pred = 1.f / (dist_pred + 5.f*im_size_diff_pred); // TODO: certainty
            if (score_pred > score)
                score = score_pred;
        }

        return score;
    }
};

#endif //ITEMTRACKER_H
