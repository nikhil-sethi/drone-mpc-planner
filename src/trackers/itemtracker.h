#pragma once
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "filtering.h"
#include "tracking.h"

#include "common.h"
#include "visiondata.h"

#include "cereal/types/unordered_map.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/archives/binary.hpp"
#include <fstream>

namespace tracking {

class ItemTracker {


public:

    virtual tracker_type type() = 0;

protected:
    class TrackerParams: public xmls::Serializable
    {
    public:
        xmls::xInt min_disparity,max_disparity;
        xmls::xInt score_threshold, background_subtract_zone_factor;
        xmls::xFloat max_size;

        TrackerParams() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("TrackerParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("min_disparity",&min_disparity );
            Register("max_disparity",&max_disparity );

            Register("score_threshold",&score_threshold );
            Register("background_subtract_zone_factor",&background_subtract_zone_factor );
            Register("max_size",&max_size );
        }
    };

    bool enable_draw_stereo_viz = false;
    int min_disparity;
    int max_disparity;

    int _score_threshold;
    int background_subtract_zone_factor;
    float max_size; // world, in meters

    std::string settings_file;
    void deserialize_settings();
    void serialize_settings();

private:
    void update_disparity(float disparity, float dt);
    void update_state(cv::Point3f measured_world_coordinates, double time);

    void update_blob_filters();

    float calc_certainty(cv::KeyPoint item);

    int16_t _uid = -1;
    int16_t _viz_id = -1;

    float disparity_prev = 0;

    filtering::Smoother smoother_score;
    filtering::Smoother smoother_brightness;

protected:

    std::string _name;

    int pos_smth_width = -1;
    int vel_smth_width = -1;
    int acc_smth_width = -1;
    bool skip_wait_smth_spos = true;
    float disparity_filter_rate = 0.7;

    filtering::Smoother smoother_posX, smoother_posY, smoother_posZ;
    filtering::Smoother smoother_velX,smoother_velY,smoother_velZ;
    filtering::Smoother smoother_accX,smoother_accY,smoother_accZ;
    filtering::Smoother yaw_smoother;
    const int smooth_blob_props_width = 10;
    bool reset_filters;

    filtering::Smoother smoother_im_size;

    int n_frames_lost = 100;
    int n_frames_lost_threshold = 10;
    std::ofstream *_logger;
    VisionData * _visdat;
    bool initialized = false;

    const float certainty_factor = 1.1f; // TODO: tune
    const float certainty_init = 0.1f; // TODO: tune
    const uint path_buf_size = 30;

    const float disparity_predict_lower_bound = 3.0f;
    const float disparity_predict_upper_bound = 5.0f;

    bool _tracking = false;

    ImageItem _image_item;
    ImagePredictItem _image_predict_item;
    WorldItem _world_item;
    uint _blobs_are_fused_cnt = 0;
    std::vector<tracking::BlobProps> _all_blobs;

    void init_logger();
    float stereo_match(cv::Point2f im_posL, float size);
    std::tuple<int,float,int> disparity_search_rng(int x1);
    std::tuple<float,float> calc_match_score_masked(int i, int disp_end, int width, int height, cv::Mat diffL_mask_patch, cv::Mat diffR_mask_patch, cv::Mat grayL_patch, cv::Mat grayR_patch, int npixels);
    float calc_match_score_motion(int i,int x, int y, int width, int height,float tmp_diffL_sum, cv::Mat diffL_roi, cv::Mat diffR);
    float estimate_sub_disparity(int disparity,float * err);

    void reset_tracker_ouput(double time);
    void calc_world_props_blob_generic(BlobProps * pbs, bool use_max);
    bool check_ignore_blobs_generic(BlobProps * pbs);
    void cleanup_paths();
    float score(BlobProps blob, ImageItem ref);
    void update_prediction(double time);
public:

    int16_t uid() {return _uid;}
    int16_t viz_id() { return _viz_id;}
    void viz_id(int16_t id) {_viz_id = id;}

    virtual ~ItemTracker() {}

    std::vector<WorldItem> path;
    std::vector<ImagePredictItem> predicted_image_path;
    std::vector<IgnoreBlob> ignores_for_other_trkrs;
    std::vector<IgnoreBlob> ignores_for_me;

    void blobs_are_fused() {
        _world_item.iti.blob_is_fused = true;
        _image_item.blob_is_fused = true;
        _blobs_are_fused_cnt++;
    }

    int n_frames_tracking =0;
    double last_sighting_time = 0;

    bool tracking() {return _tracking;}

    cv::Mat viz_disp;

    void close(void);
    void init(std::ofstream *logger, VisionData *_visdat, std::string name, int16_t viz_id);
    virtual void update(double time);
    virtual bool check_ignore_blobs(BlobProps * pbs) = 0;
    virtual void calc_world_item(BlobProps * pbs, double time) = 0;
    void append_log();

    uint track_history_max_size;
    std::vector<track_data> track_history;
    track_data Last_track_data() {
        if (track_history.empty())
            return track_data();
        return track_history.back();
    }
    virtual bool delete_me() = 0;

    ImageItem image_item() {return _image_item;}
    ImagePredictItem image_predict_item() {return _image_predict_item;}
    WorldItem world_item() {return _world_item;}
    void world_item(WorldItem world_item) {
        _world_item = world_item;
        _image_item = _world_item.iti;
    }
    void item_invalidize() {
        _image_item.valid = false;
        _world_item.valid = false;
    }
    void all_blobs(std::vector<tracking::BlobProps> blobs) {
        _all_blobs = blobs;
    }

    float score_threshold() {return static_cast<float>(_score_threshold);}

    virtual float score(BlobProps blob) {
        return score(blob,_image_item);
    }

};

}
