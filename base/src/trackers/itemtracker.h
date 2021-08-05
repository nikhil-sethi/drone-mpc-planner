#pragma once
#include "common.h"
#include "filtering.h"
#include "tracking.h"
#include "visiondata.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"

namespace tracking {

class ItemTracker {

private:
    int16_t _uid = -1;
    int16_t _viz_id = -1;
    int _motion_thresh = -1;
    float disparity_prev = 0;

    void update_disparity(float disparity, float dt);
    void update_state(cv::Point3f measured_world_coordinates, double time, bool using_prediction);
    void update_blob_filters();

protected:
    class TrackerParams: public xmls::Serializable
    {
    public:
        xmls::xInt min_disparity,max_disparity;
        xmls::xInt background_subtract_zone_factor;
        xmls::xFloat max_size,score_threshold;

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

    std::string _name;
    std::string settings_file;
    TrackerParams params;
    std::string logger_fn;
    std::ofstream *_logger;
    VisionData * _visdat;
    bool initialized = false;
    bool initialized_logger = false;
    uint track_history_max_size;

    int min_disparity;
    int max_disparity;
    float disparity_filter_rate = 0.7;

    float _score_threshold;
    int background_subtract_zone_factor;
    float max_radius; // world, in meters
    float expected_radius = 0.01;

    int pos_smth_width = -1;
    int vel_smth_width = -1;
    int acc_smth_width = -1;
    const int smooth_blob_props_width = 10;
    // bool skip_wait_smth_spos = true;
    filtering::Smoother smoother_im_size;
    filtering::Smoother smoother_brightness;
    filtering::Smoother smoother_posX, smoother_posY, smoother_posZ;
    filtering::Smoother smoother_accX,smoother_accY,smoother_accZ;
    filtering::Tf_PT2_3f vel_filt;
    bool reset_smoothers;

    bool _tracking = false;
    TrackData last_valid_trackdata_for_prediction;
    TrackData last_vel_valid_trackdata_for_prediction;
    int _n_frames_lost = 100;
    int n_frames_lost_threshold = 10;
    int _n_frames_tracking =0;  // reset after interruptions
    int _n_frames_tracked =0;   // total tracked frames ever
    int _n_frames =0;           // lifetime of the tracker
    double _last_detection = 0;

    ImageItem _image_item;
    ImagePredictItem _image_predict_item;
    WorldItem _world_item;
    uint _blobs_are_fused_cnt = 0;
    std::vector<tracking::BlobProps> _all_blobs;
    std::vector<TrackData> _track;

    void init_logger(std::ofstream *logger);
    float stereo_match(BlobProps * blob);
    std::tuple<int,float,int,int,bool> disparity_search_rng(BlobProps * blob,int x,int radius);
    int calc_rough_disparity(BlobProps * blob,int radius);
    std::tuple<float,float> calc_match_score_masked(int i,int disp_end,int width,int height,cv::Mat diffL_mask_patch,cv::Mat diffR_mask_patch,cv::Mat grayL_patch,cv::Mat grayR_patch,int npixels);
    float calc_match_score_motion(int i,int x, int y, int width, int height,float tmp_diffL_sum, cv::Mat diffL_roi, cv::Mat diffR);
    float estimate_sub_disparity(int disparity,float * err);

    void reset_tracker_ouput(double time);
    void calc_world_props_blob_generic(BlobProps * blob);
    bool check_ignore_blobs_generic(BlobProps * blob);
    void cleanup_history();
    float score(BlobProps * blob, ImageItem * ref);
    void update_prediction(double time);

    void deserialize_settings();
    void serialize_settings();

public:
    std::vector<IgnoreBlob> ignores_for_other_trkrs;
    std::vector<IgnoreBlob> ignores_for_me;
    cv::Mat viz_disp;
    bool enable_draw_stereo_viz = false;

    int16_t uid() {return _uid;}
    int16_t viz_id() { return _viz_id;}
    void viz_id(int16_t id) {_viz_id = id;}
    std::string name() {return _name;}

    void init(std::ofstream *logger, VisionData *_visdat, int motion_thresh, std::string name, int16_t viz_id);
    void init(VisionData *_visdat, int motion_thresh, std::string name, int16_t viz_id);
    virtual void update(double time);
    void invalidize(bool force) {
        if (_image_item.frame_id != _visdat->frame_id || force) {
            _image_item.valid = false;
            _world_item.valid = false;
        }
        blobs_fused_update();
    }
    virtual bool check_ignore_blobs(BlobProps * blob) = 0;
    virtual void calc_world_item(BlobProps * blob, double time) = 0;
    void append_log();
    void close(void);
    virtual bool delete_me() = 0;
    virtual ~ItemTracker() {}

    std::vector<TrackData> track() { return _track; };
    TrackData last_track_data() {
        if (_track.empty())
            return TrackData();
        return _track.back();
    }
    virtual tracker_type type() = 0;

    int n_frames() {return _n_frames;}
    int n_frames_tracking() {return _n_frames_tracking;}
    double last_detection() {return _last_detection;}
    bool tracking() {return _tracking;}
    bool properly_tracking() {return _n_frames_tracking > 3 && _tracking;}
    int n_frames_lost() { return _n_frames_lost; }

    ImageItem image_item() {return _image_item;}
    ImagePredictItem image_predict_item() {return _image_predict_item;}
    WorldItem world_item() {return _world_item;}
    void world_item(WorldItem world_item) {
        _world_item = world_item;
        _image_item = _world_item.image_item;
    }
    void all_blobs(std::vector<tracking::BlobProps> blobs) { _all_blobs = blobs; }
    void blobs_fused_update() {
        if (_blobs_are_fused_cnt && !_image_item.valid && type() == tt_insect)
            blobs_are_fused();
    }
    void blobs_are_fused() {
        _world_item.image_item.blob_is_fused = true;
        _image_item.blob_is_fused = true;
        _blobs_are_fused_cnt++;
    }

    virtual float score(BlobProps * blob) { return score(blob,&_image_item); }
    float score_threshold() {
        if (_blobs_are_fused_cnt && type() == tt_insect)
            return _score_threshold * 2;
        return std::clamp(_score_threshold+_n_frames_lost*0.3f*_score_threshold,0.f,1.5f*_score_threshold);
    }

};

}
