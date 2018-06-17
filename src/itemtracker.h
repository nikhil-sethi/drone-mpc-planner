#ifndef ITEMTRACKER_H
#define ITEMTRACKER_H
#include "defines.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

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

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( iLowH1r,iHighH1r,iLowS1r,iHighS1r,iLowV1r,iHighV1r,iOpen1r,iClose1r,minThreshold,maxThreshold,filterByArea,minArea,maxArea,filterByCircularity,minCircularity,maxCircularity,filterByConvexity,minConvexity,maxConvexity,filterByInertia,minInertiaRatio,maxInertiaRatio,min_disparity,max_disparity,roi_min_size,roi_max_grow,roi_grow_speed);
        }

    };
    std::string settingsFile;
    struct Find_result {
      cv::Mat treshL;
      std::vector<cv::KeyPoint> keypointsL;
      cv::KeyPoint best_image_locationL;
      cv::Rect roi_offset;
      int disparity;
      float smoothed_disparity;
      bool update_prev_frame;
    };

    void updateParams();
    cv::Mat segment(cv::Mat diffL, cv::Point previous_imageL_location, cv::Point roi_size);
    cv::Point3f predict(float dt);
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size);
    int match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<cv::KeyPoint> keypointsL);
    int stereo_match(cv::KeyPoint closestL, cv::Mat frameL_prev, cv::Mat prevFrameR_big, cv::Mat frameL, cv::Mat frameR, int prevDisparity);
    void update_prediction_state(cv::Point3f p);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, int n_frames_lost, cv::KeyPoint match, int disparity, cv::Point3f setpoint_world);
    void reset_tracker_ouput(int n_frames_lost);
    void find(cv::Mat frameL_small, cv::Mat frameL_s_prev_OK);
    std::vector<cv::KeyPoint> remove_ignores(std::vector<cv::KeyPoint> keypoints, cv::Point2f ignore);
    cv::Mat show_uncertainty_map_in_image(cv::Point p, cv::Mat resframeL);

    // Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kfL,kfR;
    cv::Mat stateL,stateR;
    cv::Mat measL,measR;

    bool firstFrame;




    cv::Mat blurred_circle;

    bool foundL = false;
    float t_prev = 0;
protected:
    std::ofstream *_logger;
    cv::Mat frameL_s_prev_OK;
    cv::Mat frameL_prev_OK;
    cv::Mat frameR_prev_OK;
    VisionData * visdat;
    int nframes_since_update_prev = 0;
    TrackerSettings settings;

    void reset_tracker_ouput();
    virtual cv::Mat get_probability_cloud(cv::Point size);
    virtual void init_settings() = 0;
public:

    cv::Mat cir,bkg,dif,treshL,approx;
    Find_result find_result;
    std::vector<cv::KeyPoint> pathL;
    std::vector<cv::KeyPoint> predicted_pathL;


    int n_frames_tracking =0;

    void close (void);
    bool init(std::ofstream *logger, VisionData *visdat, std::string name);
    virtual bool track(float time, cv::Point3f setpoint_world, cv::Point2f ignore, float drone_max_border_y, float drone_max_border_z);

    trackData data;
    Smoother smoother_posX, smoother_posY, smoother_posZ;
    Smoother smoother_velX, smoother_velY, smoother_velZ;
    const int smooth_width_vel = 10;
    const int smooth_width_pos = 10;

    Smoother disp_smoothed;


    bool breakpause;
};




#endif //ITEMTRACKER_H
