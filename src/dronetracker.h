#ifndef DRONETRACKER_H
#define DRONETRACKER_H
#include "defines.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "smoother.h"
#include "common.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>


/*
 * This class will track a micro drone with leds
 *
 */
class DroneTracker {


private:
    cv::SimpleBlobDetector::Params params;
    struct patsSettings{

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

        int uncertainty_multiplier = 2;
        int uncertainty_power = 6;
        int uncertainty_background = 0.3*255.0;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( iLowH1r,iHighH1r,iLowS1r,iHighS1r,iLowV1r,iHighV1r,iOpen1r,iClose1r,minThreshold,maxThreshold,filterByArea,minArea,maxArea,filterByCircularity,minCircularity,maxCircularity,filterByConvexity,minConvexity,maxConvexity,filterByInertia,minInertiaRatio,maxInertiaRatio,min_disparity,max_disparity,uncertainty_power,uncertainty_multiplier,uncertainty_background);
        }


    };
    patsSettings settings;

    struct Find_drone_result {
      cv::Mat treshfL;
      std::vector<cv::KeyPoint> keypointsL;
      cv::KeyPoint best_image_locationL;
      int disparity;
      float smoothed_disparity;
      bool update_prev_frame;
    };
    Find_drone_result find_drone_result;

    void updateParams();
    cv::Mat createBlurryCircle(int size, float background);
    cv::Mat segment_drone(cv::Mat frame, cv::Mat frame_prev, bool build_uncertainty_map2, cv::Point previous_imageL_location);
    cv::Point3f predict_drone(float dt);
    cv::Mat get_uncertainty_map_with_drone(cv::Point p);
    int match_closest_to_prediciton(cv::Point3f predicted_drone_locationL, std::vector<cv::KeyPoint> keypointsL);
    int stereo_match(cv::KeyPoint closestL, cv::Mat frameL_big_prev, cv::Mat prevFrameR_big, cv::Mat frameL, cv::Mat frameR, int prevDisparity);
    void update_prediction_state(cv::Point3f p);
    void update_tracker_ouput(cv::Point3f measured_world_coordinates, float dt, int n_frames_lost, cv::KeyPoint match, int disparity);
    void reset_tracker_ouput(int n_frames_lost);
    void drawviz(cv::Mat frameL, cv::Mat treshfL, cv::Mat framegrayL, cv::Point previous_imageL_location);
    void find_drone(cv::Mat frameL_small, cv::Mat frameL_s_prev, cv::Mat frameL_s_prev_OK);
    void beep(cv::Point2f drone, int n_frames_lost, float time, cv::Mat frameL_small);

    cv::Mat show_uncertainty_map_in_image(cv::Point p, cv::Mat resframeL);

    void init_avg_prev_frame(void);
    void collect_avg_prev_frame(cv::Mat frame);
    void collect_no_drone_frames(cv::Mat diff);

    // Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    const float background_calib_time = 15.0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kfL,kfR;
    cv::Mat stateL,stateR;
    cv::Mat measL,measR;


#define SETPOINTXMAX 3000 // in mm
#define SETPOINTYMAX 3000 // in mm
#define SETPOINTZMAX 5000 // in mm

    int setpointX = SETPOINTXMAX / 2;
    int setpointY = SETPOINTYMAX / 2;
    int setpointZ = 2000;
    int wpid = 0;

    std::vector<cv::Point3i> setpoints;

    std::ofstream *_logger;

    bool firstFrame;
    cv::Mat frameL_s_prev;
    cv::Mat frameL_s_prev_OK;

    cv::Mat frameL_big_prev;
    cv::Mat frameL_big_prev_OK;
    cv::Mat frameR_big_prev;
    cv::Mat frameR_big_prev_OK;

    cv::Mat uncertainty_map;
    cv::Mat blurred_circle;

    cv::Mat avg_prev_frame;
    int n_avg_prev_frames = 0;

    std::vector<cv::KeyPoint> dronepathL;
    std::vector<cv::KeyPoint> predicted_dronepathL;

public:       

    cv::Point3d setpoint;
    cv::Point3f setpointw;

    cv::Mat resFrame;

    void close (void);
    bool init(std::ofstream *logger);
    bool track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf, float time, int frame_id);



    trackData data;
    Smoother smoother_posX, smoother_posY, smoother_posZ;
    Smoother smoother_velX, smoother_velY, smoother_velZ;
    const int smooth_width_vel = 10;
    const int smooth_width_pos = 10;

    Smoother disp_smoothed;


    bool breakpause;
};




#endif //DRONETRACKER_H
