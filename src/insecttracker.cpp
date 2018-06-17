#include "insecttracker.h"

using namespace cv;
using namespace std;

bool InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"insect");
}
void InsectTracker::init_settings() {
    //thresh params
            settings.iLowH1r = 10;
            settings.iHighH1r = 255;
            settings.iLowS1r = 0;
            settings.iHighS1r = 255;
            settings.iLowV1r = 188;
            settings.iHighV1r = 255;
            settings.iOpen1r =0;
            settings.iClose1r =2;

            //blob params

            // Change thresholds
            settings.minThreshold = 10;
            settings.maxThreshold = 91;

            // Filter by Area.
            settings.filterByArea = 1;
            settings.minArea = 1;
            settings.maxArea = 40;

            // Filter by Circularity
            settings.filterByCircularity = 0;
            settings.minCircularity = 10;
            settings.maxCircularity = 100;

            // Filter by Convexity
            settings.filterByConvexity = 0;
            settings.minConvexity = 87;
            settings.maxConvexity = 100;

            // Filter by Inertia
            settings.filterByInertia = 0;
            settings.minInertiaRatio = 1;
            settings.maxInertiaRatio = 100;

            settings.min_disparity=0;
            settings.max_disparity=20;

            settings.roi_min_size = 200;
            settings.roi_max_grow = 160;
            settings.roi_grow_speed = 64;

}

bool InsectTracker::track(float time, cv::Point3f setpoint_world, cv::Point2f ignore) {
    ItemTracker::track(time,setpoint_world,ignore,MAX_BORDER_Y_DEFAULT,MAX_BORDER_Z_DEFAULT);
}
