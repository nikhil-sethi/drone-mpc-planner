#include "insecttracker.h"

using namespace cv;
using namespace std;

void InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
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

void InsectTracker::track(float time, cv::Point3f setpoint_world, std::vector<track_item> ignore) {
    ItemTracker::track(time,setpoint_world,ignore,MAX_BORDER_Y_DEFAULT,MAX_BORDER_Z_DEFAULT);
}


/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat InsectTracker::get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size) {

    cv::Mat blurred_circle = get_probability_cloud(size);

    int x = p.x-size.x/2;
    if (x < 0)
        x = 0;
    int width = size.x;
    if (x+width > _visdat->diffL.cols)
        x -= (x+width) - _visdat->diffL.cols;

    int y = p.y-size.y/2;
    if (y < 0)
        y = 0;
    int height = size.y;
    if (y+height>_visdat->diffL.rows)
        y -= (y+height) - _visdat->diffL.rows;

    cv::Rect roi(x,y,width,height);
    find_result.roi_offset = roi;

    _bkg = _visdat->uncertainty_map(roi);
    diffL(roi).convertTo(_dif, CV_32F);
    cv::Mat res;
    res = blurred_circle.mul(_dif).mul(_bkg);
    res.convertTo(res, CV_8UC1);

    return res;
}
