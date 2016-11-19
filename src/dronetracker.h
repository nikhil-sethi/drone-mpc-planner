#ifndef DRONETRACKER_H
#define DRONETRACKER_H
#include "defines.h"
#include <opencv2/features2d/features2d.hpp>



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

    struct mugSettings{

        //thresh params
        int iLowH1r = 0;
        int iHighH1r = 6;
        int iLowS1r = 0;
        int iHighS1r = 255;
        int iLowV1r = 188;
        int iHighV1r = 255;
        int iOpen1r =1;
        int iClose1r =1;

        //thresh params
        int iLowH1b = 92;
        int iHighH1b = 117;
        int iLowS1b = 0;
        int iHighS1b = 255;
        int iLowV1b = 165;
        int iHighV1b = 255;
        int iOpen1b =1;
        int iClose1b =1;

        //blob params

        // Change thresholds
        int minThreshold = 10;
        int maxThreshold = 91;

        // Filter by Area.
        int filterByArea = 1;
        int minArea = 2;
        int maxArea = 21;

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


        template <class Archive>
        void serialize( Archive & ar )
        {
          ar( iLowH1r,iHighH1r,iLowS1r,iHighS1r,iLowV1r,iHighV1r,iOpen1r,iClose1r,iLowH1b,iHighH1b,iLowS1b,iHighS1b,iLowV1b,iHighV1b,iOpen1b,iClose1b,minThreshold,maxThreshold,filterByArea,minArea,maxArea,filterByCircularity,minCircularity,maxCircularity,filterByConvexity,minConvexity,maxConvexity,filterByInertia,minInertiaRatio,maxInertiaRatio );
        }


    };
    mugSettings settings;

    void updateParams();

public:

    cv::Mat resFrame;

    void close (void);
    bool init(void);
    void track(cv::Mat frameL, cv::Mat frameR);

};




#endif //DRONETRACKER_H
