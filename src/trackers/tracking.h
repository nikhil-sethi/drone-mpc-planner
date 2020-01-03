#pragma once
#include "defines.h"
#include "common.h"

namespace tracking {

enum tracker_type {
    tt_drone,
    tt_insect,
    tt_blink,
    tt_replay
};

struct IgnoreBlob {
    enum IgnoreType {
        takeoff_spot,
        blink_spot,
        drone_taking_off,
        landing_spot
    };
    IgnoreBlob() {}
    IgnoreBlob(cv::Point2f location, float ignore_radius,double timeout, IgnoreType type) {
        p = location;
        ignore_type = type ;
        invalid_after = timeout;
        radius = ignore_radius;
    }
    cv::Point2f p;
    float radius;
    double invalid_after;
    bool was_used = true;
    IgnoreType ignore_type;

};

struct BlobWorldProps {
    float x,y,z,distance,distance_bkg,radius;
    float disparity; // not really a world prop, but OK.
    bool radius_in_range = false,disparity_in_range = false,bkg_check_ok = false,takeoff_reject = false,valid = false;
    cv::Point3f pt() {return cv::Point3f(x,y,z);}
    float heading;
    int trkr_id = -1;
};
struct BlobProps {
    float x,y,radius,pixel_max;
    std::vector<IgnoreBlob> ignores;
    cv::Mat mask;
    BlobProps(cv::Point2f pt, float blob_radius,float blob_pixel_max, cv::Mat blob_mask) : radius(blob_radius), pixel_max(blob_pixel_max), mask(blob_mask) {
        x = pt.x;
        y = pt.y;
    }
    BlobWorldProps world_props;
};

struct ImageItem {
    uint frame_id;
    uint keypoint_id;
    float x,y,size,pixel_max,score,disparity = 0;
    bool valid = false;
    bool blob_is_fused = false;

    cv::KeyPoint k() {
        return cv::KeyPoint(x,y,size);
    }
    cv::Point2f pt() {
        return cv::Point2f(x,y);
    }
    ImageItem() {}
    ImageItem(float x_, float y_, float disparity_, int frameid) {
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
    ImageItem(BlobProps blob, int frameid, float matching_score, uint keypointid) {
        x = blob.x;
        y = blob.y;
        size = blob.radius;
        pixel_max = blob.pixel_max;
        score = matching_score;
        frame_id = frameid;
        keypoint_id = keypointid;
        disparity = blob.world_props.disparity;
        valid = true;
    }
};
struct ImagePredictItem {
    uint frame_id;
    float x,y,size,certainty;
    float pixel_max;
    bool valid;
    cv::KeyPoint k() {
        return cv::KeyPoint(x,y,size);
    }
    cv::Point2f pt() {
        return cv::Point2f(x,y);
    }
    ImagePredictItem() {}
    ImagePredictItem(cv::Point2f p, float certainty_, float size_, float pixel_max_, int frameid) {
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
    WorldItem() {}
    WorldItem(ImageItem new_iti, BlobWorldProps wbp) {
        iti = new_iti;
        distance = wbp.distance;
        distance_bkg = wbp.distance_bkg;
        pt.x = wbp.x;
        pt.y = wbp.y;
        pt.z = wbp.z;
        valid = wbp.valid;
        heading = wbp.heading;
    }
    cv::Point3f pt;
    ImageItem  iti;
    cv::Point2f image_coordinates() {
        return cv::Point2f(iti.x,iti.y);
    }
    float heading;
    float distance, distance_bkg;
    bool valid = false;

    uint frame_id() {
        return iti.frame_id;
    }
    float size_in_image() {
        return iti.size;
    }

};

}
