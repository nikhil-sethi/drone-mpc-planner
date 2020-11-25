#pragma once
#include "common.h"

namespace tracking {

enum tracker_type {
    tt_drone,
    tt_insect,
    tt_blink,
    tt_replay,
    tt_virtualmoth
};

struct IgnoreBlob { // scaled with pparams.imscalef
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
    cv::Point2f p; // scaled with pparams.imscalef
    float radius; // scaled with pparams.imscalef
    double invalid_after;
    bool was_used = true;
    IgnoreType ignore_type;

};

struct BlobWorldProps {
    float x,y,z,distance,distance_bkg,radius;
    float disparity; // not really a world prop, but OK.
    bool radius_in_range = false,disparity_in_range = false,bkg_check_ok = false,takeoff_reject = false,im_pos_ok = false, valid = false;
    cv::Point3f pt() {return cv::Point3f(x,y,z);}
    int trkr_id = -1;
};
struct BlobProps { // scaled with pparams.imscalef
    float x,y,size,pixel_max; // these values are scaled with pparams.imscaled
    std::vector<IgnoreBlob> ignores;
    bool false_positive = false;
    cv::Mat mask;
    cv::Point2f pt() {return cv::Point2f(x,y);} //scaled with pparams.imscaled
    cv::Point2f pt_unscaled() {return cv::Point2f(x,y)*pparams.imscalef;}
    float size_unscaled() {return size*pparams.imscalef;}
    bool in_overexposed_area;
    int threshold_method;
    BlobProps(cv::Point2f pt,float blob_size,float blob_pixel_max, cv::Mat blob_mask, bool overexposed_area, int threshold_method_) : size(blob_size), pixel_max(blob_pixel_max), mask(blob_mask), in_overexposed_area(overexposed_area),threshold_method(threshold_method_) {
        x = pt.x;
        y = pt.y;
    }
    BlobWorldProps world_props;
};

struct ImageItem {
    uint frame_id = 0;
    uint blob_id = 0;
    float x = 0,y = 0,size = 0,pixel_max = 0,score = 0,disparity = 0;
    bool valid = false;
    bool blob_is_fused = false;

    cv::Point2f pt() {
        return cv::Point2f(x,y);
    }
    cv::Point3f ptd() {
        return cv::Point3f(x,y,disparity);
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
        blob_id = 666;
        frame_id = frameid;
        valid = true; //todo: implement when the log is not valid
    }
    ImageItem(BlobProps blob, int frameid, float matching_score, uint blob_id_) {
        x = blob.x*pparams.imscalef;
        y = blob.y*pparams.imscalef;
        size = blob.size_unscaled();
        pixel_max = blob.pixel_max;
        score = matching_score;
        frame_id = frameid;
        blob_id = blob_id_;
        disparity = blob.world_props.disparity;
        valid = true;
    }
};
struct ImagePredictItem {
    uint frame_id = 0;
    float x = 0,y = 0,disparity = 0,size = 0;
    float pixel_max = 0;
    bool valid = false;
    cv::Point2f pt() {
        return cv::Point2f(x,y);
    }
    ImagePredictItem() {}
    ImagePredictItem(cv::Point3f p,float size_,float pixel_max_,int frameid) {
        x = p.x;
        y = p.y;
        disparity = p.z;
        size = size_;
        pixel_max = pixel_max_;
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
        radius = wbp.radius;
        pt.x = wbp.x;
        pt.y = wbp.y;
        pt.z = wbp.z;
        valid = wbp.valid;
    }
    cv::Point3f pt = {0};
    ImageItem  iti;
    cv::Point2f image_coordinates() {
        return cv::Point2f(iti.x,iti.y);
    }
    float distance = 0, distance_bkg = 0;
    float radius = 0;
    bool valid = false;

    uint frame_id() {
        return iti.frame_id;
    }
    float size_in_image() {
        return iti.size;
    }

};
struct StateData {
    cv::Point3f pos = {0},spos = {0},vel = {0},vel_unfiltered = {0},acc = {0};
};
struct TrackData {
    WorldItem world_item;
    ImagePredictItem predicted_image_item;
    StateData state;
    cv::Point3f pos() {return state.pos;}
    cv::Point3f spos() {return state.spos;}
    cv::Point3f vel() {return state.vel;}
    cv::Point3f acc() {return state.acc;}
    float dt = 0;
    bool pos_valid = false;
    bool spos_valid = false;
    bool vel_valid = false;
    bool acc_valid = false;
    double time = 0;
    float yaw_deviation = 0;
    bool yaw_deviation_valid = 0;
};

[[maybe_unused]] static const char* false_positive_names[] = {"fp_not_a_fp",
                                                              "fp_short_detection",
                                                              "fp_static_location",
                                                              "fp_bkg"
                                                             };
enum false_positive_type {
    fp_not_a_fp = 0,
    fp_short_detection,
    fp_static_location,
    fp_bkg
};

struct FalsePositive {
    FalsePositive(WorldItem world_item,false_positive_type type_,double last_seen_time_) {
        im_size = world_item.iti.size;
        im_pt = world_item.iti.pt();
        type = type_;
        last_seen_time = last_seen_time_;
    }
    FalsePositive(false_positive_type type_, int detection_count_, float imx, float imy, float im_size_) {
        im_size = im_size_;
        im_pt = cv::Point2f(imx,imy);
        detection_count = detection_count_;
        type = type_;
        last_seen_time = 0;
    }
    false_positive_type type;
    cv::Point2f im_pt = {0};
    float im_size = 0;
    double last_seen_time = {0};
    int detection_count = 1;
};

}
