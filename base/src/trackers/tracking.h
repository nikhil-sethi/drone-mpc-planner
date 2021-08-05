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
    float x=0,y=0,z=0,distance=0,distance_bkg=0,radius=0;
    float disparity,motion_sum=0; // not really world props, but OK.
    bool radius_in_range = false,disparity_in_range = false,bkg_check_ok = false,takeoff_reject = false,im_pos_ok = false, valid = false;
    cv::Point3f pt() {return cv::Point3f(x,y,z);}
    int trkr_id = -1;
};
struct BlobProps { // scaled with pparams.imscalef
    float x=0,y=0,size=0; // these values are scaled with pparams.imscaled
    uint32_t n_motion_pixels=0,motion_sum=0;
    uint8_t motion_noise=0,pixel_max=0;
    std::vector<IgnoreBlob> ignores;
    bool false_positive = false;
    cv::Point2f pt() {return cv::Point2f(x,y);} //scaled with pparams.imscaled
    cv::Point2f pt_unscaled() {return cv::Point2f(x,y)*pparams.imscalef;}
    float size_unscaled() {return size*pparams.imscalef;}
    bool in_overexposed_area=false;
    BlobProps(cv::Point2f pt,float size_,uint32_t n_motion_pixels_, uint32_t motion_sum_, uint8_t pixel_max_, uint8_t pixel_motion_noise_, bool overexposed_area, int frame_id_) : size(size_), n_motion_pixels(n_motion_pixels_), motion_sum(motion_sum_), motion_noise(pixel_motion_noise_), pixel_max(pixel_max_), in_overexposed_area(overexposed_area) {
        x = pt.x;
        y = pt.y;
        frame_id = frame_id_;
    }
    BlobWorldProps world_props;
    int frame_id=0;
};

struct ImageItem {
    uint frame_id = 0;
    uint blob_id = 0;
    float x = 0,y = 0,size = 0,pixel_max = 0,score = 0,disparity = 0,motion_sum = 0;
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
        motion_sum = -1;
        blob_id = 666;
        frame_id = frameid;
        valid = true;
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
        motion_sum = blob.world_props.motion_sum;
        valid = true;
    }
};
struct ImagePredictItem {
    uint frame_id = 0;
    float disparity = 0,size = 0;
    float pixel_max = 0;
    bool out_of_image = false;
    bool out_of_image_right = false;
    bool valid = false;
    cv::Point2f pt;
    cv::Point2f pt_unbound;
    ImagePredictItem() {}
    ImagePredictItem(cv::Point3f p,float size_,float pixel_max_,int frameid) {
        pt_unbound.x = p.x;
        pt_unbound.y = p.y;
        out_of_image = (p.x<0 || p.y < 0 || p.x >= IMG_W || p.y >= IMG_H);
        pt.x = std::clamp(p.x,0.f,IMG_Wf-1);
        pt.y = std::clamp(p.y,0.f,IMG_Hf-1);
        disparity = p.z;
        out_of_image_right = p.x-disparity<0;
        size = size_;
        pixel_max = pixel_max_;
        frame_id = frameid;
        valid = true;
    }
};
struct WorldItem {
    WorldItem() {}
    WorldItem(ImageItem new_iti, BlobWorldProps wbp) {
        image_item = new_iti;
        distance = wbp.distance;
        distance_bkg = wbp.distance_bkg;
        radius = wbp.radius;
        pt = wbp.pt();
        valid = wbp.valid;
    }
    cv::Point3f pt = {0};
    ImageItem  image_item;
    cv::Point2f image_coordinates() {
        return cv::Point2f(image_item.x,image_item.y);
    }
    float distance = 0, distance_bkg = 0;
    float radius = 0;
    bool valid = false;

    uint frame_id() {
        return image_item.frame_id;
    }
    float size_in_image() {
        return image_item.size;
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
    bool vel_valid = false;
    bool acc_valid = false;
    bool out_of_image() {return predicted_image_item.out_of_image;}
    double time = 0;
    float yaw_deviation = 0;
    bool yaw_deviation_valid = 0;
    bool using_prediction = false;
};

[[maybe_unused]] static const char* false_positive_names[] = {"fp_not_a_fp",
                                                              "fp_short_detection",
                                                              "fp_static_location",
                                                              "fp_bkg",
                                                              "fp_too_big",
                                                              "fp_too_far"
                                                             };
enum false_positive_type {
    fp_not_a_fp = 0,
    fp_short_detection,
    fp_static_location,
    fp_bkg,
    fp_too_big,
    fp_too_far
};

struct FalsePositive {
    FalsePositive(WorldItem world_item,false_positive_type type_,double last_seen_time_) {
        im_size = world_item.image_item.size;
        im_pt = world_item.image_item.pt();
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
