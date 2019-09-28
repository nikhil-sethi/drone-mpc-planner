#pragma once

#include "defines.h"
#include "dronecontroller.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "visiondata.h"
#include "smoother.h"

class DronePredictor {
public :
    void init(VisionData *visdat, DroneTracker *dtrk, InsectTracker *itrk, DroneController *dctrl);
    void update(bool drone_is_active, double time);

private:
    void swap_check(cv::Point3f rtp);

    InsectTracker *_itrk;
    DroneTracker *_dtrk;
    DroneController *_dctrl;
    VisionData *_visdat;

    bool initialized = false;
    Smoother roll_smth,throttle_smth,pitch_smth;
    Smoother roll_gain_smth,throttle_gain_smth,pitch_gain_smth;

    float dt_prev = 0;
    cv::Point3f pos_prev ={0};
    cv::Point3f vel_prev ={0};
    cv::Point3f rtp_prev ={0}; // roll, throttle, pitch -> same order as when used with pos xyz
    cv::Point3f rtp_gain_prev ={0};
    cv::Point3f acc_pred_prev ={0};
    cv::Point3f predicted_pos ={0};
    cv::Point3f drag_gain;

    Smoother swap_x;
    Smoother swap_y;
    Smoother swap_z;

    cv::Point3f tot_d = {0};
    cv::Point3f tot_i = {0};
    cv::Point3f tot_t = {0};




};
