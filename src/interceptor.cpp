#include "interceptor.h"
#include "opencv2/imgproc.hpp"

void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr, VisionData * visdat) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;
    _visdat = visdat;
}
void Interceptor::update(bool drone_at_base) {


    switch (_interceptor_state) {
     case  is_init: {
        _interceptor_state = is_waiting_for_target;
        _intercept_pos = {0,0,0};
    } FALLTHROUGH_INTENDED; case is_waiting_for_target: {
        _intercept_vel = {0,0,0};
        _intercept_acc = {0,0,0};
        _count_insect_not_in_range++;

        if ( (norm(_intercept_vel) > 0 || _itrkr->foundL)){ //TODO: whatsup with the intercept_vel....?
            _interceptor_state = is_waiting_in_reach_zone;
        } else
            break;
    } FALLTHROUGH_INTENDED; case is_waiting_in_reach_zone: {
        _intercept_vel = {0,0,0};
        _intercept_acc = {0,0,0};


        if ( !_itrkr->foundL){
             _interceptor_state = is_waiting_for_target;
            break;
        }

        update_far_target(drone_at_base);
        update_insect_in_range();
        if (!_count_insect_not_in_range)
            _interceptor_state = is_move_to_intercept;
        break;
    } case is_move_to_intercept: { // move max speed to somewhere close of the insect, preferably 20cm below behind.
        if  (!_itrkr->foundL) {
          _interceptor_state = is_waiting_for_target;
          break;
        }
         update_far_target(drone_at_base);
         update_insect_in_range();
        if (_count_insect_not_in_range>5){
           _interceptor_state = is_waiting_in_reach_zone;
           break;
         }

         if (fabs(_horizontal_separation)<0.5f && fabs(_vertical_separation)<0.4f){
             _interceptor_state = is_close_chasing;
         } else
             break;

         if (_intercept_pos.y< _dtrkr->Drone_Startup_Location().y + 0.15f)
             _intercept_pos.y  = _dtrkr->Drone_Startup_Location().y + 0.15f;

    } FALLTHROUGH_INTENDED; case is_close_chasing: {
        if  (!_itrkr->foundL) {
          _interceptor_state = is_waiting_for_target;
          break;
        }
         update_close_target();
         update_insect_in_range();
         if (_intercept_pos.y< _dtrkr->Drone_Startup_Location().y + 0.15f)
             _intercept_pos.y  = _dtrkr->Drone_Startup_Location().y + 0.15f;
        if (_count_insect_not_in_range>5){
           _interceptor_state = is_waiting_in_reach_zone;
           break;
         }
        if (fabs(_horizontal_separation)>0.7f){
            _interceptor_state = is_move_to_intercept;
        }
        break;
    }
    }

}

void Interceptor::update_far_target(bool drone_at_base){
    track_data itd = _itrkr->Last_track_data();
    cv::Point3f insect_pos = {itd.posX,itd.posY,itd.posZ};
    cv::Point3f insect_vel = {itd.svelX,itd.svelY,itd.svelZ};
    cv::Point3f insect_acc = {itd.saccX,itd.saccY,itd.saccZ};

    track_data dtd = _dtrkr->Last_track_data();
    cv::Point3f drone_vel = {dtd.svelX,dtd.svelY,dtd.svelZ};
    cv::Point3f drone_pos;
    if (drone_at_base)
        drone_pos = _dtrkr->Drone_Startup_Location();
    else {
        drone_pos = {dtd.posX,dtd.posY,dtd.posZ};
    }

    float tti = calc_tti(insect_pos,_intercept_vel,drone_pos,drone_vel,drone_at_base);
    float half_tti = tti/2.f; // only predict the location of the insect for a partion of the actual time we need to get there
    _intercept_pos = insect_pos + (_intercept_vel*half_tti);
    _intercept_pos.y -= 0.2f; // put the drone a bit below the insect
    _intercept_vel = insect_vel;
    _intercept_acc = insect_acc;

    _horizontal_separation = norm(cv::Point2f(drone_pos.x,drone_pos.z) - cv::Point2f(insect_pos.x,insect_pos.z));
    _vertical_separation = insect_pos.y-drone_pos.y;
}
void Interceptor::update_close_target(){
    track_data itd = _itrkr->Last_track_data();
    cv::Point3f insect_pos = {itd.posX,itd.posY,itd.posZ};
    cv::Point3f insect_vel = {itd.svelX,itd.svelY,itd.svelZ};
    cv::Point3f insect_acc = {itd.saccX,itd.saccY,itd.saccZ};

    track_data dtd = _dtrkr->Last_track_data();
    cv::Point3f drone_pos = {dtd.posX,dtd.posY,dtd.posZ};

    _intercept_pos = insect_pos;
    _intercept_vel = insect_vel;
    _intercept_acc = insect_acc;

    cv::Point3f vector = insect_pos-drone_pos;
    float norm_vector = norm(vector);
    _intercept_vel += vector/norm_vector*0.6f; //TODO: check if this work??

    _horizontal_separation = norm(cv::Point2f(drone_pos.x,drone_pos.z) - cv::Point2f(insect_pos.x,insect_pos.z));
    _vertical_separation = insect_pos.y-drone_pos.y;
}

void Interceptor::update_insect_in_range() {
    //calculate if the drone will stay within the camera borders where it still can be controlled:
    if (_intercept_pos.x > _intercept_pos.z+0.3f &&
            _intercept_pos.x < -_intercept_pos.z-0.3f &&
            abs(_intercept_pos.x) < 2.0f &&
            _intercept_pos.y > _intercept_pos.z*1.5f &&
            _intercept_pos.y < -0.3f &&
            _intercept_pos.z > -3.0f &&
            _intercept_pos.z < -1.0f) {
        _count_insect_not_in_range= 0;
    } else {
        _count_insect_not_in_range++;

    }
}

float Interceptor::calc_tti(cv::Point3f insect_pos,cv::Point3f insect_vel,cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off){
    //basic physics:
    //x = 0.5at² t = sqrt((2x)/a)
    //x = vt t = x/v
    //v = at t = v/a

    //TODO: put in some actually measured values:
    const float drone_vel_max = 10; // [m/s]
    const float drone_acc_max = 20; // [m/s^2]
    const float t_estimated_take_off = 0.32f; //[s]

    float ic_dx = norm(insect_pos-drone_pos);
    float ic_dv = norm(insect_vel-drone_vel);
    float vi = norm(insect_vel);
    float vd = norm(drone_vel);

    /*Part 1, match_v*/
    //First the drone needs some time to match speeds. During this time its average speed v_d_avg = 0.5*(vd-vi) + vd
    //Time needed to match speed:
    float t_match_v = ic_dv / drone_acc_max;
    //Average speed and distance traveled during this time
    float v_d_avg = 0.5f * ic_dv + vd;
    float x_match_v = t_match_v * v_d_avg;

    /*Part 2, intercept the remaining distance with max v*/
    float ic_dx_2 = ic_dx - x_match_v;
    float t_ic = 0;
    if (ic_dx_2>0) {
        t_ic = ic_dx_2 / (drone_vel_max-vi);
    }

    //time to intercept is time of part 1 and 2 summed
    float tti =t_match_v + t_ic;

    //plus the time needed to takeoff, if not already flying
    if (drone_taking_off){
        float t_remaining_takeoff = t_estimated_take_off - _dtrkr->time_since_take_off();
        if (t_remaining_takeoff<0)
            t_remaining_takeoff = 0;

        tti += t_remaining_takeoff;
    }
    _tti = tti;
    return tti;
}

void Interceptor::intercept_spiral() {
    //1. asume moth will spiral down.
    //2. assume the radius of the spiral is inversely proportional to vertical speed
    //3. assume that the insect will go straight down when achieving some vertical speed v_t

    //below v_t, the current center of the spiral can be calculated as follows:
    //[x_center,y_center] = normalize([vx_insect,vy_insect]) * r - [ x_insect,y_insect]

    //    cv::Point2f ccircle;
    //    float r = 1.f / _itrkr->get_last_track_data().svelZ;
    //    cv::Point2f v2d_insect(_itrkr->get_last_track_data().svelX,_itrkr->get_last_track_data().svelY);
    //    cv::Point2f direction_insect = v2d_insect/ cv::norm(v2d_insect);
    //    ccircle.x = direction_insect.x*r - _itrkr->get_last_track_data().posX;
    //    ccircle.y = direction_insect.y*r - _itrkr->get_last_track_data().posY;


    //    if (_itrkr->n_frames_tracking>2) {
    //        std::vector<cv::Point2f> pts;
    //        cv::Point2f p0(_itrkr->track_history.at(0).posX,_itrkr->track_history.at(0).posZ);
    //        cv::Point2f p1(_itrkr->track_history.at(1).posX,_itrkr->track_history.at(1).posZ);
    //        cv::Point2f p2(_itrkr->track_history.at(2).posX,_itrkr->track_history.at(2).posZ);

    //        cv::Point2f a = p0-p2;
    //        float d = fabs(a.x*p1.x + a.y+p1.y) / (powf(a.x,2) + powf(a.y,2));


    //        for (uint i = _itrkr->track_history.size()-5; i<_itrkr->track_history.size(); i++){
    //            if (_itrkr->track_history.at(i).valid) {
    //                float x = _itrkr->track_history.at(i).posX;
    //                float z = _itrkr->track_history.at(i).posZ;

    //                pts.push_back(cv::Point2f(x,z));
    //            }
    //        }

    //        cv::RotatedRect rr = cv::fitEllipse(pts);
    //    }
}



