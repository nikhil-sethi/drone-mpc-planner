#include "interceptor.h"
#include "opencv2/imgproc.hpp"

void Interceptor::init(TrackerManager *trackers, VisionData *visdat, CameraVolume *camvol) {
    _trackers = trackers;
    _visdat = visdat;
    _camvol = camvol;
    insect_cleared_timeout = pparams.fps*0.5f;
}


void Interceptor::update(bool drone_at_base, double time) {

    switch (_interceptor_state) {
    case  is_init: {
        _interceptor_state = is_waiting_for_target;
        _intercept_pos = {0,0,0};
        [[fallthrough]];

    } case is_waiting_for_target: {
        _intercept_vel = {0,0,0};
        _intercept_acc = {0,0,0};
        _count_insect_not_in_range++;

        if ( (_trackers->insecttracker()->tracking() )){
            _interceptor_state = is_waiting_in_reach_zone;
        } else
            break;
        [[fallthrough]];
    } case is_waiting_in_reach_zone: {
        _intercept_vel = {0,0,0};
        _intercept_acc = {0,0,0};


        if ( !_trackers->insecttracker()->tracking() ){
            _interceptor_state = is_waiting_for_target;
            break;
        }

        update_far_target(drone_at_base);
        update_insect_in_range();

        if (_intercept_pos.y< _trackers->dronetracker()->drone_startup_location().y + minimal_height)
            _intercept_pos.y  = _trackers->dronetracker()->drone_startup_location().y + minimal_height;

        if (!_count_insect_not_in_range)
            _interceptor_state = is_move_to_intercept;
        break;
    } case is_flower_of_fire_intercept: {
        if  (!_trackers->insecttracker()->tracking()) {
            _interceptor_state = is_waiting_for_target;
            break;
        }
        update_flower_of_fire(time);
        update_insect_in_range();
        if (_intercept_pos.y< _trackers->dronetracker()->drone_startup_location().y + minimal_height)
            _intercept_pos.y  = _trackers->dronetracker()->drone_startup_location().y + minimal_height;
        if (_count_insect_not_in_range>5){
            _interceptor_state = is_waiting_in_reach_zone;
            break;
        }
        break;
    } case is_move_to_intercept: { // move max speed to somewhere close of the insect, preferably 20cm below behind.
        if  (!_trackers->insecttracker()->tracking()) {
            _interceptor_state = is_waiting_for_target;
            break;
        }
        update_far_target(drone_at_base);
        update_insect_in_range();

        if (_intercept_pos.y< _trackers->dronetracker()->drone_startup_location().y + minimal_height)
            _intercept_pos.y  = _trackers->dronetracker()->drone_startup_location().y + minimal_height;

        if (_count_insect_not_in_range>5){
            _interceptor_state = is_waiting_in_reach_zone;
            break;
        }

        if (fabs(_horizontal_separation) < 0.6f * _vertical_separation && _vertical_separation<0.8f && _vertical_separation>0)
            _interceptor_state = is_close_chasing;

        break;
    } case is_close_chasing: {
        if  (!_trackers->insecttracker()->tracking()) {
            _interceptor_state = is_waiting_for_target;
            break;
        }
        update_close_target();
        update_insect_in_range();
        if (_intercept_pos.y< _trackers->dronetracker()->drone_startup_location().y + minimal_height)
            _intercept_pos.y  = _trackers->dronetracker()->drone_startup_location().y + minimal_height;
        if (_count_insect_not_in_range>5){
            _interceptor_state = is_waiting_in_reach_zone;
            break;
        }
        //        if (fabs(_horizontal_separation)>0.3f){
        //            _interceptor_state = is_move_to_intercept;
        //        }
        break;
    }
    }

}

void Interceptor::update_flower_of_fire(double time){

    track_data itd = _trackers->insecttracker()->Last_track_data();
    cv::Point3f insect_pos = itd.pos();

    track_data dtd = _trackers->dronetracker()->Last_track_data();
    cv::Point3f drone_pos = dtd.pos();

    _horizontal_separation = norm(cv::Point2f(drone_pos.x,drone_pos.z) - cv::Point2f(insect_pos.x,insect_pos.z));
    _vertical_separation = insect_pos.y-drone_pos.y;

    float timef = static_cast<float>(time);

    cv::Point3f p_now = get_circle_pos(timef);
    cv::Point3f p_prev= get_circle_pos(timef-1.f/pparams.fps);

    cv::Point3f tmp_v = (p_now - p_prev)*static_cast<float>(pparams.fps);
    _intercept_acc = (tmp_v-_intercept_vel)*powf(static_cast<float>(pparams.fps),0.5);
    _intercept_pos = p_now + insect_pos;
    _intercept_vel = tmp_v;
}


cv::Point3f Interceptor::get_circle_pos(float timef){
    cv::Point3f p;
    p.x = (r_crcl1/100.f) * sinf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * cosf((v_crcl2/100.f)*timef);
    p.y = 0;
    p.z = (r_crcl1/100.f) * cosf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * sinf((v_crcl2/100.f)*timef);
    return p;
}

void Interceptor::update_far_target(bool drone_at_base){
    track_data itd = _trackers->insecttracker()->Last_track_data();
    cv::Point3f insect_pos = itd.pos();
    cv::Point3f insect_vel = itd.vel();
    cv::Point3f insect_acc = itd.acc();

    track_data dtd = _trackers->dronetracker()->Last_track_data();

    cv::Point3f drone_pos;
    if (drone_at_base)
        drone_pos = _trackers->dronetracker()->drone_startup_location();
    else {
        drone_pos = dtd.pos();
    }

    cv::Point3f drone_vel = dtd.vel();
    float tti = calc_tti(insect_pos,_intercept_vel,drone_pos,drone_vel,drone_at_base); // only used for viz _tti
    //    float half_tti = tti/2.f; // only predict the location of the insect for a partion of the actual time we need to get there
    _intercept_pos = insect_pos;
    _intercept_pos.y -= 0.1f; // put the drone a bit below the insect
    _intercept_vel = insect_vel;
    _intercept_vel.y = 0; // we don't want to follow the vertical speed of the insect, ever. TODO: improve this
    _intercept_acc = insect_acc;

    _horizontal_separation = norm(cv::Point2f(drone_pos.x,drone_pos.z) - cv::Point2f(insect_pos.x,insect_pos.z));
    _vertical_separation = insect_pos.y-drone_pos.y;
}
void Interceptor::update_close_target(){
    track_data itd = _trackers->insecttracker()->Last_track_data();
    cv::Point3f insect_pos = itd.pos();
    cv::Point3f insect_vel = itd.vel();
    cv::Point3f insect_acc = itd.acc();

    track_data dtd = _trackers->dronetracker()->Last_track_data();
    cv::Point3f drone_pos = dtd.pos();

    _intercept_pos = insect_pos;
    _intercept_acc = insect_acc;

    cv::Point3f vector = insect_pos-drone_pos;
    float norm_vector = norm(vector);

    insect_vel.y = 0; // we don't want to follow the vertical speed of the insect, ever
    insect_vel = 0.5f* insect_vel + vector/norm_vector*0.8f;
    if (norm_vector > 0.05f) // when insect and drone come close to each other, the blobs get fused..., so keep the previous speed vector in that case
        _intercept_vel = insect_vel;


    _horizontal_separation = norm(cv::Point2f(drone_pos.x,drone_pos.z) - cv::Point2f(insect_pos.x,insect_pos.z));
    _vertical_separation = insect_pos.y-drone_pos.y;
}

void Interceptor::update_insect_in_range() {
    //Checks whether the interception point is in view:
    if (_camvol->in_view(_intercept_pos, CameraVolume::relaxed)){
        _count_insect_not_in_range= 0;
    } else {
        _count_insect_not_in_range++;
    }

    //Checks whether the insect position is in a good area to hunt:
    if(_camvol->in_hunt_area (_trackers->dronetracker()->drone_startup_location(), _trackers->insecttracker()->world_item().pt))
        realy_nicely_in_the_middle = true;
    else
        realy_nicely_in_the_middle = false;
}

float Interceptor::calc_tti(cv::Point3f insect_pos,cv::Point3f insect_vel,cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off){
    //basic physics:
    //x = 0.5at² t = sqrt((2x)/a)
    //x = vt t = x/v
    //v = at t = v/a

    //TODO: put in some actually measured values:
    const float drone_vel_max = 5; // [m/s]
    const float drone_acc_max = 40; // [m/s^2]
    const double t_estimated_take_off = 0.29; //[s]

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
    double tti =t_match_v + t_ic;

    //plus the time needed to takeoff, if not already flying
    if (drone_taking_off){
        double t_remaining_takeoff = t_estimated_take_off - _trackers->dronetracker()->time_since_take_off();
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



