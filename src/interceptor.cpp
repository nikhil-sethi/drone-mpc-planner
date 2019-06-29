#include "interceptor.h"
#include "opencv2/imgproc.hpp"

void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr, VisionData * visdat) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;
    _visdat = visdat;

    cv::invert(visdat->Qf,Qfi);

    _prev_estimated_interception_location = {0.f,0.f,0.f};
}
void Interceptor::update(bool drone_at_base) {

    track_data itd = _itrkr->Last_track_data();

    _insect_vel = {itd.svelX,itd.svelY,itd.svelZ};
    _insect_acc = {itd.saccX,itd.saccY,itd.saccZ};
    _insect_pos = {itd.posX,itd.posY,itd.posZ};

    update_insect_prediction(); // todo: move to itrkr

    double insectVelNorm = norm(_insect_vel);
    _insect_in_range = false;
    _count_insect_not_in_range++;

    if ( insectVelNorm > 0 || _itrkr->foundL ) {

        track_data dtd = _dtrkr->Last_track_data();
        cv::Point3f drone_pos;
        //cv::Point3f droneVel = {dtd.svelX,dtd.svelY,dtd.svelZ};
        if (drone_at_base)
            drone_pos = _dtrkr->Drone_Startup_Location();
        else {
            drone_pos = {dtd.posX,dtd.posY,dtd.posZ};
        }



        //estimated time to interception

        //flight time is simply assumed to be linear with distance
        float tti = norm(insectPos-dronePos)*0.7;

        if (drone_at_base)
            tti += estimated_take_off_time; // take off t

        tti = 0.0;

        cv::Point2f insectPos2D,dronePos2D;
        insectPos2D.x = _insect_pos.x;
        insectPos2D.y = _insect_pos.z;
        dronePos2D.x = drone_pos.x;
        dronePos2D.y = drone_pos.z;

        _estimated_interception_location = _insect_pos + (_insect_vel*tti) + (0.5f*_insect_acc*tti*tti);
        _estimated_interception_location.y += 0.01f;

        float horizontal_separation = norm(dronePos2D-insectPos2D);
        float vertical_separation = _insect_pos.y-drone_pos.y;
        if (horizontal_separation<0.5f && vertical_separation>0.1f && vertical_separation<0.6f){
            final_approach = true;
        } else {
//            _estimated_interception_location = insectPos + (insectVel*tti);
//            _estimated_interception_location.y -= 0.01f;
        }

//        if (insectPos.y<dronePos.y )
//            final_approach = false;

        if (final_approach){
            _insect_pos.y -= 0.1f;
            cv::Point3f vector = _insect_pos-drone_pos;
            float norm_vector = norm(vector);
            //_estimated_interception_location = dronePos + vector/norm_vector*0.3f;
            _insect_vel += vector/norm_vector*0.6f;

//            float interception_vel = 0.6f;
//            float norm_vector = norm(droneVel);
//            insectVel += droneVel/norm_vector*interception_vel;

        }
        //_estimated_interception_location.y += 0.1f;

        // safe zone above flowers
        if (_estimated_interception_location.y < -1.3f)
            _estimated_interception_location.y = -1.3f;

        if (_estimated_interception_location.y > -0.1f)
            _estimated_interception_location.y = -0.1f;

        //TODO: _estimated_interception_speed = ...

        //calculate worst case deviation:

        //calculate if the drone will stay within the camera borders where it still can be controlled:
        if (_estimated_interception_location.x > _estimated_interception_location.z+0.3f && _estimated_interception_location.x < -_estimated_interception_location.z-0.3f && abs(_estimated_interception_location.x) < 2.0f) {
            if (_estimated_interception_location.y > _estimated_interception_location.z*1.5f && _estimated_interception_location.y < -0.3f) {
                if (_estimated_interception_location.z > -3.0f && _estimated_interception_location.z < -1.0f) {
                    _insect_in_range = true;
                    _count_insect_not_in_range = 0;
                    _prev_estimated_interception_location = _estimated_interception_location;
                }
            }
        }

    }

}

void Interceptor::update_insect_prediction() {

    // predict insect position for next frame
    float dt_pred = 1.f/VIDEOFPS;
    cv::Point3f predicted_pos = _insect_pos + _insect_vel*dt_pred;

    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    cv::Point3f tmp(predicted_pos.x,predicted_pos.y,predicted_pos.z);

    //derotate camera and convert to double:
    cv::Point3d tmpd;
    float theta = -_visdat->camera_angle * deg2rad;
    float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
    tmpd.z = -tmp.y * sinf(theta) + tmp.z * cosf(theta);
    tmpd.y = temp_y;
    tmpd.x = tmp.x;

    world_coordinates.push_back(tmpd);
    cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);

    //update tracker with prediciton
    cv::Point2f image_location;
    image_location.x= camera_coordinates.at(0).x/IMSCALEF;
    image_location.y= camera_coordinates.at(0).y/IMSCALEF;

    if (image_location.x < 0)
        image_location.x = 0;
    else if (image_location.x >= IMG_W/IMSCALEF)
        image_location.x = IMG_W/IMSCALEF-1;
    if (image_location.y < 0)
        image_location.y = 0;
    else if (image_location.y >= IMG_H/IMSCALEF)
        image_location.y = IMG_H/IMSCALEF-1;

    _itrkr->predicted_locationL_last.x = image_location.x;
    _itrkr->predicted_locationL_last.y = image_location.y;

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

bool Interceptor::insect_in_range() {
    return _insect_in_range;
}

bool Interceptor::insect_cleared() {
    return _count_insect_not_in_range > 60*3;
}

void Interceptor::reset_insect_cleared() {
    _count_insect_not_in_range = 0;
}

cv::Point3f Interceptor::intercept_position() {
    return _estimated_interception_location;
}

cv::Point3f Interceptor::prev_intercept_position() {
    return _prev_estimated_interception_location;
}

cv::Point3f Interceptor::target_speed() {
    return _insect_vel;
}

cv::Point3f Interceptor::target_accelleration() {
    return _insect_acc;
}

