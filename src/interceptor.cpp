#include "interceptor.h"
#include "opencv2/imgproc.hpp"

void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;

    _prev_estimated_interception_location = {0.f,0.f,0.f};
}
void Interceptor::update(bool drone_at_base) {

    cv::Point3f insectVel = {_itrkr->Last_track_data().svelX,_itrkr->Last_track_data().svelY,_itrkr->Last_track_data().svelZ};


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



    double insectVelNorm = norm(insectVel);
    _insect_in_range = false;
    _count_insect_not_in_range++;

    if ( insectVelNorm > 0 || _itrkr->foundL ) {

        cv::Point3f insectPos = {_itrkr->Last_track_data().sposX,_itrkr->Last_track_data().sposY,_itrkr->Last_track_data().sposZ};
        cv::Point3f dronePos = {_dtrkr->Last_track_data().sposX,_dtrkr->Last_track_data().sposY,_dtrkr->Last_track_data().sposZ};

        if (drone_at_base)
            dronePos = {_dtrkr->Drone_Startup_Location().x,_dtrkr->Drone_Startup_Location().y,_dtrkr->Drone_Startup_Location().z};

        //estimated time to interception

        //flight time is simply assumed to be linear with distance
        float tti = norm(insectPos-dronePos)*0.7;

        //float tti = 0.0;

        if (drone_at_base)
            tti += estimated_take_off_time; // take off t

        cv::Point2f insectPos2D,dronePos2D;
        insectPos2D.x = insectPos.x;
        insectPos2D.y = insectPos.z;
        dronePos2D.x = dronePos.x;
        dronePos2D.y = dronePos.z;

        if (norm(dronePos2D-insectPos2D)<0.3 && _estimated_interception_location.y-dronePos.y<0.05f){
            final_approach = true;
        } else {
            _estimated_interception_location = insectPos + (insectVel*tti);
            _estimated_interception_location.y -= 0.3f;
        }

        if (insectPos.y<dronePos.y )
            final_approach = false;

        if (final_approach){
            cv::Point3f vector = insectPos-dronePos;
            _estimated_interception_location = dronePos + 2*vector;
        }
        //_estimated_interception_location.y += 0.1f;

        // safe zone above flowers
        if (_estimated_interception_location.y < -1.3f)
            _estimated_interception_location.y = -1.3f;

        //TODO: _estimated_interception_speed = ...

        //calculate worst case deviation:

        //calculate if the drone will stay within the camera borders where it still can be controlled:
        if (_estimated_interception_location.x > _estimated_interception_location.z && _estimated_interception_location.x < -_estimated_interception_location.z) {
            if (_estimated_interception_location.y > _estimated_interception_location.z*1.5f && _estimated_interception_location.y < -0.3f) {
                if (_estimated_interception_location.z > -5.3f && _estimated_interception_location.z < -0.8f) {
                    _insect_in_range = true;
                    _count_insect_not_in_range = 0;
                    _prev_estimated_interception_location = _estimated_interception_location;
                }
            }
        }

    }

}

bool Interceptor::get_insect_in_range() {
    return _insect_in_range;
}

bool Interceptor::get_insect_cleared() {
    return _count_insect_not_in_range > 60*3;
}

void Interceptor::reset_insect_cleared() {
    _count_insect_not_in_range = 0;
}

cv::Point3f Interceptor::get_intercept_position() {
    return _estimated_interception_location;
}

cv::Point3f Interceptor::get_prev_intercept_position() {
    return _prev_estimated_interception_location;
}


cv::Point3f Interceptor::get_target_speed() {
    return insectVel;
}

