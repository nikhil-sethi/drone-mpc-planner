#include "interceptor.h"
#include "opencv2/imgproc.hpp"

void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;
}
void Interceptor::update(bool drone_at_base) {

    cv::Point3f insectVel = {_itrkr->get_last_track_data().svelX,_itrkr->get_last_track_data().svelY,_itrkr->get_last_track_data().svelZ};


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

    if ( insectVelNorm > 0 ) {

        cv::Point3f insectPos = {_itrkr->get_last_track_data().sposX,_itrkr->get_last_track_data().sposY,_itrkr->get_last_track_data().sposZ};
        cv::Point3f dronePos = {_dtrkr->get_last_track_data().sposX,_dtrkr->get_last_track_data().sposY,_dtrkr->get_last_track_data().sposZ};

        if (drone_at_base)
            dronePos = {0.f,-MAX_BORDER_Y_DEFAULT,-1.07f}; // TODO: define this in a consistent way with flightplan waypoints

        //estimated time to interception

        //flight time is simply assumed to be linear with distance
        //float tti = norm(insectPos-dronePos)*2;

        float tti = 0.0;

        if (drone_at_base)
            tti += estimated_take_off_time; // take off t


        //calculate estimated interception location and speed:
        _estimated_interception_location = insectPos + (insectVel*tti);

        if (_estimated_interception_location.y < -2.0f)
            _estimated_interception_location.y = -2.0f;

        //TODO: _estimated_interception_speed = ...

        //calculate worst case deviation:

        //calculate if the drone will stay within the borders where it still can be controlled:
        if (_estimated_interception_location.x > -2.0f && _estimated_interception_location.x < 0.75f) {
            if (_estimated_interception_location.y > -2.0f && _estimated_interception_location.y < -0.1f) {
                if (_estimated_interception_location.z > -3.3f && _estimated_interception_location.z < -0.8f) {
                    _insect_in_range = true;
                }
            }
        }

    }

}

bool Interceptor::get_insect_in_range() {
    return _insect_in_range;
}

cv::Point3f Interceptor::get_intercept_position() {
    return _estimated_interception_location;
}

