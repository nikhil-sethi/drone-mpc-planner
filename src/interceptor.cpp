#include "interceptor.h"


void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;
}
void Interceptor::update(bool drone_at_base) {

    cv::Point3f insectVel = {_itrkr->get_last_track_data().svelX,_itrkr->get_last_track_data().svelY,_itrkr->get_last_track_data().svelZ};

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

        //TODO: _estimated_interception_speed = ...

        //calculate worst case deviation:

        //calculate if the drone will stay within the borders where it still can be controlled:
        if (_estimated_interception_location.x > -0.5f && _estimated_interception_location.x < 0.75f) {
            if (_estimated_interception_location.y > -2.0f && _estimated_interception_location.y < -0.1f) {
                if (_estimated_interception_location.z > -2.5f && _estimated_interception_location.z < -0.8f) {
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

