#include "interceptor.h"


void Interceptor::init(DroneTracker * dtrkr, InsectTracker * itrkr) {
    _dtrkr = dtrkr;
    _itrkr = itrkr;
}
void Interceptor::update(bool drone_at_base) {
    float tti =0; //estimated time to interception

    if (drone_at_base)
        tti += estimated_take_off_time; // take off t


    //calculate estimated interception location and speed:
    //TODO: _estimated_interception_location = ...
    //TODO: _estimated_interception_speed = ...

    //calculate worst case deviation:

    //calculate if the drone will stay within the borders where it still can be controlled:
    //TODO: _insect_in_range = ...

}

bool Interceptor::get_insect_in_range() {
    return _insect_in_range;
}
