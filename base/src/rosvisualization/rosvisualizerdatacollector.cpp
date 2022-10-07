#include "rosvisualizerdatacollector.h"
#include "flightareaconfig.h"

void RosVisualizerDataCollector::init(Patser *patser) {
    ros_interface.init();
    _patser = patser;
    _dtrkr = &(patser->drone.tracker);
    _interceptor = &(patser->interceptor);
    _dctrl = &(patser->drone.control);
    _dnav = &(patser->drone.nav);
    _flight_area = &(patser->flight_area);
#ifdef OPTI_ROSVIS
    _interceptor->ros_interface(&ros_interface);
#endif
}

void RosVisualizerDataCollector::update() {
    tracking::TrackData drone = _dtrkr->last_track_data();
    ros_interface.drone(drone);
    ros_interface.drone_input(drone, _dctrl->applied_acceleration);
    ros_interface.update_drone_path(drone);

    _itrkr = _patser->interceptor.target_insecttracker();
    if (_itrkr)
        ros_interface.insect(_itrkr->last_track_data());

    ros_interface.flightarea(_flight_area->flight_area_config(relaxed));

    ros_interface.publish();

}
