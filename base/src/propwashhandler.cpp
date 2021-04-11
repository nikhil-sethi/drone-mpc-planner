#include "propwashhandler.h"


bool PropwashHandler::prop_wash(cv::Point3f drone_velocity, cv::Point3f des_acc_drone) {
    float speed = normf(drone_velocity);

    if(speed==0 || normf(des_acc_drone)==0)
        return false; //Assume no propwash is happening

    float dot = des_acc_drone.dot(drone_velocity)/normf(des_acc_drone)/normf(drone_velocity);
    dot = 1 - (dot + 1)/2;
    float vel_ind = speed>3.f ? 1.f : speed/3.f;
    // std::cout << "Propwash-indicator: " << dot << ", " << vel_ind << "->" << dot*vel_ind << std::endl;
    if(dot*vel_ind > 0.75f) {
        return true;
    } else
        return false;
}

cv::Point3f PropwashHandler::update(cv::Point3f drone_velocity, cv::Point3f desired_acc, float thrust) {
    _propwash = prop_wash(drone_velocity, desired_acc);

    float control_margin = GRAVITY * tanf(acosf(GRAVITY/thrust)); // see doc/control-margin.svg
    if(_propwash) {
        std::cout << "Sodan" << std::endl;
        // desired_acc /= normf(desired_acc);
        // desired_acc *= 0.7f*control_margin;
    }

    return desired_acc;
}
