#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"
#include "plane.h"
#include "tracking.h"
#include "flightareaconfig.h"
#include "cam.h"

#define FLIGHT_AREA_DEBUG false
class FlightArea {
private:
    std::string default_config_name;
    std::map<safety_margin_types, FlightAreaConfig> flight_area_configs;

    std::vector<Plane> deserialize_location(std::string replay_dir);

public:
    void init(std::string replay_dir, Cam* cam);
    void update_bottom_plane_based_on_blink(float pad_height);
    bool inside(cv::Point3f point, safety_margin_types margin);
    cv::Point3f move_inside(cv::Point3f point, safety_margin_types margin);
    cv::Point3f move_inside(cv::Point3f point, safety_margin_types margin, cv::Point3f drone_pos);
    FlightAreaConfig* flight_area_config(safety_margin_types margin);
    bool trajectory_in_view(std::vector<tracking::StateData> traj, safety_margin_types margin);
};
