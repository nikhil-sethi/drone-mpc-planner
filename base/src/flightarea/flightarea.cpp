#include <experimental/filesystem>
#include "flightarea.h"
#include "linalg.h"
#include "location.h"

#ifndef UNIT_TESTING
void FlightArea::init(std::string replay_dir, Cam *cam) {
    FlightAreaConfig bare_area(cam, default_config_name, bare);
    bare_area.add_plane(cv::Point3f(0, 0, -10), cv::Point3f(0, 0, 1), back_plane);

    std::vector<Plane> location_planes = deserialize_location(replay_dir);

    for (auto plane : location_planes) {
        bare_area.add_plane(plane);
    }
    bare_area.reindex_planes();
    bare_area.update_config();

    flight_area_configs[bare] = bare_area;


    for (uint i = 1; i < number_safety_margin_types; i++) {
        FlightAreaConfig tmp = bare_area;
        tmp.apply_safety_margin(static_cast<safety_margin_types>(i));
        flight_area_configs[static_cast<safety_margin_types>(i)] = tmp;
    }
}

std::vector<Plane> FlightArea::deserialize_location(std::string replay_dir) {
    XML_Location location;
    if (replay_dir == "") {
        if (!file_exist("../xml/locations/" + pparams.location + ".xml"))
            throw std::runtime_error("Location xml does not exist: " + pparams.location);
        location.deserialize("../xml/locations/" + pparams.location + ".xml");
        std::experimental::filesystem::copy("../xml/locations/" + pparams.location + ".xml", data_output_dir + "location.xml", std::experimental::filesystem::copy_options::update_existing);
    } else {
        location.deserialize(replay_dir + "/location.xml");
    }

    return location.planes();
}

#else
void FlightArea::init(std::vector<Plane> planes) {
    FlightAreaConfig bare_area(default_config_name, bare);

    for (auto plane : planes) {
        bare_area.add_plane(plane);
    }
    bare_area.reindex_planes();
    bare_area.update_config();

    flight_area_configs[bare] = bare_area;

    for (uint i = 1; i < number_safety_margin_types; i++) {
        FlightAreaConfig tmp = bare_area;
        tmp.apply_safety_margin(static_cast<safety_margin_types>(i));
        flight_area_configs[static_cast<safety_margin_types>(i)] = tmp;
    }
}
#endif

void FlightArea::update_bottom_plane_based_on_blink(float pad_height) {
    if (pad_height < 0) {
        for (auto &flight_area_config_pair : flight_area_configs) {
            flight_area_config_pair.second.update_bottom_plane_based_on_blink(pad_height);
#if FLIGHT_AREA_DEBUG
            flight_area_config_pair.second.cout_debug_info();
#endif
        }
    }
}


void FlightArea::set_vertical_camera_plane(float z) {
    if (z < 0) {
        for (auto &flight_area_config_pair : flight_area_configs) {
            flight_area_config_pair.second.add_plane(cv::Point3f(0, 0, z), cv::Point3f(0, 0, -1), vertical_camera_plane);
            flight_area_config_pair.second.update_config();
#if FLIGHT_AREA_DEBUG
            flight_area_config_pair.second.cout_debug_info();
#endif
        }
    }
}

void FlightArea::remove_plane(plane_types plane) {
    for (auto &flight_area_config_pair : flight_area_configs) {
        flight_area_config_pair.second.remove_plane(plane);
    }
}

bool FlightArea::inside(cv::Point3f point, safety_margin_types safety_margin_type) {
    return flight_area_config(safety_margin_type)->inside(point);
}

FlightAreaConfig *FlightArea::flight_area_config(safety_margin_types safety_margin_type) {
    std::map<safety_margin_types, FlightAreaConfig>::iterator ret = flight_area_configs.find(safety_margin_type);
    if (ret == flight_area_configs.end())
        throw std::runtime_error("FlightAreaConfiguration error: configuration missing! Implementation error!");

    return &(ret->second);
}

cv::Point3f FlightArea::move_inside(cv::Point3f point, safety_margin_types safety_margin_type) {
    return flight_area_config(safety_margin_type)->move_inside(point);
}
cv::Point3f FlightArea::move_inside(cv::Point3f point, safety_margin_types safety_margin_type, cv::Point3f drone_pos) {
    return flight_area_config(safety_margin_type)->move_inside(point, drone_pos);
}

std::vector<Plane> FlightArea::active_planes(safety_margin_types safety_margin_type) {
    return flight_area_config(safety_margin_type)->active_planes();
}

bool FlightArea::trajectory_in_view(std::vector<tracking::StateData> traj, safety_margin_types safety_margin_type) {
    FlightAreaConfig *areaconfig = flight_area_config(safety_margin_type);
    for (auto state : traj) {
        if (!areaconfig->inside(state.pos))
            return false;
    }
    return true;
}

std::vector<CornerPoint> FlightArea::corner_points(safety_margin_types safety_margin_type) {
    return flight_area_config(safety_margin_type)->corner_points();
}