#pragma once
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "plane.h"
#include "cam.h"

struct CornerPoint {
    cv::Point3f pos = {0};
    std::array<uint, 3> intersecting_planes = {0};
    CornerPoint(cv::Point3f p, uint p1, uint p2, uint p3) : pos(p),intersecting_planes({p1, p2, p3}) {}
};

enum safety_margin_types {
    bare,
    relaxed,
    strict,
    number_safety_margin_types // only to get the length of the enum, must be the last element
};

inline const char* safety_margin_types_str[] = {
    "bare",
    "relaxed",
    "strict",
    "number_safety_margin_types"
};
class FlightAreaConfig {
private:
    // Params:
    float bottom_plane_above_pad = 0.1f;

    // Data;
    Cam* _cam;
    std::string _name;
    safety_margin_types _safety_margin_type;
    std::vector<Plane> _planes = {};
    std::vector<CornerPoint> corner_points;
    ViewLimit view_data;

    //Methods
    void create_camera_planes();
    void rotate_hoirzontal_planes_inwards(float angle);
    void rotate_vertical_planes_inwards(float angle);
    void apply_safety_angle(float angle);
    void find_active_planes_and_their_corner_points();
    cv::Point3f project_into_flight_area_towards_point(cv::Point3f point, cv::Point3f drone_pos, std::vector<bool> violated_planes);
    cv::Point3f project_to_closest_point_in_flight_area(cv::Point3f point, std::vector<bool> violated_planes);
    cv::Point3f project_inside_plane_polygon(cv::Point3f point, std::vector<CornerPoint> corner_points);
    bool in_plane_polygon(cv::Point3f point, std::vector<CornerPoint> corner_points);
    std::vector<CornerPoint> corner_points_of_plane(uint plane_id);
    bool vertices_on_one_edge(CornerPoint cp1, CornerPoint cp2);

    float safety_margin(safety_margin_types type) {
        switch(type) {
        case bare:
            return 0.;
        case relaxed:
            return 0.15f;
        case strict:
            return 0.3f;
        default:
            return 0.3f;
        }
    };

public:
    FlightAreaConfig() {}
    FlightAreaConfig(Cam* cam, std::string name, safety_margin_types safety_margin_type): _cam(cam), _name(name),_safety_margin_type(safety_margin_type), view_data(cam->view_limits()) {
        create_camera_planes();
    }
    void update_config();

    void add_plane(Plane plane);
    void add_plane(cv::Point3f support_vector, cv::Point3f normal_vector, plane_types type);
    void reindex_planes();
    void update_bottom_plane_based_on_blink(float pad_height);
    void apply_safety_margin(safety_margin_types safety_margin_type);

    bool inside(cv::Point3f point);
    std::tuple<bool, std::vector<bool>> find_violated_planes(cv::Point3f point);
    cv::Point3f move_inside(cv::Point3f point);
    cv::Point3f move_inside(cv::Point3f point, cv::Point3f drone_pos);

    uint n_planes() {return _planes.size();};
    std::string name() const {return _name;};
    safety_margin_types safety_margin_value() const {return _safety_margin_type;};
    std::string safety_margin_str() const {return safety_margin_types_str[_safety_margin_type];};
    std::vector<Plane> planes() const {return _planes;};
    Plane plane(uint plane_id) const {return _planes.at(plane_id);};
    void cout_debug_info();
};


std::ostream &operator<<(std::ostream &os, const FlightAreaConfig &f);
std::ostream &operator<<(std::ostream &os, const CornerPoint &cp);
