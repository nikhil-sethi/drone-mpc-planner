#include "flightareaconfig.h"
#include "linalg.h"
#include <opencv2/core/types.hpp>
#include <vector>

#ifndef UNIT_TESTING
void FlightAreaConfig::create_camera_planes() {
    _planes.push_back(Plane(1, view_data.point_left_bottom,   view_data.point_right_bottom, lower_plane, _planes.size()));
    _planes.push_back(Plane(-1, view_data.point_left_top,      view_data.point_right_top,    top_plane,   _planes.size()));
    _planes.push_back(Plane(-1, view_data.point_left_bottom,   view_data.point_left_top,     left_plane,  _planes.size()));
    _planes.push_back(Plane(1, view_data.point_right_bottom, view_data.point_right_top,    right_plane, _planes.size()));

    cv::Point3f camera_normal(cv::Point3f(0, -sinf(_cam->camera_pitch()* deg2rad), -cosf(_cam->camera_pitch()* deg2rad)));
    add_plane(0.7f * camera_normal, camera_normal, camera_protector_plane);
    apply_safety_angle(5.f * deg2rad);
}
#endif

void FlightAreaConfig::add_plane(cv::Point3f support_vector, cv::Point3f normal_vector, plane_types type) {
    _planes.push_back(Plane(support_vector, normal_vector, type, _planes.size()));
}

void FlightAreaConfig::add_plane(Plane plane) {
    _planes.push_back(plane);
}

void FlightAreaConfig::remove_plane(plane_types type) {
    _planes.erase(std::remove_if(_planes.begin(), _planes.end(), [type](const Plane & x) { return x.type == type;}), _planes.end());
}

void FlightAreaConfig::reindex_planes() {
    uint id = 0;
    for (auto &plane : _planes) {
        plane.id = id;
        id++;
    }
}

void FlightAreaConfig::update_bottom_plane_based_on_blink(float pad_height) {
    bool plane_found = false;
    for (auto &plane : _planes) {
        if (plane.type == bottom_plane)
            plane_found = true;
        break;
    }

    if (!plane_found) {
        add_plane(cv::Point3f(0, pad_height + bottom_plane_above_pad + safety_margin(_safety_margin_type), 0), cv::Point3f(0, 1, 0), bottom_plane);
    }
    update_config();
}

void FlightAreaConfig::rotate_horizontal_planes_inwards(float angle) {
    for (auto &plane : _planes) {
        if (plane.type == top_plane) {
            plane.normal = rotate_vector_around_x_axis(plane.normal, -angle);
        } else if (plane.type == lower_plane) {
            plane.normal = rotate_vector_around_x_axis(plane.normal, angle);
        }
    }
}

void FlightAreaConfig::rotate_vertical_planes_inwards(float angle) {
    for (auto &plane : _planes) {
        if (plane.type == left_plane) {
            plane.normal = rotate_vector_around_y_axis(plane.normal, angle);
        } else if (plane.type == right_plane) {
            plane.normal = rotate_vector_around_y_axis(plane.normal, -angle);
        }
    }
}

void FlightAreaConfig::apply_safety_angle(float angle) {
    //rotate_horizontal_planes_inwards(angle);
    rotate_vertical_planes_inwards(angle);
}

void FlightAreaConfig::update_config() {
    find_active_planes_and_their_corner_points();

    if (_corner_points.max_size() == 0) {
        std::cout << "Whoops! Volume is empty! Check FlightAreaConfiguration and safety margins for " << _name << ", " << safety_margin_types_str[_safety_margin_type] << std::endl;
        throw std::runtime_error("FlightAreaConfiguration error: User error!");
    }

    for (auto plane : _planes) {
        if (corner_points_of_plane(plane.id).size() == 0 && plane.is_active) {
            std::cout << "A plane does not have any corner points but is active:" << _name << " - " << safety_margin_types_str[_safety_margin_type] << " - plane " << plane_types_str[plane.id] << std::endl;
            throw std::runtime_error("FlightAreaConfiguration error: Implementation error!");
        }
    }
}

void FlightAreaConfig::find_active_planes_and_their_corner_points() {
    const float eps = 0.001f; // Just a small number to prevent wrong results from rounding errors

    _corner_points.clear();
    _active_planes.clear();
    for (auto &plane : _planes)
        plane.is_active = false;

    for (auto &plane1 : _planes) {
        for (auto &plane2 : _planes) {
            for (auto &plane3 : _planes) {
                if (plane1.id < plane2.id && plane2.id < plane3.id && plane3.id > plane1.id) {
                    cv::Point3f intrs_pnt = intersection_of_3_planes(&plane1, &plane2, &plane3);

                    if (!std::isnan(intrs_pnt.x)) {
                        bool in_view = true;
                        for (auto plane : _planes) {
                            if (!plane.on_normal_side(intrs_pnt, eps)) {
                                in_view = false;
                                break;
                            }
                        }
                        if (in_view) {
                            CornerPoint cp = CornerPoint(intrs_pnt, plane1.id, plane2.id, plane3.id);
                            _corner_points.push_back(cp);
                            mark_active_planes(plane1, plane2, plane3);
                        }
                    }
                }
            }
        }
    }
}

std::vector<CornerPoint> FlightAreaConfig::corner_points_of_plane(uint plane_id) {
    std::vector<CornerPoint> plane_corner_points;
    for (auto pnt : _corner_points) {
        for (auto corner_point_plane_id : pnt.intersecting_planes) {
            if (corner_point_plane_id == plane_id)
                plane_corner_points.push_back(pnt);
        }
    }
    return plane_corner_points;
}

void FlightAreaConfig::apply_safety_margin(safety_margin_types type) {
    if (_safety_margin_type != bare) {
        std::cout << "WARNING: Current configuration has already a applied safety margin: " << safety_margin_types_str[_safety_margin_type] << std::endl;
        std::cout << "FlightArea will get smaller than expexted" << std::endl;
    }
    _safety_margin_type = type;

    for (auto &plane : _planes) {
        plane.support += safety_margin(type) * plane.normal;
    }
    update_config();
}

/******************************* PLANE CHECK METHODS ************************/

bool FlightAreaConfig::inside(cv::Point3f point) {
    for (auto plane : _planes) {
        if (plane.is_active) {
            if (!plane.on_normal_side(point) && !plane.on_plane(point))
                return false;
        }
    }
    return true;
}

std::tuple<bool, std::vector<bool>> FlightAreaConfig::find_violated_planes(cv::Point3f point) {
    std::vector<bool> violated_planes;
    violated_planes.assign(_planes.size(), false);
    bool in_view = true;
    for (auto plane : _planes) {
        if (plane.is_active) {
            if (!plane.on_normal_side(point) && !plane.on_plane(point)) {
                in_view = false;
                violated_planes.at(plane.id) = true;
            }
        }

    }
    return std::tuple(in_view, violated_planes);
}

cv::Point3f FlightAreaConfig::move_inside(cv::Point3f point) {
    auto [in_view, violated_planes] = find_violated_planes(point);
    if (!in_view)
        return project_to_closest_point_in_flight_area(point, violated_planes);
    return point;
}
cv::Point3f FlightAreaConfig::move_inside(cv::Point3f point, cv::Point3f drone_pos) {
    auto [point_in_view, point_violated_planes] = find_violated_planes(point);

    if (!point_in_view) {
        auto drone_in_view = inside(drone_pos);
        if (drone_in_view)
            return project_into_flight_area_towards_point(point, drone_pos, point_violated_planes);
        else
            return project_to_closest_point_in_flight_area(point, point_violated_planes);

    }
    return point;
}

cv::Point3f FlightAreaConfig::project_to_closest_point_in_flight_area(cv::Point3f point, std::vector<bool> violated_planes) {
    // The closest point in the volume to the setpoint lies on the violated planes
    cv::Point3f closest_point = point;
    float ref_distance = std::numeric_limits<float>::max();;

    for (auto plane : _planes) {
        if (violated_planes.at(plane.id) && plane.is_active) {
            cv::Point3f projected_point = point + fabs(plane.distance(point)) * plane.normal;

            // check if point is in correct plane segment (the one which actually limits the volume)
            std::vector<CornerPoint> plane_corner_points = corner_points_of_plane(plane.id);
            bool in_polygon = in_plane_polygon(projected_point, plane_corner_points);
            if (!in_polygon)
                projected_point = project_inside_plane_polygon(projected_point, plane_corner_points);

            float distance = normf(projected_point - point);
            if (distance < ref_distance) {
                closest_point = projected_point;
                ref_distance = distance;
            }
        }
    }
    return closest_point;
}

cv::Point3f FlightAreaConfig::project_inside_plane_polygon(cv::Point3f point, std::vector<CornerPoint> plane_corner_points) {
    // https://stackoverflow.com/questions/42248202/find-the-projection-of-a-point-on-the-convex-hull-with-scipy?noredirect=1
    cv::Point3f closest_point = point;
    float ref_distance = 9999.f;

    for (size_t i = 0; i < plane_corner_points.size() - 1; i++) {
        for (size_t j = i + 1; j < plane_corner_points.size(); j++) {
            if (vertices_on_one_edge(plane_corner_points.at(i), plane_corner_points.at(j))) {
                cv::Point3f cmp_point = project_between_two_points(point, plane_corner_points.at(i).pos, plane_corner_points.at(j).pos);
                float distance = norm(cmp_point - point);
                if (distance < ref_distance) {
                    closest_point = cmp_point;
                    ref_distance = distance;
                }
            }
        }
    }
    return closest_point;
}

cv::Point3f FlightAreaConfig::project_towards_plane(cv::Point3f point, Plane plane, float distance) {
    return point - distance * plane.normal;
}

cv::Point3f FlightAreaConfig::project_onto_plane(cv::Point3f point, Plane plane) {
    return point + fabs(plane.distance(point)) * plane.normal;
}

bool FlightAreaConfig::in_plane_polygon(cv::Point3f point, std::vector<CornerPoint> plane_corner_points) {
    // If the plane is in the segment (polygon) then the sum of all angles between the point and the pairwise vertices is 2pi or -2pi.
    // http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    float angle_sum = 0;
    for (size_t i = 0; i < plane_corner_points.size() - 1; i++) {
        for (size_t j = i + 1; j < plane_corner_points.size(); j++) {
            if (vertices_on_one_edge(plane_corner_points.at(i), plane_corner_points.at(j))) {
                angle_sum += angle_between_points(plane_corner_points.at(i).pos, point, plane_corner_points.at(j).pos);
            }
        }
    }
    angle_sum = fabs(angle_sum);
    if (fabs(angle_sum - 2 * M_PIf32) < 0.0001f)
        return true;
    return false;
}

cv::Point3f FlightAreaConfig::project_into_flight_area_towards_point(cv::Point3f point, cv::Point3f drone_pos, std::vector<bool> violated_planes) {
    cv::Point3f closest_point = point;
    float ref_distance = 999.f;

    for (auto plane : _planes) {
        if (violated_planes.at(plane.id) && plane.is_active) {
            cv::Point3f projected_point = intersection_of_plane_and_line(plane.support, plane.normal, point, point - drone_pos);
            float distance = normf(projected_point - drone_pos);
            if (distance < ref_distance) {
                closest_point = projected_point;
                ref_distance = distance;
            }
        }
    }
    return closest_point;
}

bool FlightAreaConfig::vertices_on_one_edge(CornerPoint cp1, CornerPoint cp2) {
    uint common_planes = 0;
    for (auto intersecting_plane1 : cp1.intersecting_planes) {
        for (auto intersecting_plane2 : cp2.intersecting_planes) {
            if (intersecting_plane1 == intersecting_plane2)
                common_planes++;
        }
    }
    if (common_planes >= 2)
        return true;
    return false;
}


bool FlightAreaConfig::plane_in_active_planes(Plane plane) {
    for (auto &active_plane : _active_planes) {
        if (plane.id == active_plane.id) {
            return true;
        }
    }
    return false;
}

void FlightAreaConfig::mark_active_planes(Plane plane1, Plane plane2, Plane plane3) {
    _planes.at(plane1.id).is_active = true;
    _planes.at(plane2.id).is_active = true;
    _planes.at(plane3.id).is_active = true;

    if (_active_planes.size() == 0) {
        _active_planes.push_back(_planes.at(plane1.id));
        _active_planes.push_back(_planes.at(plane2.id));
        _active_planes.push_back(_planes.at(plane3.id));
        return;
    }

    if (!plane_in_active_planes(plane1))
        _active_planes.push_back(_planes.at(plane1.id));

    if (!plane_in_active_planes(plane2))
        _active_planes.push_back(_planes.at(plane2.id));

    if (!plane_in_active_planes(plane3))
        _active_planes.push_back(_planes.at(plane3.id));
}

std::vector<cv::Point3f> FlightAreaConfig::find_corner_points_of_plane(uint plane_id) {
    std::vector<cv::Point3f> corner_pnts = {};
    for (auto cp : corner_points()) {
        for (auto planei_id : cp.intersecting_planes) {
            if (planei_id == plane_id)
                corner_pnts.push_back(cp.pos);
        }
    }
    return corner_pnts;
}

std::vector<Plane> FlightAreaConfig::sort_planes_by_proximity(cv::Point3f point) {
    std::vector<Plane> _sorted_planes_by_proximity = _planes;
    std::sort(_sorted_planes_by_proximity.begin(), _sorted_planes_by_proximity.end(), [point](Plane p1, Plane p2) {
        return normf(p1.support - point) < normf(p2.support - point);
    });
    return _sorted_planes_by_proximity;
}

Plane FlightAreaConfig::find_most_constraining_plane(cv::Point3f point) {
    std::vector<Plane> _sorted_planes_by_proximity = _planes;
    std::sort(_sorted_planes_by_proximity.begin(), _sorted_planes_by_proximity.end(), [point](Plane p1, Plane p2) {
        if (p1.is_active && !p2.is_active)
            return true;
        else if (!p1.is_active && p2.is_active)
            return false;

        if (!(p1.on_normal_side(point) || p1.on_plane(point)) && (p2.on_normal_side(point) || p2.on_plane(point)))
            return true;
        else if ((p1.on_normal_side(point) || p1.on_plane(point)) && !(p2.on_normal_side(point) || p2.on_plane(point)))
            return false;
        else
            return normf(p1.support - point) < normf(p2.support - point);
    });
    for (auto plane : _sorted_planes_by_proximity) {
        if (plane.is_active)
            return plane;
    }
    return _sorted_planes_by_proximity.at(0); // not great
}

int FlightAreaConfig::find_next_non_parallel_plane(std::vector<Plane> sorted_planes, int plane_index) {
    bool _parallel = true;
    int N = plane_index;
    Plane _first_plane = sorted_planes[N];
    while (_parallel) {
        N++;
        Plane _second_plane = sorted_planes[N];
        _parallel = norm(_first_plane.normal.cross(_second_plane.normal)) < 0.0001;
        // if (N > len(_sorted_planes_by_proximity)) {
        //     std::cout << "No non-parallel plane found" << std::endl;
        //     return _first_plane;
        // }
    }
    return N;
}

int FlightAreaConfig::find_next_non_parallel_plane(std::vector<Plane> sorted_planes, int first_plane_index, int second_plane_index) {
    bool _parallel = true;
    int N = first_plane_index;
    Plane _first_plane = sorted_planes[N];
    Plane _second_plane = sorted_planes[second_plane_index];
    while (_parallel) {
        N++;
        Plane _third_plane = sorted_planes[N];
        _parallel = norm(_first_plane.normal.cross(_third_plane.normal)) < 0.0001;
        if (!_parallel) {
            _parallel = norm(_second_plane.normal.cross(_third_plane.normal)) < 0.0001;
        }
    }
    return N;
}

void FlightAreaConfig::cout_debug_info() {
    std::cout << *this << std::endl;
    std::cout << "CornerPoints:" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    for (auto plane : _planes) {
        std::cout << "Plane: " << plane.id << std::endl;

        for (auto pnt : _corner_points) {
            for (auto id : pnt.intersecting_planes) {
                if (id == plane.id)
                    std::cout << pnt;
            }
        }
    }
    std::cout << "-------------------------------------------------" << std::endl;
}


Plane FlightAreaConfig::active_back_plane() {
    std::vector<Plane> back_planes;
    for (auto plane : _planes) {
        if (plane.type == back_plane && plane.is_active)
            return plane;
    }

    throw std::runtime_error("No back plane found or back plane is inactive.");
}

std::ostream &operator<<(std::ostream &os, const FlightAreaConfig &f) {
    os << "-------------------------------------------------" << std::endl;
    os << "FlightAreaConfiguration: " << f.name() << " (safety_margin: " << f.safety_margin_str() << ")" << std::endl;
    os << "-------------------------------------------------" << std::endl;
    for (auto plane : f.planes())
        os << plane;
    os << "-------------------------------------------------" << std::endl;
    return os;
}

std::ostream &operator<<(std::ostream &os, const CornerPoint &cp) {
    os << "CornerPoint: pos: " << cp.pos << std::endl;
    return os;
}
