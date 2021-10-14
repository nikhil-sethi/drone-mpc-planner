#include "flightareaconfig.h"
#include "linalg.h"

void FlightAreaConfig::create_camera_planes() {
    _planes.push_back(Plane(1, view_data.point_left_bottom,   view_data.point_right_bottom, lower_plane, _planes.size()));
    _planes.push_back(Plane(-1, view_data.point_left_top,      view_data.point_right_top,    top_plane,   _planes.size()));
    _planes.push_back(Plane(-1, view_data.point_left_bottom,   view_data.point_left_top,     left_plane,  _planes.size()));
    _planes.push_back(Plane(1, view_data.point_right_bottom, view_data.point_right_top,    right_plane, _planes.size()));

    cv::Point3f camera_normal(cv::Point3f(0, -sinf(_cam->camera_pitch()* deg2rad), -cosf(_cam->camera_pitch()* deg2rad)));
    add_plane(0.85f * camera_normal, camera_normal, camera_protector_plane);
    apply_safety_angle(15.f * deg2rad);
}

void FlightAreaConfig::add_plane(cv::Point3f support_vector, cv::Point3f normal_vector, plane_types type) {
    _planes.push_back(Plane(support_vector, normal_vector, type, _planes.size()));
}

void FlightAreaConfig::add_plane(Plane plane) {
    _planes.push_back(plane);
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

void FlightAreaConfig::rotate_hoirzontal_planes_inwards(float angle) {
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
    //rotate_hoirzontal_planes_inwards(angle);
    rotate_vertical_planes_inwards(angle);
}

void FlightAreaConfig::update_config() {
    find_active_planes_and_their_corner_points();

    if (corner_points.max_size() == 0) {
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

    corner_points.clear();
    for (auto &plane : _planes)
        plane.is_active = false;

    for (auto &plane1 : _planes) {
        for (auto &plane2 : _planes) {
            for (auto &plane3 : _planes) {
                if (plane1.id < plane2.id && plane2.id < plane3.id && plane3.id > plane1.id) {
                    cv::Point3f intrs_pnt = intersection_of_3_planes(plane1, plane2, plane3);

                    bool in_view = true;
                    for (auto plane : _planes) {
                        if (!plane.on_normal_side(intrs_pnt, eps)) {
                            in_view = false;
                            break;
                        }
                    }
                    if (in_view == true) {
                        CornerPoint cp = CornerPoint(intrs_pnt, plane1.id, plane2.id, plane3.id);
                        corner_points.push_back(cp);
                        plane1.is_active = true;
                        plane2.is_active = true;
                        plane3.is_active = true;
                    }

                }
            }
        }
    }
}

std::vector<CornerPoint> FlightAreaConfig::corner_points_of_plane(uint plane_id) {
    std::vector<CornerPoint> plane_corner_points;
    for (auto pnt : corner_points) {
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
            if (!plane.on_normal_side(point))
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
            if (!plane.on_normal_side(point)) {
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
    float ref_distance = 999.f;

    for (auto plane : _planes) {
        if (violated_planes.at(plane.id) && plane.is_active) {
            cv::Point3f projected_point = point + fabs(plane.distance(point)) * plane.normal;

            // check if point is correct plane segment (the one which actually limits the volume)
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

    for (uint i = 0; i < plane_corner_points.size() - 1; i++) {
        for (uint j = i + 1; j < plane_corner_points.size(); j++) {
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

bool FlightAreaConfig::in_plane_polygon(cv::Point3f point, std::vector<CornerPoint> plane_corner_points) {
    // If the plane is in the segment (polygon) then the sum of all angles between the point and the pairwise vertices is 2pi or -2pi.
    // http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    float angle_sum = 0;
    for (uint i = 0; i < plane_corner_points.size() - 1; i++) {
        for (uint j = i + 1; j < plane_corner_points.size(); j++) {
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

void FlightAreaConfig::cout_debug_info() {
    std::cout << *this << std::endl;
    std::cout << "CornerPoints:" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    for (auto plane : _planes) {
        std::cout << "Plane: " << plane.id << std::endl;

        for (auto pnt : corner_points) {
            for (auto id : pnt.intersecting_planes) {
                if (id == plane.id)
                    std::cout << pnt;
            }
        }
    }
    std::cout << "-------------------------------------------------" << std::endl;
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
