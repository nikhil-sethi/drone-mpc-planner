#include "cameraview.h"

#include "linalg.h"

static const char* plane_names[] = { "top",
                                     "bottom",
                                     "back",
                                     "front",
                                     "left",
                                     "right",
                                     "camera"
                                   };

void CameraView::init(cv::Point3f point_left_top, cv::Point3f point_right_top, cv::Point3f point_left_bottom, cv::Point3f point_right_bottom,
                      float b_depth, float b_height, float camera_pitch_deg) {

    plane_normals.at(front_plane) = get_plane_normal_vector (point_left_bottom, point_right_bottom);
    plane_supports.at(front_plane) = (cv::Mat_<float>(3,1) << 0, 0, 0);

    plane_normals.at(top_plane) = get_plane_normal_vector (point_left_top, point_right_top);
    plane_normals.at(top_plane) *= -1; // let normal vector look inside the volume
    plane_supports.at(top_plane) = (cv::Mat_<float>(3,1) << 0, 0, 0);

    plane_normals.at(left_plane) = get_plane_normal_vector (point_left_bottom, point_left_top);
    plane_normals.at(left_plane) *= -1; // let normal vector look inside the volume
    plane_supports.at(left_plane) = (cv::Mat_<float>(3,1) << 0, 0, 0);

    plane_normals.at(right_plane) = get_plane_normal_vector (point_right_bottom, point_right_top);
    plane_supports.at(right_plane) = (cv::Mat_<float>(3,1) << 0, 0, 0);

    if(b_depth<-10.f)
        b_depth = -10.f; //10 m is the max supported depth by the realsense

    plane_normals.at(back_plane) = (cv::Mat_<float>(3,1) << 0, 0, 1);
    plane_supports.at(back_plane) = (cv::Mat_<float>(3,1) << 0, 0, b_depth);

    plane_normals.at(camera_plane) = (cv::Mat_<float>(3,1) << 0, -sin(camera_pitch_deg/180.L*M_PIf64), -cos(camera_pitch_deg/180.L*M_PIf64));
    plane_supports.at(camera_plane) = plane_normals.at(camera_plane)*0.85; //If the drones gets too close to the camera (~ 0.5 m) the drone position is not calculated anymore.

    plane_normals.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, 1, 0);
    p0_bottom_plane(b_height, false);

    plane_normals_hunt.at(bottom_plane) = plane_normals.at(bottom_plane);
    plane_normals_hunt.at(top_plane) = plane_normals.at(top_plane);
    plane_normals_hunt.at(front_plane) = (cv::Mat_<float>(3,1) << 0, 0, -1);
    plane_normals_hunt.at(back_plane) = plane_normals.at(back_plane);
    plane_normals_hunt.at(camera_plane) = plane_normals.at(camera_plane);

    point_left_bottom.x -= static_cast<float>(margin_left);
    point_left_top.x -= static_cast<float>(margin_left);
    point_right_bottom.x += static_cast<float>(margin_right);
    point_right_top.x += static_cast<float>(margin_right);

    plane_normals_hunt.at(left_plane) = get_plane_normal_vector (point_left_bottom, point_left_top);
    plane_normals_hunt.at(left_plane) *= -1;
    plane_normals_hunt.at(right_plane) = get_plane_normal_vector (point_right_bottom, point_right_top);

    plane_supports_hunt.at(top_plane) = plane_supports.at(top_plane) + margin_top * plane_normals.at(top_plane);
    plane_supports_hunt.at(front_plane) =  margin_front * plane_normals.at(front_plane);
    plane_supports_hunt.at(back_plane) = plane_supports.at(back_plane) + margin_back * plane_normals.at(back_plane);
    plane_supports_hunt.at(left_plane) = plane_supports.at(left_plane);
    plane_supports_hunt.at(right_plane) = plane_supports.at(right_plane);
    plane_supports_hunt.at(camera_plane) = plane_supports.at(camera_plane) + margin_camera*plane_normals.at(camera_plane);


    for (uint i = 0; i < N_PLANES; i++)
        adjacency_matrix.push_back(cv::Mat::zeros(N_PLANES, N_PLANES, CV_32S));

    adjacency_entry(1, bottom_plane, front_plane, right_plane);
    adjacency_entry(1, bottom_plane, right_plane, back_plane);
    adjacency_entry(1, bottom_plane, back_plane, left_plane);
    adjacency_entry(1, bottom_plane, left_plane, front_plane);
    adjacency_entry(1, front_plane, camera_plane, right_plane);
    adjacency_entry(1, front_plane, camera_plane, left_plane);
    adjacency_entry(1, camera_plane, top_plane, left_plane);
    adjacency_entry(1, camera_plane, top_plane, right_plane);
    adjacency_entry(1, top_plane, back_plane, left_plane);
    adjacency_entry(1, top_plane, back_plane, right_plane);

    update_plane_vertices();
}


void CameraView::p0_bottom_plane(float b_height, bool update_vertices) {
    plane_supports.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, b_height, 0);
    plane_supports_hunt.at(bottom_plane) = plane_supports.at(bottom_plane) + margin_bottom * plane_normals.at(bottom_plane);

    if(update_vertices)
        update_plane_vertices();
#if CAMERA_VIEW_DEBUGGING
    std::cout << *this <<std::endl;
#endif
}


void CameraView::update_plane_vertices() {
    vertices.clear();
    vertices_relaxed.clear();
    vertices_strict.clear();
    vertices_hunt.clear();
    cv::Mat vertice_pos;
    for(uint i=0; i<N_PLANES; i++) {
        for (uint j=i; j<N_PLANES; j++) {
            for (uint k=j; k<N_PLANES; k++) {
                if(adjacency_matrix.at(i).at<int>(j,k)==1) {
                    vertice_pos = intersection_of_3_planes(plane_supports.at(i), plane_normals.at(i),
                                                           plane_supports.at(j), plane_normals.at(j),
                                                           plane_supports.at(k), plane_normals.at(k));
                    vertices.push_back(intersection_point(vertice_pos, i, j, k));

                    vertice_pos = intersection_of_3_planes(plane_supports.at(i)+safety_margin(relaxed)*plane_normals.at(i), plane_normals.at(i),
                                                           plane_supports.at(j)+safety_margin(relaxed)*plane_normals.at(j), plane_normals.at(j),
                                                           plane_supports.at(k)+safety_margin(relaxed)*plane_normals.at(k), plane_normals.at(k));
                    vertices_relaxed.push_back(intersection_point(vertice_pos, i, j, k));

                    vertice_pos = intersection_of_3_planes(plane_supports.at(i)+safety_margin(strict)*plane_normals.at(i), plane_normals.at(i),
                                                           plane_supports.at(j)+safety_margin(strict)*plane_normals.at(j), plane_normals.at(j),
                                                           plane_supports.at(k)+safety_margin(strict)*plane_normals.at(k), plane_normals.at(k));
                    vertices_strict.push_back(intersection_point(vertice_pos, i, j, k));

                    vertice_pos = intersection_of_3_planes(plane_supports_hunt.at(i), plane_normals_hunt.at(i),
                                                           plane_supports_hunt.at(j), plane_normals_hunt.at(j),
                                                           plane_supports_hunt.at(k), plane_normals_hunt.at(k));
                    vertices_hunt.push_back(intersection_point(vertice_pos, i, j, k));
                }
            }
        }
    }
    assert(vertices.size()==N_PLANE_VERTICES);
}


std::tuple<bool, std::array<bool, N_PLANES>> CameraView::in_view(cv::Point3f p, view_volume_check_mode c) {
    std::array<bool, N_PLANES> violated_planes = {{false}};
    bool inview = true;

    // Attention check the negative case!
    for(uint i=0; i<N_PLANES; i++) {
        if(!on_normal_side(plane_supports.at(i) + safety_margin(c)*plane_normals.at(i), plane_normals.at(i), cv::Mat(p))) {
            inview = false;
            violated_planes.at(i) = true;
        }
    }
    return std::tuple<bool, std::array<bool, N_PLANES>>(inview, violated_planes);
}


CameraView::hunt_check_result CameraView::in_hunt_area(cv::Point3f moth_pos) {
    bool inhuntarea = true;
    std::array <bool, N_PLANES> violated_planes = {{false}};
    for(uint i=0; i<N_PLANES; i++) {
        if(!on_normal_side(plane_supports_hunt.at(i), plane_normals_hunt.at(i), cv::Mat(moth_pos))) {
            inhuntarea = false;
            violated_planes.at(i) = false;
        }
    }
    if(inhuntarea)
        return HuntVolume_OK;
    if(violated_planes.at(top_plane))
        return HuntVolume_Too_High;
    else if(violated_planes.at(bottom_plane))
        return HuntVolume_Too_Low;
    else
        return HuntVolume_Too_Far_Aside;
}


std::tuple<bool, std::array<bool, N_PLANES>> CameraView::check_distance_to_borders(track_data data_drone, float req_breaking_distance) {
    std::vector<cv::Point3f> p{data_drone.pos (), data_drone.vel ()};
    std::array<float, N_PLANES> distances_to_planes = calc_distance_to_borders (p);
    std::array<bool, N_PLANES> violated_planes = {{false}};
    bool planes_not_violated = true;

    for(int i=0; i<N_PLANES; i++) {
        if(distances_to_planes.at(i)>= 0 && distances_to_planes.at(i)<req_breaking_distance) {
            violated_planes.at(i) = true;
            planes_not_violated = false;
        }
    }

    return std::tuple<bool, std::array<bool, N_PLANES>>(planes_not_violated, violated_planes);
}


std::array<float, N_PLANES> CameraView::calc_distance_to_borders(std::vector<cv::Point3f> p) {
    cv::Mat pMat = (cv::Mat_<float_t>(3,2) << p[0].x, p[1].x, p[0].y, p[1].y, p[0].z, p[1].z);
    std::array<float, N_PLANES> distances_to_planes;

    cv::Mat plane = cv::Mat::zeros(cv::Size(2,3), CV_32F);
    for(uint i=0; i<N_PLANES; i++) {
        plane_supports.at(i).copyTo(plane.col(0));
        plane_normals.at(i).copyTo(plane.col(1));
        distances_to_planes.at(i) = distance_to_plane_along_vec(pMat, plane);
    }

    return distances_to_planes;
}


float CameraView::calc_shortest_distance_to_plane(cv::Point3f drone_pos, uint plane_idx, view_volume_check_mode cm) {
    cv::Point3f shifted_support_vector = support_vector(plane_idx) + safety_margin(cm)*normal_vector(plane_idx);
    return shortest_distance_to_plane(drone_pos, shifted_support_vector, normal_vector(plane_idx));
}


cv::Point3f CameraView::project_into_camera_volume(cv::Point3f pos_setpoint, cv::Point3f drone_pos, view_volume_check_mode cm, std::array<bool, N_PLANES> violated_planes) {
    cv::Point3f projected_point, closest_point;
    float ref_distance = 999.f;
    float cmp_distance;

    bool drone_inview;
    std::array<bool, N_PLANES> drone_violated_planes;
    std::tie(drone_inview, drone_violated_planes) = in_view(drone_pos, cm);
    if(!drone_inview)
        return project_into_camera_volume(pos_setpoint, cm, violated_planes);

    for(uint i=0; i<N_PLANES; i++) {
        if(violated_planes.at(i)) {
            projected_point = intersection_of_plane_and_line(cv::Point3f(plane_supports.at(i)) + safety_margin(cm)*cv::Point3f(plane_normals.at(i)),
                              cv::Point3f(plane_normals.at(i)),
                              pos_setpoint,
                              pos_setpoint - drone_pos);
            cmp_distance = normf(projected_point - drone_pos);
            if(cmp_distance<ref_distance) {
                closest_point = projected_point;
                ref_distance = cmp_distance;
            }
        }
    }
    return closest_point;
}


cv::Point3f CameraView::project_into_camera_volume(cv::Point3f pos_setpoint, view_volume_check_mode cm, std::array<bool, N_PLANES> violated_planes) {
    //  The closest point in the volume to the setpoint lies on the violated planes
    cv::Point3f projected_point, closest_point;
    float ref_distance = 999.f;
    float cmp_distance;
    for(uint i=0; i<N_PLANES; i++) {
        if(violated_planes.at(i)) {
            projected_point = pos_setpoint + abs(calc_shortest_distance_to_plane(pos_setpoint, i, cm))*cv::Point3f(plane_normals.at(i));

            // check if point is correct plane segment (the one which actually limits the volume)
            std::vector<intersection_point> _plane_vertices = plane_vertices(i, cm);
            bool in_segment = in_plane_segment(projected_point, _plane_vertices);
            if(!in_segment)
                projected_point = project_into_plane_segment(projected_point, _plane_vertices);

            cmp_distance = norm(projected_point-pos_setpoint);
            if(cmp_distance<ref_distance) {
                closest_point = projected_point;
                ref_distance = cmp_distance;
            }
        }
    }
    return closest_point;
}


bool CameraView::in_plane_segment(cv::Point3f p, std::vector<intersection_point> _plane_vertices) {
    // If the plane is in the segment (polygon) then the sum of all angles between the point and the pairwise vertices is 2pi or -2pi.
    // http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    float angle_sum = 0;
    for(uint i=0; i<_plane_vertices.size()-1; i++) {
        for(uint j=i+1; j<_plane_vertices.size(); j++) {
            if(vertices_on_one_edge(_plane_vertices.at(i), _plane_vertices.at(j))) {
                angle_sum += angle_between_points(cv::Point3f(_plane_vertices.at(i).pos), p, cv::Point3f(_plane_vertices.at(j).pos));
            }
        }
    }
    angle_sum = abs(angle_sum);
    if(abs(static_cast<double>(angle_sum)-2*M_PI)<0.0001)
        return true;
    return false;
}


cv::Point3f CameraView::project_into_plane_segment(cv::Point3f p, std::vector<intersection_point> _plane_vertices) {
    // https://stackoverflow.com/questions/42248202/find-the-projection-of-a-point-on-the-convex-hull-with-scipy?noredirect=1
    cv::Point3f closest_point, cmp_point;
    float ref_distance=9999.f;
    float cmp_distance;
    for(uint i=0; i<_plane_vertices.size()-1; i++) {
        for(uint j=i+1; j<_plane_vertices.size(); j++) {
            if(vertices_on_one_edge(_plane_vertices.at(i), _plane_vertices.at(j))) {
                cmp_point = project_between_two_points(p, cv::Point3f(_plane_vertices.at(i).pos), cv::Point3f(_plane_vertices.at(j).pos));
                cmp_distance = norm(cmp_point-p);
                if(cmp_distance<ref_distance) {
                    closest_point = cmp_point;
                    ref_distance = cmp_distance;
                }
            }
        }
    }
    return closest_point;
}


std::vector<intersection_point> CameraView::plane_vertices(uint plane_idx, view_volume_check_mode cm) {
    std::vector<intersection_point> plane_vertices;
    for(uint i=0; i<N_PLANE_VERTICES; i++) {
        for(uint j=0; j<3; j++) {
            if(vertices_relaxed.at(i).planes.at(j)==static_cast<int>(plane_idx)) {
                if(cm==relaxed)
                    plane_vertices.push_back(vertices_relaxed.at(i));
                else if(cm==strict)
                    plane_vertices.push_back(vertices_strict.at(i));
                else
                    plane_vertices.push_back(vertices.at(i));
            }
        }
    }
    return plane_vertices;
}


cv::Point3f CameraView::setpoint_in_cameraview(cv::Point3f pos_setpoint, view_volume_check_mode cm) {
    bool inview;
    std::array<bool, N_PLANES> violated_planes;
    std::tie(inview, violated_planes) = in_view(pos_setpoint, cm);
    if(!inview)
        pos_setpoint = project_into_camera_volume(pos_setpoint, cm, violated_planes);

    return pos_setpoint;
}


cv::Point3f CameraView::setpoint_in_cameraview(cv::Point3f pos_setpoint, cv::Point3f drone_pos, view_volume_check_mode cm) {
    bool inview;
    std::array<bool, N_PLANES> violated_planes;
    std::tie(inview, violated_planes) = in_view(pos_setpoint, cm);
    if(!inview)
        pos_setpoint = project_into_camera_volume(pos_setpoint, drone_pos, cm, violated_planes);

    return pos_setpoint;
}


void CameraView::cout_plane_violation(std::array<bool, N_PLANES> inview_violations, std::array<bool, N_PLANES> breaking_violations) {
    bool first = true;
    for (uint i=0; i<N_PLANES; i++) {
        if(inview_violations.at(i)==true) {
            if(!first)
                std::cout << ",";
            else
                std::cout << "Plane inview violation:";
            first = false;
            std::cout << " " << plane_names[i];
        }
    }
    if (!first)
        std::cout << std::endl;


    first = true;
    for (uint i=0; i<N_PLANES; i++) {
        if(breaking_violations.at(i)==true) {
            if(!first)
                std::cout << ",";
            else
                std::cout << "Plane breaking distance violation:";
            first = false;
            std::cout << " " << plane_names[i];
        }
    }
    if(!first)
        std::cout << std::endl;

}


void CameraView::adjacency_entry(uint val, uint p1, uint p2, uint p3) {
    adjacency_matrix.at(p1).at<int>(p2, p3) = val;
    adjacency_matrix.at(p1).at<int>(p3, p2) = val;
    adjacency_matrix.at(p2).at<int>(p1, p3) = val;
    adjacency_matrix.at(p2).at<int>(p3, p1) = val;
    adjacency_matrix.at(p3).at<int>(p1, p2) = val;
    adjacency_matrix.at(p3).at<int>(p2, p1) = val;
}


bool CameraView::vertices_on_one_edge(intersection_point p1, intersection_point p2) {
    uint common_planes = 0;
    for(uint i=0; i<3; i++) {
        for(uint j=0; j<3; j++) {
            if(p1.planes.at(i)==p2.planes.at(j))
                common_planes++;
        }
    }
    if(common_planes>=2)
        return true;
    return false;
}


std::ostream &operator<<(std::ostream &os, const CameraView &c) {
    os << "Cameraview>p0_front: " << c.plane_supports.at(CameraView::front_plane).t() << std::endl;
    os << "Cameraview>n_front: " << c.plane_normals.at(CameraView::front_plane).t() << std::endl;
    os << "Cameraview>p0_top: " << c.plane_supports.at(CameraView::top_plane).t() << std::endl;
    os << "Cameraview>n_top: " << c.plane_normals.at(CameraView::top_plane).t() << std::endl;
    os << "Cameraview>p0_left: " << c.plane_supports.at(CameraView::left_plane).t() << std::endl;
    os << "Cameraview>n_left: " << c.plane_normals.at(CameraView::left_plane).t() << std::endl;
    os << "Cameraview>p0_right: " << c.plane_supports.at(CameraView::right_plane).t() << std::endl;
    os << "Cameraview>n_right: " << c.plane_normals.at(CameraView::right_plane).t() << std::endl;
    os << "Cameraview>p0_bottom: " << c.plane_supports.at(CameraView::bottom_plane).t() << std::endl;
    os << "Cameraview>n_bottom: " << c.plane_normals.at(CameraView::bottom_plane).t() << std::endl;
    os << "Cameraview>p0_back: " << c.plane_supports.at(CameraView::back_plane).t() << std::endl;
    os << "Cameraview>n_back: " << c.plane_normals.at(CameraView::back_plane).t() << std::endl;
    os << "Cameraview>p0_camera: " << c.plane_supports.at(CameraView::camera_plane).t() << std::endl;
    os << "Cameraview>n_camera: " << c.plane_normals.at(CameraView::camera_plane).t() << std::endl;
    return os;
}