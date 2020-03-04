#include "cameraview.h"

#include "linalg.h"


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
    plane_supports.at(camera_plane) = plane_normals.at(camera_plane)*0.85f; //If the drones gets too close to the camera (~ 0.5 m) the drone position is not calculated anymore.

    plane_normals.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, 1, 0);
    p0_bottom_plane(b_height);

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

    calc_corner_points_hunt(plane_supports_hunt.at(front_plane), plane_normals_hunt.at(front_plane), plane_supports_hunt.at(back_plane), plane_normals_hunt.at(back_plane),
                            plane_supports_hunt.at(top_plane), plane_normals_hunt.at(top_plane), plane_supports_hunt.at(bottom_plane), plane_normals_hunt.at(bottom_plane),
                             plane_supports_hunt.at(left_plane), plane_normals_hunt.at(left_plane), plane_supports_hunt.at(right_plane), plane_normals_hunt.at(right_plane));
    calc_corner_points(plane_supports.at(front_plane), plane_normals.at(front_plane), plane_supports.at(back_plane), plane_normals.at(back_plane),
                       plane_supports.at(top_plane), plane_normals.at(top_plane), plane_supports.at(bottom_plane), plane_normals.at(bottom_plane),
                       plane_supports.at(left_plane), plane_normals.at(left_plane), plane_supports.at(right_plane), plane_normals.at(right_plane));
}


void CameraView::p0_bottom_plane(float b_height){
    plane_supports.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, b_height, 0);
    plane_supports_hunt.at(bottom_plane) = plane_supports.at(bottom_plane) + margin_bottom * plane_normals.at(bottom_plane);
#if CAMERA_VIEW_DEBUGGING
    std::cout << *this <<std::endl;
#endif
}

void CameraView::calc_corner_points(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                      cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                      cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right) {
                                          
    corner_points.at(bottom_left_back) = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    corner_points.at(bottom_right_back) = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    corner_points.at(top_left_back) = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    corner_points.at(top_right_back) = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    corner_points.at(bottom_left_front) = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    corner_points.at(bottom_right_front) = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    corner_points.at(top_left_front) = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    corner_points.at(top_right_front) = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

void CameraView::calc_corner_points_hunt(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
        cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
        cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right) {
    corner_points_hunt.at(bottom_left_back) = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    corner_points_hunt.at(bottom_right_back) = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    corner_points_hunt.at(top_left_back) = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    corner_points_hunt.at(top_right_back) = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    corner_points_hunt.at(bottom_left_front) = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    corner_points_hunt.at(bottom_right_front) = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    corner_points_hunt.at(top_left_front) = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    corner_points_hunt.at(top_right_front) = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraView::in_view(cv::Point3f p, view_volume_check_mode c) {
    if (c == relaxed)
        return in_view(p,relaxed_safety_margin);
    else
        return in_view(p,strict_safetty_margin);
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraView::in_view(cv::Point3f p,float hysteresis_margin) {
    std::array<bool, N_PLANES> violated_planes = {{false}}; 
    bool inview = true;

    // Attention check the negative case!
    for(uint i=0; i<N_PLANES; i++){ 
        if(!on_normal_side(plane_supports.at(i) + hysteresis_margin*plane_normals.at(i), plane_normals.at(i), cv::Mat(p))) {
            inview = false;
            violated_planes.at(i) = true;
        }
    }
    return std::tuple(inview, violated_planes);
}

CameraView::hunt_check_result CameraView::in_hunt_area(cv::Point3f d[[maybe_unused]], cv::Point3f m) {
    bool inhuntarea = true;
    std::array <bool, N_PLANES> violated_planes = {{false}};
    for(uint i=0; i<N_PLANES; i++) {
       if(!on_normal_side(plane_supports.at(i), plane_normals.at(i), cv::Mat(m))) {
           inhuntarea = false;
           violated_planes.at(i) = false;
       }
    }
    if(inhuntarea)
        return HuntVolume_OK;
    if(violated_planes.at(top_plane))
        return HuntVolume_To_High;
    else if(violated_planes.at(bottom_plane))
        return HuntVolume_To_Low;
    else
        return HuntVolume_Outside_Huntarea;  
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraView::check_distance_to_borders(track_data data_drone, float req_breaking_distance) {
    std::vector<cv::Point3f> p{data_drone.pos (), data_drone.vel ()};
    std::array<float, N_PLANES> distances_to_planes = calc_distance_to_borders (p);
    std::array<bool, N_PLANES> violated_planes = {{false}};
    bool planes_not_violated = true;

    for(int i=0; i<N_PLANES; i++){
        if(distances_to_planes.at(i)>= 0 && distances_to_planes.at(i)<req_breaking_distance) {
            violated_planes.at(i) = true;
            planes_not_violated = false;
        }
    }

    return std::tuple(planes_not_violated, violated_planes);
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

float CameraView::calc_shortest_distance_to_border(cv::Point3f drone_pos, uint plane_idx, view_volume_check_mode cm) {
    float safety_margin = relaxed_safety_margin;
    if(cm == strict)
        safety_margin = strict_safetty_margin;

    cv::Point3f shifted_support_vector = support_vector(plane_idx) + safety_margin*normal_vector(plane_idx);
    return shortest_distance_to_plane(drone_pos, shifted_support_vector, normal_vector(plane_idx));
}

cv::Point3f CameraView::project_into_camera_volume(cv::Point3f pos_setpoint, std::array<bool, N_PLANES> violated_planes) {
    for (uint i=0; i<N_PLANES; i++) {
        if(violated_planes.at(i)==true)
            pos_setpoint = pos_setpoint - calc_shortest_distance_to_border(pos_setpoint, i, relaxed)*normal_vector(i);
    }
    return pos_setpoint;
    
}

cv::Point3f CameraView::setpoint_in_cameraview(cv::Point3f pos_setpoint) {
    bool inview;
    std::array<bool, N_PLANES> violated_planes;
    std::tie(inview, violated_planes) = in_view(pos_setpoint, relaxed);
    if(!inview)
        pos_setpoint = project_into_camera_volume(pos_setpoint, violated_planes);

    return pos_setpoint;
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