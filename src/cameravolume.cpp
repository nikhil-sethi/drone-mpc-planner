#include "cameravolume.h"

#include "linalg.h"


void CameraVolume::init(cv::Point3f point_left_top, cv::Point3f point_right_top, cv::Point3f point_left_bottom, cv::Point3f point_right_bottom,
                        float b_depth, float b_height) {

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

    plane_normals.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, 1, 0);
    p0_bottom_plane(b_height);

    if(b_depth<-10.f)
        b_depth = -10.f; //10 m is the max supported depth by the realsense

    plane_normals.at(back_plane) = (cv::Mat_<float>(3,1) << 0, 0, 1);
    plane_supports.at(back_plane) = (cv::Mat_<float>(3,1) << 0, 0, b_depth);

#if true
    std::cout << "front> p0_front:" << plane_supports.at(front_plane).t() << " n_front: " << plane_normals.at(front_plane).t() << std::endl;
    std::cout << "top> p0_top:" << plane_supports.at(front_plane).t() << " n_top: " << plane_normals.at(front_plane).t() << std::endl;
    std::cout << "left> p0_left:" << plane_supports.at(left_plane).t() << " n_left: " << plane_normals.at(left_plane).t() << std::endl;
    std::cout << "right> p0_right:" << plane_supports.at(right_plane).t() << " n_right: " << plane_normals.at(right_plane).t() << std::endl;
    std::cout << "bottom> p0_bottom:" << plane_supports.at(bottom_plane).t() << " n_bottom: " << plane_normals.at(bottom_plane).t() << std::endl;
    std::cout << "back> p0_back:" << plane_supports.at(back_plane).t() << " n_back: " << plane_normals.at(back_plane).t() << std::endl;
#endif

    _n_bottom_hunt = plane_normals.at(bottom_plane);
    _n_top_hunt = plane_normals.at(top_plane);
    _n_front_hunt = (cv::Mat_<float>(3,1) << 0, 0, -1);
    _n_back_hunt = plane_normals.at(back_plane);

    point_left_bottom.x -= static_cast<float>(margin_left);
    point_left_top.x -= static_cast<float>(margin_left);
    point_right_bottom.x += static_cast<float>(margin_right);
    point_right_top.x += static_cast<float>(margin_right);

    _n_left_hunt = get_plane_normal_vector (point_left_bottom, point_left_top);
    _n_left_hunt *= -1;
    _n_right_hunt = get_plane_normal_vector (point_right_bottom, point_right_top);

    _p0_top_hunt = plane_supports.at(top_plane) + margin_top * _n_top_hunt;
    _p0_front_hunt =  margin_front * _n_front_hunt;
    _p0_back_hunt = plane_supports.at(back_plane) + margin_back * _n_back_hunt;
    _p0_left_hunt = plane_supports.at(left_plane);
    _p0_right_hunt = plane_supports.at(right_plane);

    calc_corner_points_hunt(_p0_front_hunt, _n_front_hunt, _p0_back_hunt, _n_back_hunt, _p0_top_hunt, _n_top_hunt, _p0_bottom_hunt, _n_bottom_hunt, _p0_left_hunt, _n_left_hunt, _p0_right_hunt, _n_right_hunt);
    calc_corner_points(plane_supports.at(front_plane), plane_normals.at(front_plane), plane_supports.at(back_plane), plane_normals.at(back_plane),
                       plane_supports.at(top_plane), plane_normals.at(top_plane), plane_supports.at(bottom_plane), plane_normals.at(bottom_plane),
                       plane_supports.at(left_plane), plane_normals.at(left_plane), plane_supports.at(right_plane), plane_normals.at(right_plane));
}


void CameraVolume::p0_bottom_plane(float b_height){
    plane_supports.at(bottom_plane) = (cv::Mat_<float>(3,1) << 0, b_height, 0);
    _p0_bottom_hunt = plane_supports.at(bottom_plane) + margin_bottom * plane_normals.at(bottom_plane);
}

void CameraVolume::calc_corner_points(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                      cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                      cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right)
{
    _bottom_left_back = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    _bottom_right_back = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    _top_left_back = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    _top_right_back = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    _bottom_left_front = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    _bottom_right_front = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    _top_left_front = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    _top_right_front = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

void CameraVolume::calc_corner_points_hunt(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
        cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
        cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right)
{
    _bottom_left_back_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    _bottom_right_back_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    _top_left_back_hunt = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    _top_right_back_hunt = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    _bottom_left_front_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    _bottom_right_front_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    _top_left_front_hunt = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    _top_right_front_hunt = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraVolume::in_view(cv::Point3f p, view_volume_check_mode c) {
    if (c == relaxed)
        return in_view(p,0.3f);
    else
        return in_view(p,0.6f);
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraVolume::in_view(cv::Point3f p,float hysteresis_margin) {
    std::array<bool, N_PLANES> violated_planes = {{false}}; 
    bool inview = true;

    // Attention check the negative case!
    for(uint i=1; i<N_PLANES; i++){ // Start at 1 to not check the top-plane which is plane0
        if(!on_normal_side(plane_supports.at(i) + hysteresis_margin*plane_normals.at(i), plane_normals.at(i), cv::Mat(p))) {
            inview = false;
            violated_planes.at(i) = true;
        }
    }
    return std::tuple(inview, violated_planes);
}

CameraVolume::hunt_check_result CameraVolume::in_hunt_area(cv::Point3f d[[maybe_unused]], cv::Point3f m) {
    bool front_check = on_normal_side (plane_supports.at(front_plane), plane_normals.at(front_plane), cv::Mat(m));
    bool left_check = on_normal_side (plane_supports.at(left_plane), plane_normals.at(left_plane), cv::Mat(m));
    bool right_check = on_normal_side (plane_supports.at(right_plane), plane_normals.at(right_plane), cv::Mat(m));
    bool bottom_check = on_normal_side (plane_supports.at(bottom_plane), plane_normals.at(bottom_plane), cv::Mat(m));
    bool back_check = on_normal_side (plane_supports.at(back_plane), plane_normals.at(top_plane), cv::Mat(m));
    bool top_check = on_normal_side (plane_supports.at(top_plane), plane_normals.at(top_plane), cv::Mat(m));

    if( !top_check)
        return HuntVolume_To_High;
    if( !bottom_check)
        return HuntVolume_To_Low;
    if( !front_check)
        return HuntVolume_Outside_Cone;
    if( !back_check)
        return HuntVolume_Outside_Cone;
    if( !left_check)
        return HuntVolume_Outside_Cone;
    if( !right_check)
        return HuntVolume_Outside_Cone;

    return HuntVolume_OK;
}

std::tuple<bool, std::array<bool, N_PLANES>> CameraVolume::check_distance_to_borders(track_data data_drone, float req_breaking_distance) {
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

std::array<float, N_PLANES> CameraVolume::calc_distance_to_borders(std::vector<cv::Point3f> p) {
    cv::Mat pMat = (cv::Mat_<float_t>(3,2) << p[0].x, p[1].x, p[0].y, p[1].y, p[0].z, p[1].z);
    std::array<float, N_PLANES> distances_to_planes;

    cv::Mat plane = cv::Mat::zeros(cv::Size(2,3), CV_32F);
    for(uint i=0; i<N_PLANES; i++) {
        plane_supports.at(i).copyTo(plane.col(0));
        plane_normals.at(i).copyTo(plane.col(1));
        distances_to_planes.at(i) = calc_distance_to_plane(pMat, plane);
    }

    return distances_to_planes;
}
