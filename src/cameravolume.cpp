#include "cameravolume.h"

#include "linalg.h"


void CameraVolume::init(cv::Point3f point_left_top, cv::Point3f point_right_top, cv::Point3f point_left_bottom, cv::Point3f point_right_bottom,
                        float b_depth, float b_height) {

    _n_front = get_plane_normal_vector (point_left_bottom, point_right_bottom);
    _p0_front = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_top = get_plane_normal_vector (point_left_top, point_right_top);
    _n_top *= -1; // let normal vector look inside the volume
    _p0_top = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_left = get_plane_normal_vector (point_left_bottom, point_left_top);
    _n_left *= -1; // let normal vector look inside the volume
    _p0_left = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_right = get_plane_normal_vector (point_right_bottom, point_right_top);
    _p0_right = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_bottom = (cv::Mat_<float>(3,1) << 0, 1, 0);
    p0_bottom_plane(b_height);

    if(b_depth<-10.f)
        b_depth = -10.f; //10 m is the max supported depth by the realsense

    _n_back = (cv::Mat_<float>(3,1) << 0, 0, 1);
    _p0_back = (cv::Mat_<float>(3,1) << 0, 0, b_depth);

#if true
    std::cout << "front> p0_front:" << _p0_front.t() << " n_front: " << _n_front.t() << std::endl;
    std::cout << "top> p0_top:" << _p0_top.t() << " n_top: " << _n_top.t() << std::endl;
    std::cout << "left> p0_left:" << _p0_left.t() << " n_left: " << _n_left.t() << std::endl;
    std::cout << "right> p0_right:" << _p0_right.t() << " n_right: " << _n_right.t() << std::endl;
    std::cout << "bottom> p0_bottom:" << _p0_bottom.t() << " n_bottom: " << _n_bottom.t() << std::endl;
    std::cout << "back> p0_back:" << _p0_back.t() << " n_back: " << _n_back.t() << std::endl;
#endif

    _n_bottom_hunt = _n_bottom;
    _n_top_hunt = _n_top;
    _n_front_hunt = (cv::Mat_<float>(3,1) << 0, 0, -1);
    _n_back_hunt = _n_back;

    point_left_bottom.x -= static_cast<float>(margin_left);
    point_left_top.x -= static_cast<float>(margin_left);
    point_right_bottom.x += static_cast<float>(margin_right);
    point_right_top.x += static_cast<float>(margin_right);

    _n_left_hunt = get_plane_normal_vector (point_left_bottom, point_left_top);
    _n_left_hunt *= -1;
    _n_right_hunt = get_plane_normal_vector (point_right_bottom, point_right_top);

    _p0_top_hunt = _p0_top + margin_top * _n_top_hunt;
    _p0_front_hunt =  margin_front * _n_front_hunt;
    _p0_back_hunt = _p0_back + margin_back * _n_back_hunt;
    _p0_left_hunt = _p0_left;
    _p0_right_hunt = _p0_right;

    calc_corner_points_hunt(_p0_front_hunt, _n_front_hunt, _p0_back_hunt, _n_back_hunt, _p0_top_hunt, _n_top_hunt, _p0_bottom_hunt, _n_bottom_hunt, _p0_left_hunt, _n_left_hunt, _p0_right_hunt, _n_right_hunt);
    calc_corner_points(_p0_front, _n_front, _p0_back, _n_back, _p0_top, _n_top, _p0_bottom, _n_bottom, _p0_left, _n_left, _p0_right, _n_right);
}


void CameraVolume::p0_bottom_plane(float b_height){
    _p0_bottom = (cv::Mat_<float>(3,1) << 0, b_height, 0);
    _p0_bottom_hunt = _p0_bottom + margin_bottom * _n_bottom;
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
    bool front_check = on_normal_side (_p0_front+hysteresis_margin*_n_front, _n_front, cv::Mat(p));
    bool left_check = on_normal_side (_p0_left-hysteresis_margin*_n_left, _n_left, cv::Mat(p));
    bool right_check = on_normal_side (_p0_right+hysteresis_margin*_n_right, _n_right, cv::Mat(p));
    bool bottom_check = on_normal_side (_p0_bottom+hysteresis_margin*_n_bottom, _n_bottom, cv::Mat(p));
    bool back_check = on_normal_side (_p0_back+hysteresis_margin*_n_back, _n_back, cv::Mat(p));

    std::array<bool, N_PLANES> violated_planes = {{false}}; // TODO: CHeck if this is working [Ludwig]

    bool inview = true;

    // Attention check the negative case!
    if(!front_check) {
        inview = false;
        violated_planes.at(plane_index::front_plane) = true;
    }
    if(!left_check){
        inview = false;    
        violated_planes.at(plane_index::left_plane) = true;
    }
    if(!right_check) {
        inview = false;    
        violated_planes.at(plane_index::right_plane) = true;
    }
    if(!bottom_check) {
        inview = false;    
        violated_planes.at(plane_index::bottom_plane) = true;
    }
    if(!back_check) {
        inview = false;    
        violated_planes.at(plane_index::back_plane) = true;
    }
    
    return std::tuple(inview, violated_planes);
}

CameraVolume::hunt_check_result CameraVolume::in_hunt_area(cv::Point3f d[[maybe_unused]], cv::Point3f m) {
    bool front_check = on_normal_side (_p0_front_hunt, _n_front_hunt, cv::Mat(m));
    bool left_check = on_normal_side (_p0_left_hunt, _n_left_hunt, cv::Mat(m));
    bool right_check = on_normal_side (_p0_right_hunt, _n_right_hunt, cv::Mat(m));
    bool bottom_check = on_normal_side (_p0_bottom_hunt, _n_bottom_hunt, cv::Mat(m));
    bool back_check = on_normal_side (_p0_back_hunt, _n_back_hunt, cv::Mat(m));
    bool top_check = on_normal_side (_p0_top_hunt, _n_top_hunt, cv::Mat(m));

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

    // Check the back:
    cv::Mat plane = cv::Mat::zeros(cv::Size(2,3), CV_32F);
    _p0_back.copyTo (plane.col(0));
    _n_back.copyTo (plane.col(1));
    distances_to_planes.at(back_plane) = calc_distance_to_plane (pMat, plane);
    
    // Check the bottom:
    _p0_bottom.copyTo (plane.col(0));
    _n_bottom.copyTo (plane.col(1));
    distances_to_planes.at(bottom_plane) = calc_distance_to_plane (pMat, plane);

    // Check the front:
    _p0_front.copyTo (plane.col(0));
    _n_front.copyTo (plane.col(1));
    distances_to_planes.at(front_plane) = calc_distance_to_plane (pMat, plane);

    // Check the top:
    _p0_top.copyTo (plane.col(0)); 
    _n_top.copyTo (plane.col(1));
    distances_to_planes.at(top_plane) = calc_distance_to_plane (pMat, plane);

    // Check the left:
    _p0_left.copyTo (plane.col(0));
    _n_left.copyTo (plane.col(1));
    distances_to_planes.at(left_plane) = calc_distance_to_plane (pMat, plane);

    // Check the right:
    _p0_right.copyTo (plane.col(0));
    _n_right.copyTo (plane.col(1));
    distances_to_planes.at(right_plane) = calc_distance_to_plane (pMat, plane);

    return distances_to_planes;
}

cv::Point3f CameraVolume::normal_vector(plane_index plane_idx) {
    if(plane_idx==plane_index::back_plane)
        return cv::Point3f(_n_back);
    if(plane_idx==plane_index::bottom_plane)
        return cv::Point3f(_n_bottom);
    if(plane_idx==plane_index::front_plane)
        return cv::Point3f(_n_front);
    if(plane_idx==plane_index::top_plane)
        return cv::Point3f(_n_top);
    if(plane_idx==plane_index::left_plane)
        return cv::Point3f(_n_left);
    if(plane_idx==plane_index::right_plane)
        return cv::Point3f(_n_right);

    else
        return cv::Point3f(0,0,0); 
}