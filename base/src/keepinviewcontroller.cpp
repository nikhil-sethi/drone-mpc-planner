#include "keepinviewcontroller.h"
#include "common.h"


void KeepInViewController::init(CameraView* camview, xmls::DroneCalibration* dcalib) {
    _camview = camview;
    _dcalib = dcalib;
    drone_rotating_time = 6.f/pparams.fps; // Est. time to rotate the drone around 180 deg. see betaflight

    for (uint i=0; i<N_PLANES; i++) {
        d_vel_err_kiv.at(i).init(1.f/pparams.fps);
        d_pos_err_kiv.at(i).init(1.f/pparams.fps);
    }
}


cv::Point3f KeepInViewController::update(tracking::TrackData data_drone, float transmission_delay_duration, bool correction_requested) {
    active = false;
    std::array<float, N_PLANES> speed_error_normal_to_plane = {0};
    float effective_acceleration, remaining_breaking_distance_normal_to_plane, required_breaking_time, allowed_velocity_normal_to_plane;
    bool enough_braking_distance_left=true;
    std::array<bool, N_PLANES> violated_planes_brakedistance = {false};
    float current_drone_speed_normal_to_plane;
    for(uint i=0; i<N_PLANES; i++) {
        current_drone_speed_normal_to_plane = data_drone.state.vel.dot(-cv::Point3f(_camview->plane_normals.at(i)));
        remaining_breaking_distance_normal_to_plane = _camview->calc_shortest_distance_to_plane(data_drone.pos(), i, CameraView::relaxed)
                - current_drone_speed_normal_to_plane*(drone_rotating_time+transmission_delay_duration);
        if(remaining_breaking_distance_normal_to_plane<0)
            remaining_breaking_distance_normal_to_plane = 0;
        effective_acceleration = _dcalib->thrust/safety + cv::Point3f(0,-GRAVITY,0).dot(cv::Point3f(_camview->plane_normals.at(i)));
        required_breaking_time = sqrt(2*remaining_breaking_distance_normal_to_plane/effective_acceleration);
        if(required_breaking_time!=required_breaking_time) { // if required_breaking_time is nan
            // drone thrust including the safety is not strong enough to compensate gravity!
            // assume drone can at least accelerate slightly against gravity:
            effective_acceleration = 2.5;
            required_breaking_time = sqrt(2*remaining_breaking_distance_normal_to_plane/effective_acceleration);
        }
        allowed_velocity_normal_to_plane = required_breaking_time * effective_acceleration;
        speed_error_normal_to_plane.at(i) = current_drone_speed_normal_to_plane - allowed_velocity_normal_to_plane;
        if(speed_error_normal_to_plane.at(i)>0) {
            enough_braking_distance_left = false;
            violated_planes_brakedistance.at(i) = true;
        }
    }

    bool drone_in_boundaries;
    std::array<bool, N_PLANES> violated_planes_inview;
    std::tie(drone_in_boundaries, violated_planes_inview) = _camview->in_view(data_drone.pos(), CameraView::relaxed);

    for (uint i=0; i<N_PLANES; i++) {
        pos_err_kiv.at(i) = -_camview->calc_shortest_distance_to_plane(data_drone.pos(), i, CameraView::relaxed);
        d_pos_err_kiv.at(i).new_sample(pos_err_kiv.at(i));
        if(data_drone.vel_valid) { // ask sjoerd
            vel_err_kiv.at(i) = speed_error_normal_to_plane.at(i);
            d_vel_err_kiv.at(i).new_sample(vel_err_kiv.at(i));
        }
    }

    if((!drone_in_boundaries || !enough_braking_distance_left ) && correction_requested) {
        active = true;
        cv::Point3f correction_acceleration = kiv_acceleration(violated_planes_inview, violated_planes_brakedistance);
        // std::cout <<"KIV: " << correction_acceleration << std::endl;
        return correction_acceleration;
    }
    return cv::Point3f(0,0,0);
}

cv::Point3f KeepInViewController::kiv_acceleration(std::array<bool, N_PLANES> violated_planes_inview, std::array<bool, N_PLANES> violated_planes_brakedistance) {
#if CAMERA_VIEW_DEBUGGING
    _camview->cout_plane_violation(violated_planes_inview, violated_planes_brakedistance);
#endif
    cv::Point3f correction_acceleration(0,0,0);
    for(uint i=0; i<N_PLANES; i++) {
        if(i!=CameraView::bottom_plane) {
            if(violated_planes_inview.at(i)) {
                bool d_against_p_error = (sign(d_pos_err_kiv.at(i).current_output())!=sign(pos_err_kiv.at(i)));
                correction_acceleration += _camview->normal_vector(i)*(4.f*pos_err_kiv.at(i)
                                           + d_against_p_error * 0.1f*d_pos_err_kiv.at(i).current_output());
            }

            if(violated_planes_brakedistance.at(i))
                correction_acceleration += _camview->normal_vector(i)*(0.5f*vel_err_kiv.at(i) + 0.01f*d_vel_err_kiv.at(i).current_output());
        }
    }
    return correction_acceleration;
}

bool KeepInViewController::trajectory_in_view(std::vector<tracking::StateData> traj, CameraView::view_volume_check_mode c) {
    for (auto state : traj) {
        bool inview;
        std::tie(inview, ignore) = _camview->in_view(state.pos,c);
        if (!inview)
            return false;
    }
    return true;
}
