#pragma once
#include "common.h"
#include "itemtracker.h"
#include "insecttracker.h"

namespace tracking {
static const char *drone_tracking_state_names[] = {
    "dts_detecting_takeoff",
    "dts_detecting_takeoff_fail",
    "dts_detecting",
    "dts_tracking",
    "dts_detect_yaw",
    "dts_landing_init",
    "dts_landing",
    "dts_landed"
};
class DroneTracker : public ItemTracker {

private:
    enum drone_tracking_states {
        dts_detecting_takeoff,
        dts_detecting_takeoff_failure,
        dts_detecting,
        dts_tracking,
        dts_detect_yaw,
        dts_landing_init,
        dts_landing,
        dts_landed
    };
    class DroneTrackerCalibrationData: public xmls::Serializable
    {
    public:

        xmls::xFloat Drone_takeoff_image_location_x;
        xmls::xFloat Drone_takeoff_image_location_y;

        xmls::xFloat Drone_takeoff_world_location_x;
        xmls::xFloat Drone_takeoff_world_location_y;
        xmls::xFloat Drone_takeoff_world_location_z;

        DroneTrackerCalibrationData():
            Drone_takeoff_image_location_x(0),
            Drone_takeoff_image_location_y(0),
            Drone_takeoff_world_location_x(0),
            Drone_takeoff_world_location_y(0),
            Drone_takeoff_world_location_z(0)
        {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("DroneTrackerCalibrationData");

            // Set class version
            setVersion("1.1");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("Drone_takeoff_image_location_x", &Drone_takeoff_image_location_x);
            Register("Drone_takeoff_image_location_y", &Drone_takeoff_image_location_y);

            Register("Drone_takeoff_world_location_x", &Drone_takeoff_world_location_x);
            Register("Drone_takeoff_world_location_y", &Drone_takeoff_world_location_y);
            Register("Drone_takeoff_world_location_z", &Drone_takeoff_world_location_z);
        };
    };

    drone_tracking_states _drone_tracking_status = dts_detecting_takeoff;
    drone_tracking_states _drone_tracking_status_before_tracking_loss = dts_detecting_takeoff;
    cv::Point3f _target = {0};
    bool _manual_flight_mode = false;
    bool _hover_mode = false;

    double start_take_off_time = 0;
    double spinup_detect_time = 0;
    double _time = 0;
    double time_yaw_not_ok = -1;

    double takeoff_location_ignore_timeout = 1;
    double landing_ignore_timeout = 5;
    int spinup_detected = 0;
    bool liftoff_detected = false;
    uint16_t take_off_frame_cnt = 0;

    bool enable_motion_shadow_delete = false;
    float motion_shadow_im_size;
    cv::Point2f motion_shadow_im_location;
    float motion_shadow_disparity;

    float _pad_im_size;
    cv::Point2f _pad_im_location;
    float _pad_disparity;
    float _drone_on_pad_im_size;
    bool _pad_location_valid = false;
    cv::Point3f _pad_world_location;

    double deviation_angle;
    bool yaw_deviation_vec_length_OK = false;
    const float min_deviate_vec_length = 0.05;
    const double min_yaw_ok_time = 1.5;

    bool enable_viz_motion = false;
    bool drone_on_pad = true;

    cv::Point3f *_commanded_acceleration;
    cv::Point3f takeoff_prediction_pos;
    cv::Point3f takeoff_prediction_vel;
    cv::Point2f _takeoff_direction_predicted;

    cv::Mat _template_drone;
    cv::Size2f _drone_im_size;

    void calc_takeoff_prediction(double time);
    void reset_takeoff_im_prediction_if_direction_bad(cv::Point2f takeoff_direction_measured, float measured_versus_predicted_angle_diff);
    void handle_brightness_change(double time);
    void delete_motion_shadow(cv::Point2f im_location, float im_size, float disparity);
    void delete_motion_shadow_run();
    bool detect_lift_off();
    bool detect_takeoff();
    void detect_deviation_yaw_angle();
    void update_drone_prediction(double time);
    void clean_ignore_blobs(double time);
    cv::Point2f template_matching_tracking(cv::Mat img_l, cv::Point2f previous_im_pos, double time);

public:
    cv::Mat diff_viz;
    cv::Point2f template_matching_tracking_pos;

    bool init(VisionData *_visdat, int motion_thresh, int16_t viz_id);
    void init_flight(std::ofstream *logger, double time);
    void update(double time);
    void calc_world_item(BlobProps *pbs, double time);
    bool check_ignore_blobs(BlobProps *pbs);
    void update_target(cv::Point3f target) {
        if (target.x == target.x)
            _target = target;
    }
    void land() {_drone_tracking_status = dts_landing_init;}
    void detect_yaw(double time) {
        time_yaw_not_ok = time;
        _drone_tracking_status = dts_detect_yaw;
    }
    bool check_yaw(double time);
    bool delete_me() {return false;}

    void manual_flight_mode(bool value) { _manual_flight_mode = value; }
    void hover_mode(bool value);

    void delete_landing_motion(float duration);

    tracker_type type() { return tt_drone;}
    float score(BlobProps *blob);
    double time_since_take_off() {return start_take_off_time - _time;}
    cv::Point3f pad_location(bool landing_hack);
    cv::Point3f pad_location() { return _pad_world_location; };
    cv::Point2f takeoff_direction_predicted() { return _takeoff_direction_predicted; };
    void set_pad_location_from_blink(cv::Point3f);
    void set_pad_location(cv::Point3f pad_world);
    cv::Point3f pad_location_from_blink(cv::Point3f blink_location) {return blink_location + cv::Point3f(0, 0, -dparams.radius);}
    cv::Point2f pad_im_location() { return _pad_im_location; }
    float pad_im_size() { return _pad_im_size; }
    float pad_disparity() { return _pad_disparity; }
    bool pad_location_valid() {return _pad_location_valid;}
    bool take_off_detection_failed() { return _drone_tracking_status == dts_detecting_takeoff_failure;}
    bool taking_off() { return  _drone_tracking_status == dts_detecting_takeoff;}
    bool landing() { return _drone_tracking_status == dts_landing_init || _drone_tracking_status == dts_landing;}
    bool lost() {return _n_frames_lost > static_cast<int>(pparams.fps * 2);}
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    bool drone_on_landing_pad() {return drone_on_pad;}
    void drone_on_landing_pad(bool value) {drone_on_pad = value;}
    bool bowl_nudge_needed(cv::Point3f setpoint_pos) {return normf(last_track_data().pos() - setpoint_pos) < min_deviate_vec_length;}
    void commanded_acceleration(cv::Point3f *commanded_acceleration) {_commanded_acceleration = commanded_acceleration;}
    cv::Point2f drone_im_size() {return _drone_im_size;}
    cv::Rect return_valid_crop_region(cv::Rect crop_region) {
        crop_region.x = std::clamp(crop_region.x, 0, IMG_W - 1 - crop_region.width);
        crop_region.y = std::clamp(crop_region.y, 0, IMG_H - 1 - crop_region.height);
        return crop_region;
    }

};

}
