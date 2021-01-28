#pragma once
#include "common.h"
#include "itemtracker.h"
#include "insecttracker.h"

namespace tracking {
static const char* drone_tracking_state_names[] = { "dts_init",
                                                    "dts_inactive",
                                                    "dts_detecting_takeoff_init",
                                                    "dts_detecting_takeoff",
                                                    "dts_detecting",
                                                    "dts_tracking",
                                                    "dts_detect_yaw",
                                                    "dts_landing_init",
                                                    "dts_landing"
                                                  };
class DroneTracker : public ItemTracker {

private:
    enum drone_tracking_states {
        dts_init = 0,
        dts_inactive,
        dts_detecting_takeoff_init,
        dts_detecting_takeoff,
        dts_detecting,
        dts_tracking,
        dts_detect_yaw,
        dts_landing_init,
        dts_landing
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

    drone_tracking_states _drone_tracking_status = dts_init;
    cv::Point3f _target = {0};
    bool _manual_flight_mode = false;
    bool _hover_mode = false;

    double start_take_off_time = 0;
    double spinup_detect_time = 0;
    double current_time = 0;
    double time_yaw_not_ok = -1;

    double takeoff_location_ignore_timeout = 1;
    double landing_ignore_timeout = 5;
    int spinup_detected = 0;
    bool liftoff_detected = false;
    bool _take_off_detection_failed = false;
    uint16_t take_off_frame_cnt = 0;
    bool enable_takeoff_motion_delete = false;

    cv::Point2f _blink_im_location;
    float _blink_im_disparity;
    float _blink_im_size = 5;
    float _takeoff_im_size;
    cv::Point2f _takeoff_im_location;
    cv::Point3f _blink_world_location;
    bool _takeoff_location_valid = false;
    cv::Point3f _landing_world_location;
    xmls::LandingParameters landing_parameter;

    double deviation_angle;
    bool yaw_deviation_vec_length_OK = false;
    const double min_deviate_vec_length = 0.05;
    const double min_yaw_ok_time = 1.5;

    bool enable_viz_motion = false;
    bool drone_on_pad = true;

    void calc_takeoff_prediction();
    void delete_takeoff_fake_motion();
    bool detect_lift_off();
    bool detect_takeoff();
    void detect_deviation_yaw_angle();
    void update_drone_prediction(double time);
    void clean_ignore_blobs(double time);

public:
    cv::Mat diff_viz;
    const float min_yaw_deviation = 0.5f;

    bool init(std::ofstream *logger, VisionData *_visdat, int motion_thresh, int16_t viz_id);
    void update(double time, bool drone_is_active);
    void calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs);
    void update_target(cv::Point3f target) { _target = target; }
    void land() {_drone_tracking_status = dts_landing_init;}
    void detect_yaw(double time) {
        time_yaw_not_ok = time;
        _drone_tracking_status = dts_detect_yaw;
    }
    bool check_yaw(double time);
    bool delete_me() {return false;}

    void manual_flight_mode(bool value) { _manual_flight_mode =value; }
    void hover_mode(bool value);
    void set_landing_location(cv::Point2f im, float im_disparity,float im_size, cv::Point3f world);
    void delete_landing_motion(float duration);

    tracker_type type() { return tt_drone;}
    float score(BlobProps * blob);
    double time_since_take_off() {return start_take_off_time - current_time;}
    cv::Point2f blnk_im_location() { return _blink_im_location; }
    float blnk_im_size() { return _blink_im_size; }
    float blink_im_disparity() { return _blink_im_disparity; }
    float takeoff_im_size() { return _takeoff_im_size; }
    cv::Point2f takeoff_im_location() { return _takeoff_im_location;}
    bool takeoff_location_valid() {return _takeoff_location_valid;}
    cv::Point3f takeoff_location() {return _blink_world_location + cv::Point3f(0,0,-dparams.radius);}
    cv::Point3f landing_location(bool landing_hack);
    bool take_off_detection_failed() { return _take_off_detection_failed;}
    bool taking_off() { return _drone_tracking_status == dts_detecting_takeoff_init || _drone_tracking_status == dts_detecting_takeoff;}
    bool landing() { return _drone_tracking_status == dts_landing_init || _drone_tracking_status == dts_landing;}
    bool inactive() { return _drone_tracking_status == dts_inactive;}
    bool lost() {return _n_frames_lost > static_cast<int>(pparams.fps*2);}
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    bool drone_on_landing_pad() {return drone_on_pad;}

};

}
