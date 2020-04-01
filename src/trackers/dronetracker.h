#pragma once
#include "common.h"
#include "itemtracker.h"
#include "insecttracker.h"
#include "tinyxml/XMLSerialization.h"

namespace tracking {
static const char* drone_tracking_state_names[] = { "dts_init",
                                                    "dts_inactive",
                                                    "dts_detecting_takeoff_init",
                                                    "dts_detecting_takeoff",
                                                    "dts_detecting",
                                                    "dts_detected",
                                                    "dts_detect_yaw",
                                                    "dts_landing_init",
                                                    "dts_landing"
                                                  };
class DroneTracker : public ItemTracker {
public: tracker_type type() { return tt_drone;}

private:
    double start_take_off_time = 0;
    double spinup_detect_time = 0;
    double current_time = 0;

    double startup_location_ignore_timeout = 1; // TODO: make this dependent on the motion_update_iterator_max
    double landing_ignore_timeout = 5; // TODO: make this dependent on the motion_update_iterator_max
    double taking_off_ignore_timeout = 0.1; // TODO: make this dependent on the motion_update_iterator_max

    float yaw;
    const float landing_yaw_criteria = 0.035;

    cv::Point3f _target = {0};

    bool spinup_detected = false;
    bool liftoff_detected = false;
    bool _take_off_detection_failed = false;
    uint16_t take_off_frame_cnt = 0;

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
    drone_tracking_states _drone_tracking_status = dts_init;

    enum side {
        leftside,
        rightside
    };

    cv::Point2f _drone_blink_im_location;
    float _drone_blink_im_disparity;
    float _drone_blink_im_size = 5;
    cv::Point3f _drone_blink_world_location;
    bool _landing_pad_location_set = false;
    cv::Point3f _landing_pad_world;

    bool enable_viz_diff = false;



    class DroneTrackerCalibrationData: public xmls::Serializable
    {
    public:

        xmls::xFloat Drone_startup_image_location_x;
        xmls::xFloat Drone_startup_image_location_y;

        xmls::xFloat Drone_startup_world_location_x;
        xmls::xFloat Drone_startup_world_location_y;
        xmls::xFloat Drone_startup_world_location_z;

        DroneTrackerCalibrationData():
            Drone_startup_image_location_x(0),
            Drone_startup_image_location_y(0),
            Drone_startup_world_location_x(0),
            Drone_startup_world_location_y(0),
            Drone_startup_world_location_z(0)
        {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("DroneTrackerCalibrationData");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("Drone_startup_image_location_x", &Drone_startup_image_location_x);
            Register("Drone_startup_image_location_y", &Drone_startup_image_location_y);

            Register("Drone_startup_world_location_x", &Drone_startup_world_location_x);
            Register("Drone_startup_world_location_y", &Drone_startup_world_location_y);
            Register("Drone_startup_world_location_z", &Drone_startup_world_location_z);
        };
    };
    void serialize_calib();
    void deserialize_calib(string file);

    void clean_ignore_blobs(double time);

    void calc_takeoff_prediction();
    void delete_takeoff_fake_motion(int frames);
    bool detect_lift_off();
    bool detect_takeoff();

    cv::Mat get_big_blob(cv::Mat Mask, int connectivity);
    cv::Mat extract_mask_column(cv::Mat mask_big, float range_left, float range_right, float side_percentage, enum side side_);
    cv::Mat split_mask_half(cv::Mat mask_big, enum side);
    float yaw_from_splitted_mask(cv::Mat left, cv::Mat right);
    float calc_yaw(BlobProps * pbs, bool inspect_blob);

    xmls::LandingParameters landing_parameter;

public:
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    cv::Point2f drone_startup_im_location() { return _drone_blink_im_location; }
    float drone_startup_im_size() { return _drone_blink_im_size; }
    float drone_startup_im_disparity() { return _drone_blink_im_disparity; }
    cv::Point3f drone_startup_location() {return _drone_blink_world_location + cv::Point3f(0,0,-dparams.radius);}
    cv::Point3f drone_landing_location() {
        if(landing_parameter.initialized)
            return cv::Point3f(landing_parameter.x, landing_parameter.y, landing_parameter.z);
        else
            return _landing_pad_world;
    }

    bool take_off_detection_failed() { return _take_off_detection_failed;}

    bool taking_off() { return _drone_tracking_status == dts_detecting_takeoff_init || _drone_tracking_status == dts_detecting_takeoff;}
    bool landing() { return _drone_tracking_status == dts_landing_init || _drone_tracking_status == dts_landing;}
    bool inactive() { return _drone_tracking_status == dts_inactive;}
    bool correct_yaw() { return _drone_tracking_status == dts_detect_yaw;}

    bool _manual_flight_mode = false;
    cv::Mat diff_viz;

    bool init(std::ofstream *logger, VisionData *_visdat, int16_t viz_id);
    void track(double time, bool drone_is_active);

    void land() {_drone_tracking_status = dts_landing_init;}
    void detect_yaw() {_drone_tracking_status = dts_detect_yaw;}
    bool check_yaw() { return ((fabs(yaw)<landing_yaw_criteria) && (fabs(yaw)!=0));}
    bool check_smooth_yaw() { return (fabs(yaw_smoother.latest())<landing_yaw_criteria);}

    void set_drone_landing_location(cv::Point2f im, float drone_im_disparity,float drone_im_size, cv::Point3f world) {
        _drone_blink_im_location = im;
        _drone_blink_im_size = drone_im_size;
        _drone_blink_im_disparity = drone_im_disparity;
        _drone_blink_world_location = world;
        if (!_landing_pad_location_set) { // for now, assume the first time set is the actual landing location.
            _landing_pad_world = drone_startup_location();
            _landing_pad_location_set = true;
        }
        _target = drone_startup_location();
    }
    void delete_landing_motion(float duration);
    void calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs);

    double time_since_take_off() {return start_take_off_time - current_time;}

    void update_drone_target(cv::Point3f target) { _target = target; }

    bool delete_me() {return false;}

    float hover_throttle_estimation;
};

}
