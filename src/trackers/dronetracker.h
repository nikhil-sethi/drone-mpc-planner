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
    double time_yaw_not_ok;
    const double min_yaw_ok_time = 1.5;

    double takeoff_location_ignore_timeout = 1; // TODO: make this dependent on the motion_update_iterator_max
    double landing_ignore_timeout = 5; // TODO: make this dependent on the motion_update_iterator_max
    double taking_off_ignore_timeout = 0.1; // TODO: make this dependent on the motion_update_iterator_max

    const float landing_yaw_criteria = 0.035;

    cv::Point3f _target = {0};

    int spinup_detected = 0;
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

    cv::Point2f _blink_im_location;
    float _blink_im_disparity;
    float _blink_im_size = 5;
    float _drone_takeoff_im_size;
    cv::Point2f _drone_takeoff_im_location;
    cv::Point3f _blink_world_location;
    bool _landing_pad_location_set = false;
    cv::Point3f _landing_pad_world;

    bool enable_takeoff_motion_delete = false;

    double deviation_vec1_length;
    double deviation_vec2_length;
    double deviation_angle;

    bool enable_viz_diff = false;
    bool _manual_flight_mode = false;
    bool _hover_mode = false;

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

    void clean_ignore_blobs(double time);

    void calc_takeoff_prediction();
    void delete_takeoff_fake_motion();
    bool detect_lift_off();
    bool detect_takeoff();
    void detect_deviation_angle();


    xmls::LandingParameters landing_parameter;

public:
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    cv::Point2f blnk_im_location() { return _blink_im_location; } // scaled with imscalef
    float blnk_im_size() { return _blink_im_size; } // scaled with imscalef
    float blink_im_disparity() { return _blink_im_disparity; }
    float drone_takeoff_im_size() { return _drone_takeoff_im_size; } // NOT scaled with imscalef
    cv::Point2f drone_takeoff_im_location() { return _drone_takeoff_im_location;} // NOT scaled with imscalef

    cv::Point3f drone_takeoff_location() {return _blink_world_location + cv::Point3f(0,0,-dparams.radius);}

    cv::Point3f drone_landing_location(bool landing_hack) {

        cv::Point3f hack = {0};
        if (landing_hack)
            hack = cv::Point3f(0,0,0.04);
        if(landing_parameter.initialized)
            return cv::Point3f(landing_parameter.x, landing_parameter.y, landing_parameter.z) + hack;
        else
            return _landing_pad_world + hack;
    }

    bool take_off_detection_failed() { return _take_off_detection_failed;}

    bool taking_off() { return _drone_tracking_status == dts_detecting_takeoff_init || _drone_tracking_status == dts_detecting_takeoff;}
    bool landing() { return _drone_tracking_status == dts_landing_init || _drone_tracking_status == dts_landing;}
    bool inactive() { return _drone_tracking_status == dts_inactive;}

    void manual_flight_mode(bool value) {
        _manual_flight_mode =value;
    }
    void hover_mode(bool value) {
        if (value !=_hover_mode) {
            _hover_mode =value;
            if (_hover_mode) {
                pos_smth_width = pparams.fps/15;
                vel_smth_width = pparams.fps/15;
                acc_smth_width = pparams.fps/15;
                disparity_filter_rate = 0.7;
            } else {
                pos_smth_width = pparams.fps/30;
                vel_smth_width = pparams.fps/30;
                acc_smth_width = pparams.fps/30;
                disparity_filter_rate = 0.8;
            }
            smoother_posX.change_width(pos_smth_width);
            smoother_posY.change_width(pos_smth_width);
            smoother_posZ.change_width(pos_smth_width);
            smoother_velX.change_width(vel_smth_width);
            smoother_velY.change_width(vel_smth_width);
            smoother_velZ.change_width(vel_smth_width);
            smoother_accX.change_width(acc_smth_width);
            smoother_accY.change_width(acc_smth_width);
            smoother_accZ.change_width(acc_smth_width);
        }
    }
    cv::Mat diff_viz;

    bool init(std::ofstream *logger, VisionData *_visdat, int16_t viz_id);
    void update(double time, bool drone_is_active);

    void land() {_drone_tracking_status = dts_landing_init;}
    void detect_yaw(double time) {
        time_yaw_not_ok = time;
        _drone_tracking_status = dts_detect_yaw;
    }

    const float min_yaw_deviation = 0.5f;
    const double min_deviate_vec_length = 0.05;

    bool check_yaw(double time) {
        if (!check_deviation_vec_length() || !last_track_data().yaw_deviation_valid)
            time_yaw_not_ok = time;

        if (time - time_yaw_not_ok > min_yaw_ok_time && time_yaw_not_ok > 0.1 )
            return false;
        else
            return true;
    }

    bool check_deviation_vec_length() {
        return deviation_vec1_length < min_deviate_vec_length && deviation_vec2_length < min_deviate_vec_length;
    }

    void set_drone_landing_location(cv::Point2f im, float drone_im_disparity,float drone_im_size, cv::Point3f world) {
        _blink_im_location = im;
        _blink_im_size = drone_im_size;
        _blink_im_disparity = drone_im_disparity;
        _blink_world_location = world;
        std::cout << "blink-location: " << _blink_world_location << std::endl;
        if (!_landing_pad_location_set) { // for now, assume the first time set is the actual landing location.
            _landing_pad_world = drone_takeoff_location();
            _landing_pad_location_set = true;
        }
        _target = drone_takeoff_location();
        _drone_takeoff_im_size = world2im_size(_blink_world_location+cv::Point3f(dparams.radius,0,0),_blink_world_location-cv::Point3f(dparams.radius,0,0),_visdat->Qfi,_visdat->camera_angle);
        _drone_takeoff_im_location =  world2im_2d(drone_takeoff_location(),_visdat->Qfi,_visdat->camera_angle);
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
