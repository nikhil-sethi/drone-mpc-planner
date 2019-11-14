#ifndef DRONETRACKER_H
#define DRONETRACKER_H

#include "itemtracker.h"
#include "tinyxml/XMLSerialization.h"

/*
 * This class will track a micro drone with leds
 *
 */
static const char* drone_tracking_state_names[] = { "dts_init",
                                                   "dts_inactive",
                                                   "dts_detecting_takeoff_init",
                                                   "dts_detecting_takeoff",
                                                   "dts_detecting",
                                                   "dts_detected",
                                                   "dts_reset_heading",
                                                   "dts_landing_init",
                                                   "dts_landing"};
class DroneTracker : public ItemTracker {

private:
    double start_take_off_time = 0;
    double current_time = 0;

    double startup_location_ignore_timeout = 1; // TODO: make this dependent on the motion_update_iterator_max
    double landing_ignore_timeout = 5; // TODO: make this dependent on the motion_update_iterator_max
    double taking_off_ignore_timeout = 0.1; // TODO: make this dependent on the motion_update_iterator_max

    bool find_heading = false;
    float heading;
    const float landing_heading_criteria = 0.005;

    enum drone_tracking_states {
        dts_init = 0,
        dts_inactive,
        dts_detecting_takeoff_init,
        dts_detecting_takeoff,
        dts_detecting,
        dts_detected,
        dts_reset_heading,
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

    cv::Point2f _drone_control_predicted_image_location = {0};
    cv::Point3f _drone_control_predicted_world_location = {0};

    bool enable_viz_diff = false;

    void clean_ignore_blobs(double time);

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

    void update_drone_prediction(); //tmp

    float takeoff_detection_dy_prev;
    float takeoff_detection_dt_prev;

public:
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    cv::Point2f drone_startup_im_location(){ return _drone_blink_im_location; }
    float drone_startup_im_size(){ return _drone_blink_im_size; }
    float drone_startup_im_disparity(){ return _drone_blink_im_disparity; }
    cv::Point3f drone_startup_location() {return _drone_blink_world_location + cv::Point3f(0,0,-0.04);} // TODO: drone dependent offset!
    cv::Point3f drone_landing_location() {return _landing_pad_world;}

    bool taking_off(){ return _drone_tracking_status == dts_detecting_takeoff_init || _drone_tracking_status == dts_detecting_takeoff;}
    bool inactive(){ return _drone_tracking_status == dts_inactive;}

    bool _manual_flight_mode = false;
    cv::Mat diff_viz;

    bool init(std::ofstream *logger, VisionData *_visdat);
    void track(double time, bool drone_is_active);

    void land() {_drone_tracking_status = dts_landing_init;}
    void reset_heading() {_drone_tracking_status = dts_reset_heading;}
    bool check_heading() { return ((fabs(heading)<landing_heading_criteria) && (fabs(heading)!=0));}

    void control_predicted_drone_location(cv::Point2f drone_control_predicted_image_location, cv::Point3f drone_control_predicted_world_location){
        _drone_control_predicted_image_location = drone_control_predicted_image_location;
        _drone_control_predicted_world_location = drone_control_predicted_world_location;
    }

    void set_drone_landing_location(cv::Point2f im, float drone_im_disparity,float drone_im_size, cv::Point3f world) {
        _drone_blink_im_location = im;
        _drone_blink_im_size = drone_im_size;
        _drone_blink_im_disparity = drone_im_disparity;
        _drone_blink_world_location = world;
        if (!_landing_pad_location_set){ // for now, assume the first time set is the actual landing location.
            _landing_pad_world = world;
            _landing_pad_location_set = true;
        }
    }
    BlobWorldProps calc_world_item(BlobProps * pbs, double time);
    bool check_ignore_blobs(BlobProps * pbs, double time);

    double time_since_take_off(){return start_take_off_time - current_time;}

    bool delete_me(){return false;}

    float hover_throttle_estimation;
    cv::Mat get_big_blob(cv::Mat Mask, int connectivity);
    cv::Mat extract_mask_column(cv::Mat mask_big, float range_left, float range_right, float side_percentage, enum side side_);
    cv::Mat split_mask_half(cv::Mat mask_big, enum side);
    float yaw_heading(cv::Mat left, cv::Mat right);
    float calc_heading(BlobProps * pbs, bool inspect_blob);
};




#endif //DRONETRACKER_H
