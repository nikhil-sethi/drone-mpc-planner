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
                                                    "dts_detected" };
class DroneTracker : public ItemTracker {


private:
    double start_take_off_time = 0;
    double current_time = 0;

    double startup_location_ignore_timeout = 1; // TODO: make this dependent on the motion_update_iterator_max
    double taking_off_ignore_timeout = 0.1; // TODO: make this dependent on the motion_update_iterator_max

    const float full_throttle_im_effect = 3; // how many pixels per second will the drone go up given full throttle

    enum drone_tracking_states {
        dts_init = 0,
        dts_inactive,
        dts_detecting_takeoff_init,
        dts_detecting_takeoff,
        dts_detecting,
        dts_detected
    };
    drone_tracking_states _drone_tracking_status = dts_init;

    cv::Point2f _drone_blink_im_location;
    float _drone_blink_im_disparity;
    float _drone_blink_im_size = 5;
    cv::Point3f _drone_blink_world_location;
    bool _landing_pad_location_set = false;
    cv::Point3f _landing_pad_world;

    bool _drone_control_prediction_valid = false;
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

    cv::Mat Qfi; // tmp
    void update_drone_prediction(); //tmp
protected:
    void init_settings();
public:
    std::string drone_tracking_state() {return drone_tracking_state_names[_drone_tracking_status];}
    cv::Point2f drone_startup_im_location(){ return _drone_blink_im_location; }
    float drone_startup_im_size(){ return _drone_blink_im_size; }
    float drone_startup_im_disparity(){ return _drone_blink_im_disparity; }
    cv::Point3f drone_startup_location() {return _drone_blink_world_location;}
    cv::Point3f drone_landing_location() {return _landing_pad_world;}

    bool taking_off(){ return _drone_tracking_status == dts_detecting_takeoff_init || _drone_tracking_status == dts_detecting_takeoff;}
    bool inactive(){ return _drone_tracking_status == dts_inactive;}

    bool _manual_flight_mode = false;
    cv::Mat diff_viz;

    bool init(std::ofstream *logger, VisionData *_visdat);
    void track(double time, bool drone_is_active);

    void insert_control_predicted_drone_location(){
        if (_drone_control_prediction_valid){
            _drone_control_prediction_valid = false;
        }
    }

    void control_predicted_drone_location(cv::Point2f drone_control_predicted_image_location, cv::Point3f drone_control_predicted_world_location){
        _drone_control_predicted_image_location = drone_control_predicted_image_location;
        _drone_control_predicted_world_location = drone_control_predicted_world_location;
        _drone_control_prediction_valid = true;
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
    BlobWorldProps calc_tmp_world_item(BlobProps * pbs);
    bool check_ignore_blobs(BlobProps * pbs, uint id);

    double time_since_take_off(){return start_take_off_time - current_time;}

    bool delete_me(){return false;}
};




#endif //DRONETRACKER_H
