#ifndef DRONETRACKER_H
#define DRONETRACKER_H

#include "itemtracker.h"

#include "tinyxml/XMLSerialization.h"

/*
 * This class will track a micro drone with leds
 *
 */
static const char* blinking_drone_state_names[] = { "",
                                                    "bds_start",
                                                    "bds_reset_bkg",
                                                    "bds_searching",
                                                    "bds_1_blink_off",
                                                    "bds_1_blink_on",
                                                    "bds_2_blink_off",
                                                    "bds_2_blink_on",
                                                    "bds_3_blink_off_calib",
                                                    "bds_3_blink_off",
                                                    "bds_3_blink_on",
                                                    "bds_dedicated_calib",
                                                    "bds_calib_wait",
                                                    "bds_found" };
static const char* drone_tracking_state_names[] = { "dts_init",
                                                    "dts_blinking",
                                                    "dts_inactive",
                                                    "dts_detecting_takeoff",
                                                    "dts_detecting",
                                                    "dts_detected" };
class DroneTracker : public ItemTracker {


private:
    enum blinking_drone_states {
        bds_start=1,
        bds_reset_bkg,
        bds_searching,
        bds_1_blink_off,
        bds_1_blink_on,
        bds_2_blink_off,
        bds_2_blink_on,
        bds_3_blink_off_calib,
        bds_3_blink_off,
        bds_3_blink_on,
        bds_dedicated_calib,
        bds_calib_wait,
        bds_found
    };
    blinking_drone_states _blinking_drone_status = bds_start;
    float blink_time_start = 0;
    float start_take_off_time = 0;
    float manual_calib_time_start = 0;
    float current_time = 0;

    float startup_location_ignore_timeout = 1; // TODO: make this dependent on the motion_update_iterator_max
    float taking_off_ignore_timeout = 0.1f; // TODO: make this dependent on the motion_update_iterator_max

    enum drone_tracking_states {
        dts_init = 0,
        dts_blinking,
        dts_inactive,
        dts_detecting_takeoff_init,
        dts_detecting_takeoff,
        dts_detecting,
        dts_detected
    };
    drone_tracking_states _drone_tracking_status = dts_init;

    blinking_drone_states detect_blink(float time, bool found);
    cv::KeyPoint blink_location;

    cv::Point2f _drone_blink_image_location;
    cv::Point3f _drone_blink_world_location;
    cv::Point3f _drone_blink_world_location_start;

    bool _landing_pad_location_set = false;
    cv::Point3f _landing_pad_world_location;
    cv::Point2f _landing_pad_image_location;

    bool _drone_control_prediction_valid = false;

    bool enable_viz_diff = false;

    void clean_additional_ignores(float time);

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
    std::string calib_fn = "drone_calib.xml";

protected:
    cv::Mat get_probability_cloud(cv::Point size);
    void init_settings();
    cv::Mat get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size);
public:
    std::string Blinking_Drone_State() {
        return blinking_drone_state_names[_blinking_drone_status];
    }
    std::string Drone_Tracking_State() {
        return drone_tracking_state_names[_drone_tracking_status];
    }

    cv::Point2f Drone_Startup_Im_Location(){
        return _drone_blink_image_location;
    }

    bool taking_off(){
        return _drone_tracking_status == dts_detecting_takeoff;
    }

    cv::Mat _cir;
    cv::Mat diff_viz;

    std::vector<ItemTracker::additional_ignore_point> ignores_for_insect_tracker;

    bool init(std::ofstream *logger, VisionData *_visdat, bool fromfile,std::string bag_dir);
    void track(float time, std::vector<track_item> ignore, bool drone_is_active);

    void Locate_Startup_Location() {
        _drone_tracking_status = dts_blinking;
        _blinking_drone_status = bds_start;
    }
    void do_post_takeoff_detection() {
        cv::Point p = Drone_Startup_Im_Location();
        p.y-=5;
        DroneTracker::track_item ti(cv::KeyPoint(p,10),_visdat->frame_id,0.75);
        cv::Point3f p3 = Drone_Startup_Location();
        p3.y+=0.15f;
        predicted_locationL_last = p3;
//        _drone_tracking_status = dts_inactive;
//        pathL.clear();
//        predicted_pathL.clear();
//        predicted_pathL.push_back(ti);
//        foundL = false;
    }
    void drone_location(cv::Point p){
       _drone_blink_image_location = p/IMSCALEF;
    }

    void insert_control_predicted_drone_location(){
        if (_drone_control_prediction_valid){
            find_result.best_image_locationL.pt = _drone_control_predicted_image_location;
            _drone_control_prediction_valid = false;
        }
    }

    void control_predicted_drone_location(cv::Point2f drone_control_predicted_image_location, cv::Point3f drone_control_predicted_world_location){
        _drone_control_predicted_image_location = drone_control_predicted_image_location;
        _drone_control_predicted_world_location = drone_control_predicted_world_location;
        _drone_control_prediction_valid = true;
    }

    bool blinking_drone_located() {
        return _blinking_drone_status >= bds_found;
    }
    cv::Point3f Drone_Startup_Location() {
        return _drone_blink_world_location_start;
    }

    float time_since_take_off(){
       return start_take_off_time - current_time;
    }

    const float bind_blink_time = 0.45f;

    cv::Point2f _drone_control_predicted_image_location = {0};
    cv::Point3f _drone_control_predicted_world_location = {0};

};




#endif //DRONETRACKER_H
