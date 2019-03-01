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
                                                    "bds_3_blink_off",
                                                    "bds_3_blink_on",
                                                    "bds_found" };
static const char* drone_tracking_state_names[] = { "dts_init",
                                                    "dts_blinking",
                                                    "dts_inactive",
                                                    "dts_detecting_takeoff",
                                                    "dts_detecting",
                                                    "dts_detected" };
class DroneTracker : public ItemTracker {


    const float bind_blink_time = 0.45f;
private:
    enum blinking_drone_states {
        bds_start=1,
        bds_reset_bkg=2,
        bds_searching=3,
        bds_1_blink_off=4,
        bds_1_blink_on=5,
        bds_2_blink_off=6,
        bds_2_blink_on=7,
        bds_3_blink_off=8,
        bds_3_blink_on=9,
        bds_found=10
    };
    blinking_drone_states _blinking_drone_status = bds_start;
    float blink_time_start = 0;

    enum drone_tracking_states {
        dts_init = 0,
        dts_blinking,
        dts_inactive,
        dts_detecting_takeoff,
        dts_detecting,
        dts_detected
    };
    drone_tracking_states _drone_tracking_status = dts_init;

    blinking_drone_states detect_blink(float time, bool found);
    cv::KeyPoint blink_location;

    cv::Point2f _drone_blink_image_location;
    cv::Point3f _drone_blink_world_location;

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

    cv::Mat _cir;

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
        _drone_tracking_status = dts_inactive;
        pathL.clear();
        predicted_pathL.clear();
        predicted_pathL.push_back(ti);
        foundL = false;
    }
    void set_drone_location(cv::Point p){
       _drone_blink_image_location = p/IMSCALEF;
    }

    bool blinking_drone_located() {
        return _blinking_drone_status >= bds_found;
    }
    cv::Point3f Drone_Startup_Location() {
        return _drone_blink_world_location;
    }
};




#endif //DRONETRACKER_H
