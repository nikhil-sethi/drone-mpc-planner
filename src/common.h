#pragma once
#include "defines.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "third_party/tinyxml/XMLSerialization.h"
#include <iostream>
#include <fstream>

int sign(float x);
cv::Point2f world2im_2d(cv::Point3f p, cv::Mat Qfi, float camera_angle);
cv::Point3f world2im_3d(cv::Point3f p, cv::Mat Qfi, float camera_angle);
cv::Point3f im2world(cv::Point2f p_im, float disparity, cv::Mat Qf, float camera_angle);
int world2im_size(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi);
bool file_exist (const std::string& name);
bool path_exist(const std::string &s);
void combine_image(cv::Mat iml, cv::Mat imr, cv::Mat *res);
void combine_gray_image(cv::Mat iml,cv::Mat imr,cv::Mat *res);
cv::Mat create_column_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
cv::Mat create_row_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
std::string to_string_with_precision(float f, const int n);
std::string to_string_with_precision(double f, const int n); // TODO: template?
cv::Point3f multf(cv::Point3f  p1, cv::Point3f p2);
float normf(cv::Point2f m);
float normf(cv::Point3f m);
float deadzone( float v, float lo, float hi );
cv::Point3f deadzone(cv::Point3f p,float lo, float hi);

cv::Point3f pats_to_betaflight_coord(cv::Point3f vec);
cv::Point3f betaflight_to_pats_coord(cv::Point3f vec);

const float rad2deg = 180.f/M_PIf32;
const float deg2rad = M_PIf32/180.f;

extern std::string data_output_dir;

struct state_data {
    cv::Point3f pos = {0},vel = {0},acc = {0};
};

struct track_data {
    state_data state;
    cv::Point3f pos() {return state.pos;}
    cv::Point3f vel() {return state.vel;}
    cv::Point3f acc() {return state.acc;}
    float posX_smooth = 0,posY_smooth = 0,posZ_smooth = 0;
    float dt = 0;
    bool pos_valid = false;
    bool vel_valid = false;
    bool acc_valid = false;
    bool yaw_valid = false;
    double time = 0;
    float yaw = 0,yaw_smooth = 0;
};

struct control_data {
    control_data(float r, float tr, float p, double t) {
        throttle = tr;
        roll = r;
        pitch = p;
        time = t;
    }
    float throttle,roll,pitch;
    double time;
};

struct my_exit : public std::exception {
    std::string msg;
    my_exit(std::string return_value) : msg(return_value) {}
};
struct bag_video_ended : public std::exception {
    bag_video_ended() {}
};


enum video_modes {
    video_disabled = 0,
    video_mp4,
    video_stream,
    video_mp4_opencv,
    video_bag
};
static const char* video_modes_str[] = {
    "video_disabled",
    "video_mp4",
    "video_stream",
    "video_mp4_opencv",
    "video_bag"
    "" // must be the last entry! (check in serializer)
};
enum rc_types {
    rc_none = 0,
    rc_devo,
    rc_xlite
};
static const char* rc_types_str[] = {
    "rc_none",
    "rc_devo",
    "rc_xlite"
    "" // must be the last entry! (check in serializer)
};
enum tx_protocols {
    tx_none = 0,
    tx_dsmx,
    tx_cx10,
    tx_frskyd8,
    tx_frskyd16
};
static const char* tx_protocols_str[] = {
    "tx_none",
    "tx_dsmx",
    "tx_cx10",
    "tx_frskyd8",
    "tx_frskyd16"
    "" // must be the last entry! (check in serializer)
};
enum drone_types {
    drone_none,
    drone_trashcan,
    drone_hammer,
    drone_tinywhoop_d16,
    drone_tinywhoop_d8,
    drone_cx10
};
static const char* drone_types_str[] = {
    "drone_none",
    "drone_trashcan",
    "drone_hammer",
    "drone_tinywhoop_d16",
    "drone_tinywhoop_d8",
    "drone_cx10",
    "" // must be the last entry! (check in serializer)
};
enum op_modes {
    op_mode_monitoring_only,
    op_mode_crippled,
    op_mode_deployed
};
static const char* op_modes_str[] = {
    "op_mode_monitoring_only",
    "op_mode_crippled",
    "op_mode_deployed",
    "" // must be the last entry! (check in serializer)
};


namespace xmls {

class xVideo_mode: public MemberBase
{
private:
    void AssignValue(const video_modes value) {
        m_sValue = video_modes_str[value];
    };
public:
    xVideo_mode() {AssignValue(static_cast<video_modes>(0));};
    xVideo_mode(video_modes value) {AssignValue(value);};
    video_modes value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; video_modes_str[i] != string(""); i++) {
            if (video_modes_str[i] == sHelp)
                return static_cast<video_modes>(i);
        }
        throw my_exit("wrong video_mode: " + sHelp);
    };

    xVideo_mode operator=(const video_modes value) {AssignValue(value); return *this;};
};

class xRc_type: public MemberBase
{
private:
    void AssignValue(const rc_types value) {
        m_sValue = rc_types_str[value];
    };
public:
    xRc_type() {AssignValue(static_cast<rc_types>(0));};
    xRc_type(rc_types value) {AssignValue(value);};
    rc_types value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; rc_types_str[i] != string(""); i++) {
            if (rc_types_str[i] == sHelp)
                return static_cast<rc_types>(i);
        }
        throw my_exit("wrong rc_type: " + sHelp);
    };

    xRc_type operator=(const rc_types value) {AssignValue(value); return *this;};
};

class xTx_protocol: public MemberBase
{
private:
    void AssignValue(const tx_protocols value) {
        m_sValue = tx_protocols_str[value];
    };
public:
    xTx_protocol() {AssignValue(static_cast<tx_protocols>(0));};
    xTx_protocol(tx_protocols value) {AssignValue(value);};
    tx_protocols value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; tx_protocols_str[i] != string(""); i++) {
            if (tx_protocols_str[i] == sHelp)
                return static_cast<tx_protocols>(i);
        }
        throw my_exit("wrong tx_protocol: " + sHelp);
    };

    xTx_protocol operator=(const tx_protocols value) {AssignValue(value); return *this;};
};

class xOp_mode: public MemberBase
{
private:
    void AssignValue(const op_modes value) {
        m_sValue = op_modes_str[value];
    };
public:
    xOp_mode() {AssignValue(static_cast<op_modes>(0));};
    xOp_mode(op_modes value) {AssignValue(value);};
    op_modes value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);
        for (uint i = 0; op_modes_str[i] != string(""); i++) {
            if (op_modes_str[i] == sHelp)
                return static_cast<op_modes>(i);
        }
        throw my_exit("wrong op_mode: " + sHelp);
    };

    xOp_mode operator=(const op_modes value) {AssignValue(value); return *this;};
};

class xDrone_type: public MemberBase
{
private:
    void AssignValue(const drone_types value) {
        m_sValue = drone_types_str[value];
    };
public:
    xDrone_type() {AssignValue(static_cast<drone_types>(0));};
    xDrone_type(drone_types value) {AssignValue(value);};
    drone_types value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; drone_types_str[i] != string(""); i++) {
            if (drone_types_str[i] == sHelp)
                return static_cast<drone_types>(i);
        }
        throw my_exit("wrong drone_type: " + sHelp);
    };

    xDrone_type operator=(const drone_types value) {AssignValue(value); return *this;};
};

class PatsParameters: public Serializable
{

private: xBool _watchdog,_has_screen;
private: xVideo_mode _video_cuts,_video_raw, _video_result;
private: xRc_type _joystick;
private: xDrone_type _drone;
private: xOp_mode _op_mode;
private: xInt _wdt_timeout_us,_darkness_threshold,_fps,_close_after_n_images;
private: xBool _cam_tuning, _control_tuning, _navigation_tuning,_vision_tuning,_drone_tracking_tuning,_insect_tracking_tuning;
private: xBool _viz_plots, _viz_tracking;
private: xInt _imscalef;
private: xString _flightplan;
private: xInt _live_image_frq;
private: xFloat _max_cam_roll;

public: int wdt_timeout_us,darkness_threshold,close_after_n_images;
public: uint fps;
public: bool watchdog,has_screen;
public: video_modes video_cuts,video_raw, video_result;
public: rc_types joystick;
public: drone_types drone;
public: op_modes op_mode;
public: bool cam_tuning, control_tuning, navigation_tuning,vision_tuning,drone_tracking_tuning, insect_tracking_tuning;
public: bool viz_plots, viz_tracking;
public: int imscalef;
public: std::string flightplan;
public: int live_image_frq;
public: float max_cam_roll;

public: PatsParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("PatsParameters");

        // Set class version
        setVersion("1.6");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("wdt_timeout_us",&_wdt_timeout_us);
        Register("darkness_threshold",&_darkness_threshold);
        Register("close_after_n_images",&_close_after_n_images);
        Register("has_screen",&_has_screen);
        Register("op_mode",&_op_mode);
        Register("watchdog",&_watchdog);
        Register("fps",&_fps);
        Register("video_cuts",&_video_cuts);
        Register("video_raw",&_video_raw);
        Register("video_result",&_video_result);
        Register("joystick",&_joystick);
        Register("drone",&_drone);
        Register("cam_tuning",&_cam_tuning);
        Register("control_tuning",&_control_tuning);
        Register("navigation_tuning",&_navigation_tuning);
        Register("vision_tuning",&_vision_tuning);
        Register("drone_tracking_tuning",&_drone_tracking_tuning);
        Register("insect_tracking_tuning",&_insect_tracking_tuning);
        Register("viz_plots",&_viz_plots);
        Register("viz_tracking",&_viz_tracking);
        Register("imscalef",&_imscalef);
        Register("flightplan",&_flightplan);
        Register("live_image_frq",&_live_image_frq);
        Register("max_cam_roll",&_max_cam_roll);
    }
public: void deserialize(std::string settings_file) {
        std::cout << "Reading settings from: " << settings_file << std::endl;
        if (file_exist(settings_file)) {
            std::ifstream infile(settings_file);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw my_exit("Cannot read: " + settings_file);
            }
            PatsParameters tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw my_exit("XML version difference detected from " + settings_file);
            }
        } else {
            throw my_exit("File not found: " + settings_file);
        }

        wdt_timeout_us = _wdt_timeout_us.value();
        darkness_threshold = _darkness_threshold.value();
        close_after_n_images = _close_after_n_images.value();
        has_screen = _has_screen.value();
        op_mode = _op_mode.value();
        watchdog = _watchdog.value();
        fps = _fps.value();
        video_cuts = _video_cuts.value();
        video_raw = _video_raw.value();
        video_result = _video_result.value();
        joystick = _joystick.value();
        drone = _drone.value();
        cam_tuning  = _cam_tuning.value();
        control_tuning = _control_tuning.value();
        navigation_tuning = _navigation_tuning.value();
        vision_tuning = _vision_tuning.value();
        drone_tracking_tuning = _drone_tracking_tuning.value();
        insect_tracking_tuning = _insect_tracking_tuning.value();
        viz_plots = _viz_plots.value();
        viz_tracking = _viz_tracking.value();
        imscalef = _imscalef.value();
        flightplan = _flightplan.value();
        live_image_frq = _live_image_frq.value();
        max_cam_roll = _max_cam_roll.value();
    }

public: void serialize(std::string settings_file) {
        _wdt_timeout_us = wdt_timeout_us;
        _darkness_threshold = darkness_threshold;
        _close_after_n_images = close_after_n_images;
        _has_screen = has_screen;
        _op_mode = op_mode;
        _watchdog = watchdog;
        _fps = fps;
        _video_cuts = video_cuts;
        _video_raw = video_raw;
        _video_result = video_result;
        _joystick = joystick;
        _drone = drone;
        _cam_tuning  = cam_tuning;
        _control_tuning = control_tuning;
        _navigation_tuning = navigation_tuning;
        _vision_tuning = vision_tuning;
        _drone_tracking_tuning = drone_tracking_tuning;
        _insect_tracking_tuning = insect_tracking_tuning;
        _viz_plots = viz_plots;
        _viz_tracking = viz_tracking;
        _imscalef = imscalef;
        _flightplan= flightplan;
        _live_image_frq = live_image_frq;
        _max_cam_roll = max_cam_roll;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream (settings_file);
        outfile << xmlData ;
        outfile.close();
    }
};

class DroneParameters: public Serializable
{

public: int initial_hover_throttle;
public: float throttle_bank_factor;
public: double max_burn_time;
public: int min_throttle;
public: float full_bat_and_throttle_im_effect; // how many pixels per second will the drone go up given full throttle
public: float full_bat_and_throttle_take_off_acc;
public: float full_bat_and_throttle_spinup_duration;
public: float hover_throttle_a;
public: float hover_throttle_b;
public: float blink_period;
public: float radius;
public: float thrust;
public: int drone_blink_strength;
public: int drone_led_strength;
public: tx_protocols tx;
public: bool mode3d;
public: string control;
public: int spinup_throttle_non3d;


private: xInt _initial_hover_throttle;
private: xFloat _throttle_bank_factor;
private: xFloat _max_burn_time;
private: xInt _min_throttle;
private: xFloat _full_bat_and_throttle_im_effect;
private: xFloat _full_bat_and_throttle_take_off_acc;
private: xFloat _full_bat_and_throttle_spinup_duration;
private: xFloat _hover_throttle_a;
private: xFloat _hover_throttle_b;
private: xFloat _blink_period;
private: xFloat _radius;
private: xFloat _thrust;
private: xInt _drone_blink_strength;
private: xInt _drone_led_strength;
private: xTx_protocol _tx;
private: xBool _mode3d;
private: xString _control;
private: xInt _spinup_throttle_non3d;



public: DroneParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("DroneParameters");

        // Set class version
        setVersion("1.5");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("initial_hover_throttle",&_initial_hover_throttle);
        Register("throttle_bank_factor",&_throttle_bank_factor);
        Register("max_burn_time",&_max_burn_time);
        Register("min_throttle",&_min_throttle);
        Register("full_bat_and_throttle_im_effect",&_full_bat_and_throttle_im_effect);
        Register("full_bat_and_throttle_take_off_acc",&_full_bat_and_throttle_take_off_acc);
        Register("full_bat_and_throttle_spinup_time",&_full_bat_and_throttle_spinup_duration);
        Register("hover_throttle_a",&_hover_throttle_a);
        Register("hover_throttle_b",&_hover_throttle_b);
        Register("blink_period",&_blink_period);
        Register("radius",&_radius);
        Register("thrust",&_thrust);
        Register("drone_blink_strength",&_drone_blink_strength);
        Register("drone_led_strength",&_drone_led_strength);
        Register("tx",&_tx);
        Register("mode3d",&_mode3d);
        Register("control",&_control);
        Register("spinup_throttle_non3d",&_spinup_throttle_non3d);
    }
public: void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw my_exit("Cannot read: " + filepath);
            }
            DroneParameters tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw my_exit("XML version difference detected from " + filepath);
            }
        } else {
            throw my_exit("File not found: " + filepath);
        }

        initial_hover_throttle = _initial_hover_throttle.value();
        throttle_bank_factor = _throttle_bank_factor.value();
        max_burn_time = _max_burn_time.value();
        min_throttle = _min_throttle.value();
        full_bat_and_throttle_im_effect = _full_bat_and_throttle_im_effect.value();
        full_bat_and_throttle_take_off_acc = _full_bat_and_throttle_take_off_acc.value();
        full_bat_and_throttle_spinup_duration = _full_bat_and_throttle_spinup_duration.value();
        hover_throttle_a = _hover_throttle_a.value();
        hover_throttle_b = _hover_throttle_b.value();
        blink_period = _blink_period.value();
        thrust = _thrust.value();
        radius = _radius.value();
        drone_blink_strength = _drone_blink_strength.value();
        drone_led_strength = _drone_led_strength.value();
        tx = _tx.value();
        mode3d = _mode3d.value();
        control = _control.value();
        spinup_throttle_non3d = _spinup_throttle_non3d.value();
    }

public: void serialize(std::string filepath) {

        _initial_hover_throttle = initial_hover_throttle;
        _throttle_bank_factor = throttle_bank_factor;
        _max_burn_time = max_burn_time;
        _min_throttle = min_throttle;
        _full_bat_and_throttle_im_effect = full_bat_and_throttle_im_effect;
        _full_bat_and_throttle_take_off_acc = full_bat_and_throttle_take_off_acc;
        _full_bat_and_throttle_spinup_duration = full_bat_and_throttle_spinup_duration;
        _hover_throttle_a = hover_throttle_a;
        _hover_throttle_b = hover_throttle_b;
        _blink_period = blink_period;
        _thrust = thrust;
        _radius = radius;
        _drone_blink_strength = drone_blink_strength;
        _drone_led_strength = drone_led_strength;
        _tx = tx;
        _mode3d = mode3d;
        _control = control;
        _spinup_throttle_non3d = spinup_throttle_non3d;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream (filepath);
        outfile << xmlData ;
        outfile.close();
    }
};

class LandingParameters: public Serializable {

public:
    float x, y, z;
    bool initialized = false;

    LandingParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("LandingParameters");

        // Set class version
        setVersion("1.0");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("x",&_x);
        Register("y",&_y);
        Register("z",&_z);
    }

    void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw my_exit("Cannot read: " + filepath);
            }
            LandingParameters tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw my_exit("XML version difference detected from " + filepath);
            }
        } else {
            return;
        }

        x = _x.value();
        y = _y.value();
        z = _z.value();
        initialized = true;
    }

    void serialize(std::string filepath) {
        _x = x;
        _y = y;
        _z = z;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream (filepath);
        outfile << xmlData ;
        outfile.close();
    }

private:
    xFloat _x, _y, _z;

};

}
extern xmls::DroneParameters dparams;
extern xmls::PatsParameters pparams;
