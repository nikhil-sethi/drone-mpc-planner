#ifndef COMMON_H
#define COMMON_H

#include "defines.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "tinyxml/XMLSerialization.h"
#include <iostream>
#include <fstream>

cv::Point2f world2im_2d(cv::Point3f p, cv::Mat Qfi, float camera_angle);
cv::Point3f world2im_3d(cv::Point3f p, cv::Mat Qfi, float camera_angle);
cv::Point3f im2world(cv::Point2f p_im, float disparity, cv::Mat Qf, float camera_angle);
bool file_exist (const std::string& name);
void combine_image(cv::Mat iml, cv::Mat imr, cv::Mat *res);
void combine_gray_image(cv::Mat iml,cv::Mat imr,cv::Mat *res);
cv::Mat create_column_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
cv::Mat create_row_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
std::string to_string_with_precision(float f, const int n);
std::string to_string_with_precision(double f, const int n); // TODO: template?
int seconds_since_file_creation(std::string file_path);
/** @brief If the string holds an drone id (number between 0 and 9) that number is returned.
 * If not, return -1. */
int get_drone_id(std::string s);

const float FOV = 180.0f ;
const float FOV_size = 1280.0;
const int width_ff = 1280;
const int height_ff = 960;
const float FOVx_ff = FOV * (static_cast<float>(width_ff)/FOV_size);
const float FOVy_ff = FOV * (static_cast<float>(height_ff)/FOV_size);
const float pix2degx_ff = (FOVx_ff / static_cast<float>(width_ff));
const float pix2degy_ff = (FOVy_ff / static_cast<float>(height_ff));

const int width_dm = 96;
const int height_dm = 96;
const int width_dmB = 576;
const int height_dmB = 576;
const float FOVx_dm = FOVx_ff*(static_cast<float>(width_dmB)/static_cast<float>(width_ff));
const float FOVy_dm = FOVy_ff*(static_cast<float>(height_dmB)/static_cast<float>(height_ff));
const float pix2degx_dm = (FOVx_dm / static_cast<float>(width_dm));
const float pix2degy_dm = (FOVy_dm / static_cast<float>(height_dm));
const float pix2degx_dmB = (FOVx_dm / static_cast<float>(width_dmB));
const float pix2degy_dmB = (FOVy_dm / static_cast<float>(height_dmB));

const float depthscale = 256.0f;

const float rad2deg = 180.f/M_PIf32;
const float deg2rad = M_PIf32/180.f;

struct state_data {
    cv::Point3f pos = {0},vel ={0},acc = {0};
};

struct track_data {
    state_data state;
    cv::Point3f pos() {return state.pos;}
    cv::Point3f vel() {return state.vel;}
    cv::Point3f acc() {return state.acc;}
    float sposX = 0,sposY = 0,sposZ = 0;
    float dt = 0;
    bool pos_valid = false;
    bool vel_valid = false;
    bool acc_valid = false;
    double time = 0;
};

struct control_data {
    control_data(float r, float tr, float p, double t){
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

enum video_mode {
    video_disabled = 0,
    video_mp4,
    video_stream,
    video_mp4_opencv,
    video_bag
};
static const char* video_mode_str[] = {"video_disabled",
    "video_mp4",
    "video_stream",
    "video_mp4_opencv",
    "video_bag"
    "" // must be the last entry! (check in serializer)
};
enum rc_type {
    rc_none = 0,
    rc_devo,
    rc_usb_hobbyking,
    rc_playstation,
    rc_xlite
};
static const char* rc_type_str[] = {"rc_none",
    "rc_devo",
    "rc_usb_hobbyking",
    "rc_playstation",
    "rc_xlite"
    "" // must be the last entry! (check in serializer)
};
enum tx_protocol {
    tx_none = 0,
    tx_dsmx,
    tx_cx10,
    tx_frskyd8,
    tx_frskyd16
};
static const char* tx_protocol_str[] = {"tx_none",
    "tx_dsmx",
    "tx_cx10",
    "tx_frskyd8",
    "tx_frskyd16"
    "" // must be the last entry! (check in serializer)
};
enum drone_type {
    drone_none,
    drone_trashcan,
    drone_tinywhoop_d16,
    drone_tinywhoop_d8,
    drone_cx10
};
static const char* drone_type_str[] = {"drone_none",
    "drone_trashcan",
    "drone_tinywhoop_d16",
    "drone_tinywhoop_d8",
    "drone_cx10",
    "" // must be the last entry! (check in serializer)
};

class CameraVolume{
public:
    void init(float slope_top, float slope_front, float slope_left, float slope_right,
              float depth, float height);

    enum volume_check_mode{
        strict, /**< viewable volume including a safety distance to borders */
        relaxed /**< the actual viewable volume without any safety distance to the borders */
    };
    /** @brief Checks if the point is in the viewable area.*/
    bool in_view(cv::Point3f p,volume_check_mode c);

    /** @brief Checks whether the point m (aka moth location) is in a good area for a hunt (worth to take off).
    * This area is described as cone above the drone location d. */
    bool in_hunt_area(cv::Point3f d, cv::Point3f m);

    /** @brief Calculates the distance to the borders */
    float calc_distance_to_borders(track_data data_drone);

private:
    // These parameters define the volume
    float slope_top;
    float slope_front;
    float slope_left;
    float slope_right;
    float z_limit;
    float y_limit;

    /** @brief Checks whether the point p is for all planes defined in init on the right side.*/
    bool in_view(cv::Point3f p, float hysteresis_margin);

    /** @brief Calculates the distance to the borders. */
    float calc_distance_to_borders(std::vector<cv::Point3f> p);

    /** @brief Calculates the distance along p till the plane is intercepted.*/
    float calc_distance_to_plane(cv::Mat vec, cv::Mat plane);

    /** @brief Calculates an orthoogonal vector to a given vector. */
    cv::Mat get_orthogonal_vector(cv::Mat vec);
};


namespace xmls {

class xVideo_mode: public MemberBase
{
private:
    void AssignValue(const video_mode value){
        m_sValue = video_mode_str[value];
    };
public:
    xVideo_mode() {AssignValue(static_cast<video_mode>(0));};
    xVideo_mode(video_mode value) {AssignValue(value);};
    video_mode value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; video_mode_str[i] != string(""); i++) {
            if (video_mode_str[i] == sHelp)
                return static_cast<video_mode>(i);
        }

        return static_cast<video_mode>(0);
    };

    xVideo_mode operator=(const video_mode value) {AssignValue(value);return *this;};
};

class xRc_type: public MemberBase
{
private:
    void AssignValue(const rc_type value){
        m_sValue = rc_type_str[value];
    };
public:
    xRc_type() {AssignValue(static_cast<rc_type>(0));};
    xRc_type(rc_type value) {AssignValue(value);};
    rc_type value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; rc_type_str[i] != string(""); i++) {
            if (rc_type_str[i] == sHelp)
                return static_cast<rc_type>(i);
        }

        return static_cast<rc_type>(0);
    };

    xRc_type operator=(const rc_type value) {AssignValue(value);return *this;};
};

class xTx_protocol: public MemberBase
{
private:
    void AssignValue(const tx_protocol value){
        m_sValue = tx_protocol_str[value];
    };
public:
    xTx_protocol() {AssignValue(static_cast<tx_protocol>(0));};
    xTx_protocol(tx_protocol value) {AssignValue(value);};
    tx_protocol value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; tx_protocol_str[i] != string(""); i++) {
            if (tx_protocol_str[i] == sHelp)
                return static_cast<tx_protocol>(i);
        }

        return static_cast<tx_protocol>(0);
    };

    xTx_protocol operator=(const tx_protocol value) {AssignValue(value);return *this;};
};

class xDrone_type: public MemberBase
{
private:
    void AssignValue(const drone_type value){
        m_sValue = drone_type_str[value];
    };
public:
    xDrone_type() {AssignValue(static_cast<drone_type>(0));};
    xDrone_type(drone_type value) {AssignValue(value);};
    drone_type value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; drone_type_str[i] != string(""); i++) {
            if (drone_type_str[i] == sHelp)
                return static_cast<drone_type>(i);
        }

        return static_cast<drone_type>(0);
    };

    xDrone_type operator=(const drone_type value) {AssignValue(value);return *this;};
};



class PatsParameters: public Serializable
{

private: std::string settings_file = "../../xml/pats.xml";
private: xBool _insect_logging_mode,_watchdog,_has_screen;
private: xVideo_mode _video_cuts,_video_raw, _video_result;
private: xRc_type _joystick;
private: xDrone_type _drone;
private: xInt _wdt_timeout_us,_darkness_threshold,_fps;
private: xBool _cam_tuning, _control_tuning, _navigation_tuning,_vision_tuning,_drone_tracking_tuning,_insect_tracking_tuning;
private: xBool _viz_plots, _viz_tracking;
private: xInt _imscalef;

public: int wdt_timeout_us,darkness_threshold;
public: uint fps;
public: bool insect_logging_mode,watchdog,has_screen;
public: video_mode video_cuts,video_raw, video_result;
public: rc_type joystick;
public: drone_type drone;
public: bool cam_tuning, control_tuning, navigation_tuning,vision_tuning,drone_tracking_tuning, insect_tracking_tuning;
public: bool viz_plots, viz_tracking;
public: int imscalef;

public: PatsParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("PatsParameters");

        // Set class version
        setVersion("1.2");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("wdt_timeout_us",&_wdt_timeout_us);
        Register("darkness_threshold",&_darkness_threshold);
        Register("has_screen",&_has_screen);
        Register("insect_logging_mode",&_insect_logging_mode);
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

    }
public: void deserialize() {
        std::cout << "Reading settings from: " << settings_file << std::endl;
        if (file_exist(settings_file)) {
            std::ifstream infile(settings_file);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            { // Deserialization not successful
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
        has_screen = _has_screen.value();
        insect_logging_mode = _insect_logging_mode.value();
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
    }

public: void serialize() {
        _wdt_timeout_us = wdt_timeout_us;
        _darkness_threshold = darkness_threshold;
        _has_screen = has_screen;
        _insect_logging_mode = insect_logging_mode;
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
public: int drone_blink_strength;
public: float blink_period;
public: tx_protocol tx;
public: bool mode3d;
public: string control;

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
private: xInt _drone_blink_strength;
private: xTx_protocol _tx;
private: xBool _mode3d;
private: xString _control;

public: DroneParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("DroneParameters");

        // Set class version
        setVersion("1.3");

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
        Register("drone_blink_strength",&_drone_blink_strength);
        Register("tx",&_tx);
        Register("mode3d",&_mode3d);
        Register("control",&_control);
    }
public: void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            { // Deserialization not successful
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
        drone_blink_strength = _drone_blink_strength.value();
        tx = _tx.value();
        mode3d = _mode3d.value();
        control = _control.value();
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
        _drone_blink_strength = drone_blink_strength;
        _tx = tx;
        _mode3d = mode3d;
        _control = control;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream (filepath);
        outfile << xmlData ;
        outfile.close();
    }
};
}
extern xmls::DroneParameters dparams;
extern xmls::PatsParameters pparams;

#endif //COMMON_H

