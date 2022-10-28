#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "third_party/tinyxml/XMLSerialization.h"
#include <iostream>
#include <fstream>
#include <mutex>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#define IMG_W 848
#define IMG_H 480
#define IMG_Wf 848.f
#define IMG_Hf 480.f
#define GRAVITY 9.81f

int sign(float x);
vector<string> split_csv_line(string);
vector<string> split_csv_line(string, char);
cv::Point2f world2im_2d(cv::Point3f p, cv::Mat Qfi, float camera_roll, float camera_pitch);
cv::Point3f world2im_3d(cv::Point3f p, cv::Mat Qfi, float camera_roll, float camera_pitch);
cv::Point3f im2world(cv::Point2f p_im, float disparity, cv::Mat Qf, float camera_roll, float camera_pitch);
int world2im_dist(cv::Point3f p1, float dist, cv::Mat Qfi, float camera_roll, float camera_pitch);
int world2im_size(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_roll, float camera_pitch);
float world2im_sizef(cv::Point3f p1, cv::Point3f p2, cv::Mat Qfi, float camera_roll, float camera_pitch);
bool file_exist(const std::string &name);
bool path_exist(const std::string &s);
void combine_image(cv::Mat iml, cv::Mat imr, cv::Mat *res);
void combine_gray_image(cv::Mat iml, cv::Mat imr, cv::Mat *res);
cv::Mat create_column_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
cv::Mat create_row_image(std::vector<cv::Mat> ims, int type, float resizef = 1);
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef = 1);
cv::Rect clamp_rect(cv::Rect r, int w, int h);
std::string to_string_with_precision(float f, const int n);
std::string to_string_with_precision(double f, const int n);
cv::Point3f multf(cv::Point3f  p1, cv::Point3f p2);
float normf(cv::Point2f m);
float normf(cv::Point3f m);
float deadzone(float v, float lo, float hi);
cv::Point3f deadzone(cv::Point3f p, float lo, float hi);
cv::Point3f pats_to_betaflight_coord(cv::Point3f vec);
cv::Point3f betaflight_to_pats_coord(cv::Point3f vec);
std::string execute(const char *cmd);
float calc_light_level(int exposure, int gain, float brightness);
bool is_number(const std::string &s);
float optimization_thrust(float thrust);
std::tuple<float, float, float> solve_quadratic_solution(float a, float b, float c);
std::vector<int> combination_sample(uint n, uint i); // Find the ith combination of n elements
void combination_sample_worker(std::vector<int> *combination, std::vector<int> *elements, uint n, uint i);
inline float make_even(float n) {
    return roundf((n / 2.f) * 2);
}
const string pats_folder = string(getpwuid(getuid())->pw_dir) + "/pats/";

const float rad2deg = 180.f / M_PIf32;
const float deg2rad = M_PIf32 / 180.f;

const int im_scaler = 2;

extern std::string data_output_dir;


struct ControlData {
    ControlData(float r, float tr, float p, double t) {
        throttle = tr;
        roll = r;
        pitch = p;
        time = t;
    }
    float throttle, roll, pitch;
    double time;
};
struct StereoPair {
    StereoPair(cv::Mat left_, cv::Mat right_, unsigned long long rs_id_, double time_) {
        left = left_;
        right = right_;
        rs_id = rs_id_;
        time = time_;
        processing = false;
    }
    cv::Mat left, right;
    unsigned long long rs_id;
    double time;
    bool processing;
};
struct NoRealsenseConnected : public std::exception {
    std::string msg;
    NoRealsenseConnected() : msg("no RealSense connected") {}
};
class NotImplemented : public std::logic_error
{
public:
    NotImplemented() : std::logic_error("Function not yet implemented") { };
};
class NoCodecAvailable : public std::logic_error
{
public:
    NoCodecAvailable() : std::logic_error("No codec available") { };
};
struct ReplayVideoEnded : public std::exception {
    ReplayVideoEnded() {}
};

enum video_modes {
    video_disabled = 0,
    video_file,
    video_stream
};
static const char *video_modes_str[] = {
    "video_disabled",
    "video_file",
    "video_stream",
    "" // must be the last entry! (check in serializer)
};
enum rc_types {
    rc_none = 0,
    rc_xlite
};
static const char *rc_types_str[] = {
    "rc_none",
    "rc_xlite",
    "" // must be the last entry! (check in serializer)
};
enum tx_protocols {
    tx_none,
    tx_dsmx,
    tx_cx10,
    tx_frskyd8,
    tx_frskyd16,
    tx_redpine
};
enum d16_subprotocols {
    d16_ch16 = 0,
    d16_ch8 = 1,
    d16_eu16 = 2,
    d16_eu8 = 3,
    d16_xclone = 4
};
enum redpine_subprotocols {
    redpine_fast = 0,
    redpine_slow = 1
};

static const char *led_types_str[] = {
    "led_none",
    "led_strip",
    "led_fiber_ir",
    "led_fiber_uv",
    "led_top_uv",
    "" // must be the last entry! (check in serializer)
};
enum led_types {
    led_none,
    led_strip,
    led_fiber_ir,
    led_fiber_uv,
    led_top_uv
};

static const char *tx_protocols_str[] = {
    "tx_none",
    "tx_dsmx",
    "tx_cx10",
    "tx_frskyd8",
    "tx_frskyd16",
    "tx_redpine"
    "" // must be the last entry! (check in serializer)
};
enum drone_types {
    drone_none,
    drone_trashcan,
    drone_hammer,
    drone_anvil_crazybee,
    drone_anvil_superbee,
    drone_qutt,
    drone_quf4,
    drone_quto,
    drone_anvil_diamond
};
static const char *drone_types_str[] = {
    "drone_none",
    "drone_trashcan",
    "drone_hammer",
    "drone_anvil_crazybee",
    "drone_anvil_superbee",
    "drone_qutt",
    "drone_quf4",
    "drone_quto",
    "drone_anvil_diamond",
    "" // must be the last entry! (check in serializer)
};
enum op_modes {
    op_mode_c,
    op_mode_x
};
static const char *op_modes_str[] = {
    "op_mode_c",
    "op_mode_x"
    "" // must be the last entry! (check in serializer)
};

enum control_modes {
    position_control,
    acceleration_feedforward
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

        for (uint i = 0; strcmp(video_modes_str[i], ""); i++) {
            if (video_modes_str[i] == sHelp)
                return static_cast<video_modes>(i);
        }
        throw std::runtime_error("wrong video_mode: " + sHelp);
    };

    xVideo_mode operator=(const video_modes value) {AssignValue(value); return *this;};
};

class xRC_type: public MemberBase {
private:
    void AssignValue(const rc_types value) {
        m_sValue = rc_types_str[value];
    };
public:
    xRC_type() {AssignValue(static_cast<rc_types>(0));};
    xRC_type(rc_types value) {AssignValue(value);};
    rc_types value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; rc_types_str[i] != string(""); i++) {
            if (rc_types_str[i] == sHelp)
                return static_cast<rc_types>(i);
        }
        throw std::runtime_error("wrong rc_type: " + sHelp);
    };

    xRC_type operator=(const rc_types value) {AssignValue(value); return *this;};
};

class xTx_protocol: public MemberBase {
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
        throw std::runtime_error("wrong tx_protocol: " + sHelp);
    };

    xTx_protocol operator=(const tx_protocols value) {AssignValue(value); return *this;};
};

class xLed_types: public MemberBase {
private:
    void AssignValue(const led_types value) {
        m_sValue = led_types_str[value];
    };
public:
    xLed_types() {AssignValue(static_cast<led_types>(0));};
    xLed_types(led_types value) {AssignValue(value);};
    led_types value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; led_types_str[i] != string(""); i++) {
            if (led_types_str[i] == sHelp)
                return static_cast<led_types>(i);
        }
        throw std::runtime_error("wrong led_type: " + sHelp);
    };

    xLed_types operator=(const led_types value) {AssignValue(value); return *this;};
};

class xOp_mode: public MemberBase {
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
        throw std::runtime_error("wrong op_mode: " + sHelp);
    };

    xOp_mode operator=(const op_modes value) {AssignValue(value); return *this;};
};

class xDrone_type: public MemberBase {
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
        throw std::runtime_error("wrong drone_type: " + sHelp);
    };

    xDrone_type operator=(const drone_types value) {AssignValue(value); return *this;};
};

class PatsParameters: public Serializable {
private:
    xFloat _light_level_threshold;
    xBool _watchdog, _has_screen;
    xVideo_mode _video_raw, _video_result;
    xRC_type _joystick;
    xDrone_type _drone;
    xOp_mode _op_mode;
    xBool _disable_real_hunts;
    xInt _fps, _periodic_restart_minutes;

    xString _enable_start, _enable_end;
    xFloat _min_hunt_size, _max_hunt_size;
    xString _location;
    xString _flightplan;
    xString _flightplan_calib_thrust;
    xInt _live_image_frq;
    xFloat _max_cam_angle;
    xInt _n_cams;
    xBool _long_range_mode;

public:
    float light_level_threshold;
    int periodic_restart_minutes;
    uint fps;
    bool watchdog, has_screen;
    video_modes video_raw, video_render;
    rc_types joystick;
    drone_types drone;
    op_modes op_mode;
    bool disable_real_hunts;
    std::string enable_start, enable_end;
    float min_hunt_size, max_hunt_size;
    std::string location;
    std::string flightplan;
    std::string flightplan_calib_thrust;
    int live_image_frq;
    float max_cam_angle;
    int n_cams;
    bool long_range_mode;


    PatsParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("PatsParameters");

        // Set class version
        setVersion("1.20");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("light_level_threshold", &_light_level_threshold);
        Register("periodic_restart_minutes", &_periodic_restart_minutes);
        Register("enable_start", &_enable_start);
        Register("enable_end", &_enable_end);
        Register("min_hunt_size", &_min_hunt_size);
        Register("max_hunt_size", &_max_hunt_size);
        Register("has_screen", &_has_screen);
        Register("op_mode", &_op_mode);
        Register("disable_real_hunts", &_disable_real_hunts);
        Register("watchdog", &_watchdog);
        Register("fps", &_fps);
        Register("video_raw", &_video_raw);
        Register("video_render", &_video_result);
        Register("joystick", &_joystick);
        Register("drone", &_drone);
        Register("location", &_location);
        Register("flightplan", &_flightplan);
        Register("flightplan_calib_thrust", &_flightplan_calib_thrust);
        Register("live_image_frq", &_live_image_frq);
        Register("max_cam_angle", &_max_cam_angle);
        Register("n_cams", &_n_cams);
        Register("long_range_mode", &_long_range_mode);

    }
    void deserialize(std::string settings_file) {
        std::cout << "Reading settings from: " << settings_file << std::endl;
        if (file_exist(settings_file)) {
            std::ifstream infile(settings_file);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw std::runtime_error("Cannot read: " + settings_file);
            }
            PatsParameters tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw std::runtime_error("XML version difference detected from " + settings_file + std::string("\n") + v1 + std::string(" / ") + v2);
            }
            infile.close();
        } else {
            throw std::runtime_error("File not found: " + settings_file);
        }

        light_level_threshold = _light_level_threshold.value();
        periodic_restart_minutes = _periodic_restart_minutes.value();
        enable_start = _enable_start.value();
        enable_end = _enable_end.value();
        min_hunt_size = _min_hunt_size.value();
        max_hunt_size = _max_hunt_size.value();
        has_screen = _has_screen.value();
        op_mode = _op_mode.value();
        disable_real_hunts = _disable_real_hunts.value();
        watchdog = _watchdog.value();
        fps = _fps.value();
        video_raw = _video_raw.value();
        video_render = _video_result.value();
        joystick = _joystick.value();
        drone = _drone.value();
        location = _location.value();
        flightplan = _flightplan.value();
        flightplan_calib_thrust = _flightplan_calib_thrust.value();
        live_image_frq = _live_image_frq.value();
        max_cam_angle = _max_cam_angle.value();
        n_cams = _n_cams.value();
        long_range_mode = _long_range_mode.value();
    }

    void serialize(std::string settings_file) {
        _light_level_threshold = light_level_threshold;
        _periodic_restart_minutes = periodic_restart_minutes;
        _enable_start = enable_start;
        _enable_end = enable_end;
        _max_hunt_size = max_hunt_size;
        _min_hunt_size = min_hunt_size;
        _has_screen = has_screen;
        _op_mode = op_mode;
        _disable_real_hunts = disable_real_hunts;
        _watchdog = watchdog;
        _fps = fps;
        _video_raw = video_raw;
        _video_result = video_render;
        _joystick = joystick;
        _drone = drone;
        _location = location;
        _flightplan = flightplan;
        _flightplan_calib_thrust = flightplan_calib_thrust;
        _live_image_frq = live_image_frq;
        _max_cam_angle = max_cam_angle;
        _n_cams = n_cams;
        _long_range_mode = long_range_mode;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream(settings_file);
        outfile << xmlData ;
        outfile.close();
    }
};

class DroneParameters: public Serializable {
private:
    xString _name;
    xInt _initial_hover_throttle;
    xInt _min_throttle;
    xFloat _full_bat_and_throttle_spinup_duration;
    xFloat _blink_period;
    xFloat _radius;
    xFloat _pad_radius;
    xFloat _max_thrust;
    xInt _drone_blink_strength;
    xInt _drone_led_strength;
    xTx_protocol _tx;
    xBool _mode3d;
    xInt _spinup_throttle_non3d;
    xFloat _land_cell_v;
    xInt _max_flight_time;
    xFloat _min_hunt_cell_v;
    xInt _static_shakeit_thrust;
    xFloat _target_takeoff_velocity;
    xLed_types _led_type;

    xFloat _kp_pos_roll, _kp_pos_pitch, _kp_pos_throttle;
    xFloat _ki_pos_roll, _ki_pos_pitch, _ki_thrust;
    xFloat _kd_pos_roll, _kd_pos_pitch, _kd_pos_throttle;
    xFloat _kp_pos_roll_hover, _kp_pos_pitch_hover, _kp_pos_throttle_hover;
    xFloat _ki_pos_roll_hover, _ki_pos_pitch_hover, _ki_thrust_hover;
    xFloat _kd_pos_roll_hover, _kd_pos_pitch_hover, _kd_pos_throttle_hover;
    xFloat _kp_pos_kiv, _kd_pos_kiv;
    xFloat _kp_vel_kiv, _kd_vel_kiv;

public:
    std::string name;
    int initial_hover_throttle;
    int min_throttle;
    float full_bat_and_throttle_spinup_duration;
    float blink_period;
    float radius;
    float pad_radius;
    float max_thrust;
    int drone_blink_strength;
    int drone_led_strength;
    tx_protocols tx;
    bool mode3d;
    int spinup_throttle_non3d;
    float land_cell_v;
    int max_flight_time;
    float min_hunt_cell_v;
    int static_shakeit_thrust;
    float target_takeoff_velocity;
    led_types led_type;
    bool Telemetry() { return tx == tx_frskyd16; }
    double betaflight_angle_strength = 130;
    double drone_rotation_delay;

    float kp_pos_roll, kp_pos_throttle, kp_pos_pitch, ki_pos_roll, ki_thrust, ki_pos_pitch, kd_pos_roll, kd_pos_throttle, kd_pos_pitch;
    float kp_pos_roll_hover, kp_pos_throttle_hover, kp_pos_pitch_hover, ki_pos_roll_hover, ki_thrust_hover, ki_pos_pitch_hover, kd_pos_roll_hover, kd_pos_throttle_hover, kd_pos_pitch_hover;

    float kp_pos_kiv, kd_pos_kiv;
    float kp_vel_kiv, kd_vel_kiv;

    DroneParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("DroneParameters");

        // Set class version
        setVersion("1.16");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("name", &_name);
        Register("initial_hover_throttle", &_initial_hover_throttle);
        Register("min_throttle", &_min_throttle);
        Register("full_bat_and_throttle_spinup_duration", &_full_bat_and_throttle_spinup_duration);
        Register("blink_period", &_blink_period);
        Register("radius", &_radius);
        Register("pad_radius", &_pad_radius);
        Register("max_thrust", &_max_thrust);
        Register("drone_blink_strength", &_drone_blink_strength);
        Register("drone_led_strength", &_drone_led_strength);
        Register("tx", &_tx);
        Register("mode3d", &_mode3d);
        Register("spinup_throttle_non3d", &_spinup_throttle_non3d);
        Register("land_cell_v", &_land_cell_v);
        Register("max_flight_time", &_max_flight_time);
        Register("min_hunt_cell_v", &_min_hunt_cell_v);
        Register("static_shakeit_thrust", &_static_shakeit_thrust);
        Register("target_takeoff_velocity", &_target_takeoff_velocity);
        Register("led_type", &_led_type);

        Register("kp_pos_roll", &_kp_pos_roll);
        Register("kp_pos_pitch", &_kp_pos_pitch);
        Register("kp_pos_throttle", &_kp_pos_throttle);
        Register("ki_pos_roll", &_ki_pos_roll);
        Register("ki_pos_pitch", &_ki_pos_pitch);
        Register("ki_thrust", &_ki_thrust);
        Register("kd_pos_roll", &_kd_pos_roll);
        Register("kd_pos_pitch", &_kd_pos_pitch);
        Register("kd_pos_throttle", &_kd_pos_throttle);

        Register("kp_pos_roll_hover", &_kp_pos_roll_hover);
        Register("kp_pos_pitch_hover", &_kp_pos_pitch_hover);
        Register("kp_pos_throttle_hover", &_kp_pos_throttle_hover);
        Register("ki_pos_roll_hover", &_ki_pos_roll_hover);
        Register("ki_pos_pitch_hover", &_ki_pos_pitch_hover);
        Register("ki_thrust_hover", &_ki_thrust_hover);
        Register("kd_pos_roll_hover", &_kd_pos_roll_hover);
        Register("kd_pos_pitch_hover", &_kd_pos_pitch_hover);
        Register("kd_pos_throttle_hover", &_kd_pos_throttle_hover);

        Register("kp_pos_kiv", &_kp_pos_kiv);
        Register("kd_pos_kiv", &_kd_pos_kiv);
        Register("kp_vel_kiv", &_kp_vel_kiv);
        Register("kd_vel_kiv", &_kd_vel_kiv);
    }

    void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw std::runtime_error("Cannot read: " + filepath);
            }
            DroneParameters tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw std::runtime_error("XML version difference detected from " + filepath);
            }
            infile.close();
        } else {
            throw std::runtime_error("File not found: " + filepath);
        }

        name = _name.value();
        initial_hover_throttle = _initial_hover_throttle.value();
        min_throttle = _min_throttle.value();
        full_bat_and_throttle_spinup_duration = _full_bat_and_throttle_spinup_duration.value();
        blink_period = _blink_period.value();
        max_thrust = _max_thrust.value();
        radius = _radius.value();
        pad_radius = _pad_radius.value();
        drone_blink_strength = _drone_blink_strength.value();
        drone_led_strength = _drone_led_strength.value();
        tx = _tx.value();
        mode3d = _mode3d.value();
        spinup_throttle_non3d = _spinup_throttle_non3d.value();
        land_cell_v = _land_cell_v.value();
        max_flight_time = _max_flight_time.value();
        min_hunt_cell_v = _min_hunt_cell_v.value();
        static_shakeit_thrust = _static_shakeit_thrust.value();
        target_takeoff_velocity = _target_takeoff_velocity.value();
        led_type = _led_type.value();

        kp_pos_roll = _kp_pos_roll.value();
        kp_pos_pitch = _kp_pos_pitch.value();
        kp_pos_throttle = _kp_pos_throttle.value();
        ki_pos_roll = _ki_pos_roll.value();
        ki_pos_pitch = _ki_pos_pitch.value();
        ki_thrust = _ki_thrust.value();
        kd_pos_roll = _kd_pos_roll.value();
        kd_pos_pitch = _kd_pos_pitch.value();
        kd_pos_throttle = _kd_pos_throttle.value();
        kp_pos_roll_hover = _kp_pos_roll_hover.value();
        kp_pos_pitch_hover = _kp_pos_pitch_hover.value();
        kp_pos_throttle_hover = _kp_pos_throttle_hover.value();
        ki_pos_roll_hover = _ki_pos_roll_hover.value();
        ki_pos_pitch_hover = _ki_pos_pitch_hover.value();
        ki_thrust_hover = _ki_thrust_hover.value();
        kd_pos_roll_hover = _kd_pos_roll_hover.value();
        kd_pos_pitch_hover = _kd_pos_pitch_hover.value();
        kd_pos_throttle_hover = _kd_pos_throttle_hover.value();
        kp_pos_kiv = _kp_pos_kiv.value();
        kd_pos_kiv = _kd_pos_kiv.value();
        kp_vel_kiv = _kp_vel_kiv.value();
        kd_vel_kiv = _kd_vel_kiv.value();

        drone_rotation_delay = 10. / betaflight_angle_strength * -log(0.37); //After a new commanded altitude, the time till the drone has made 63% the way to the new altitude
    }

    void serialize(std::string filepath) {
        _name = name;
        _initial_hover_throttle = initial_hover_throttle;
        _min_throttle = min_throttle;
        _full_bat_and_throttle_spinup_duration = full_bat_and_throttle_spinup_duration;
        _blink_period = blink_period;
        _max_thrust = max_thrust;
        _radius = radius;
        _pad_radius = pad_radius;
        _drone_blink_strength = drone_blink_strength;
        _drone_led_strength = drone_led_strength;
        _tx = tx;
        _mode3d = mode3d;
        _spinup_throttle_non3d = spinup_throttle_non3d;
        _land_cell_v = land_cell_v;
        _max_flight_time = max_flight_time;
        _min_hunt_cell_v = min_hunt_cell_v;
        _static_shakeit_thrust = static_shakeit_thrust;
        _target_takeoff_velocity = target_takeoff_velocity;
        _led_type = led_type;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream(filepath);
        outfile << xmlData ;
        outfile.close();
    }
};

class CamCalibration: public xmls::Serializable {
private:
    xmls::xFloat _angle_x;
    xmls::xFloat _angle_y;
    xmls::xInt _exposure;
    xmls::xFloat _gain;
    xmls::xFloat _brightness;
    xmls::xInt _width;
    xmls::xInt _height;
    xmls::xFloat _ppx;
    xmls::xFloat _ppy;
    xmls::xFloat _fx;
    xmls::xFloat _fy;
    xmls::xInt _model;
    xmls::xFloat _coeff0;
    xmls::xFloat _coeff1;
    xmls::xFloat _coeff2;
    xmls::xFloat _coeff3;
    xmls::xFloat _coeff4;
    xmls::xFloat _depth_scale;
    xmls::xFloat _baseline;

public:
    float camera_angle_x = 0;
    float camera_angle_y = 30;
    int measured_exposure = 15400;
    int measured_gain = 0;
    int measured_brightness = 0;
    //rs2_intrinsics
    int width;
    int height;
    float ppx;
    float ppy;
    float fx;
    float fy;
    int model;
    float coeffs[5];
    //extrensics:
    float baseline;
    float depth_scale;

    CamCalibration() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("CamCalibrationData");

        // Set class version
        setVersion("1.4");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("Angle_X", &_angle_x);
        Register("Angle_Y", &_angle_y);
        Register("Exposure", &_exposure);
        Register("Gain", &_gain);
        Register("Brightness", &_brightness);
        Register("Width", &_width);
        Register("Height", &_height);
        Register("ppx", &_ppx);
        Register("ppy", &_ppy);
        Register("fx", &_fx);
        Register("fy", &_fy);
        Register("Model", &_model);
        Register("Coeff0", &_coeff0);
        Register("Coeff1", &_coeff1);
        Register("Coeff2", &_coeff2);
        Register("Coeff3", &_coeff3);
        Register("Coeff4", &_coeff4);
        Register("Depth_Scale", &_depth_scale);
        Register("Baseline", &_baseline);
    }

    void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw std::runtime_error("Cannot read: " + filepath);
            }
            CamCalibration tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw std::runtime_error("XML version difference detected from " + filepath);
            }
            infile.close();
        } else {
            throw std::runtime_error("File not found: " + filepath);
        }

        camera_angle_x = _angle_x.value();
        camera_angle_y = _angle_y.value();
        measured_exposure = _exposure.value();
        measured_gain = _gain.value();
        measured_brightness = _brightness.value();
        width = _width.value();
        height = _height.value();
        ppx = _ppx.value();
        ppy = _ppy.value();
        fx = _fx.value();
        fy = _fy.value();
        model = _model.value();
        coeffs[0] = _coeff0.value();
        coeffs[1] = _coeff1.value();
        coeffs[2] = _coeff2.value();
        coeffs[3] = _coeff3.value();
        coeffs[4] = _coeff4.value();
        depth_scale = _depth_scale.value();
        baseline = _baseline.value();
    }

    void serialize(std::string filepath) {
        _angle_x = camera_angle_x;
        _angle_y = camera_angle_y;
        _exposure = measured_exposure;
        _gain = measured_gain;
        _brightness = measured_brightness;
        _width = width;
        _height = height;
        _ppx = ppx;
        _ppy = ppy;
        _fx = fx;
        _fy = fy;
        _model = model;
        _coeff0 = coeffs[0];
        _coeff1 = coeffs[1];
        _coeff2 = coeffs[2];
        _coeff3 = coeffs[3];
        _coeff4 = coeffs[4];
        _depth_scale = depth_scale;
        _baseline = baseline;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream(filepath);
        outfile << xmlData ;
        outfile.close();
    }
};

class DroneCalibration: public Serializable {
public: string pad_calib_date = "";
public: float pad_pos_x = 0;
public: float pad_pos_y = 0;
public: float pad_pos_z = 0;
public: float pad_roll = 0;
public: float pad_pitch = 0;
public: string drone_name = "";
public: int drone_id = -1;
public: float max_thrust = -1;
public: string thrust_calib_date = "";
private: xmls::xString _pad_calib_date;
private: xmls::xFloat _pad_pos_x;
private: xmls::xFloat _pad_pos_y;
private: xmls::xFloat _pad_pos_z;
private: xmls::xFloat _pad_roll;
private: xmls::xFloat _pad_pitch;
private: xmls::xInt _drone_id;
private: xmls::xString _drone_name;
private: xmls::xFloat _max_thrust;
private: xmls::xString _thrust_calib_date;

public: cv::Point3f pad_pos() { return cv::Point3f(pad_pos_x, pad_pos_y, pad_pos_z);}

public: DroneCalibration() {
        setClassName("DroneCalibration");
        setVersion("1.3");
        Register("drone_id", &_drone_id);
        Register("drone_name", &_drone_name);
        Register("pad_calib_date", &_pad_calib_date);
        Register("pad_pos_x", &_pad_pos_x);
        Register("pad_pos_y", &_pad_pos_y);
        Register("pad_pos_z", &_pad_pos_z);
        Register("pad_roll", &_pad_roll);
        Register("pad_pitch", &_pad_pitch);
        Register("max_thrust", &_max_thrust);
        Register("thrust_calib_date", &_thrust_calib_date);
    }

public: void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw std::runtime_error("Cannot read: " + filepath);
            }
            DroneCalibration tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw std::runtime_error("XML version difference detected from " + filepath);
            }
        } else {
            throw std::runtime_error("File not found: " + filepath);
        }

        pad_calib_date = _pad_calib_date.value();
        pad_pos_x = _pad_pos_x.value();
        pad_pos_y = _pad_pos_y.value();
        pad_pos_z = _pad_pos_z.value();
        pad_roll = _pad_roll.value();
        pad_pitch = _pad_pitch.value();
        drone_id = _drone_id.value();
        drone_name = _drone_name.value();
        max_thrust = _max_thrust.value();
        thrust_calib_date = _thrust_calib_date.value();
    }

public: void serialize(std::string filepath) {
        _pad_calib_date = pad_calib_date;
        _pad_pos_x = pad_pos_x;
        _pad_pos_y = pad_pos_y;
        _pad_pos_z = pad_pos_z;
        _pad_roll = pad_roll;
        _pad_pitch = pad_pitch;
        _drone_name = drone_name;
        _drone_id = drone_id;
        _max_thrust = max_thrust;
        _thrust_calib_date = thrust_calib_date;

        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream(filepath);
        outfile << xmlData ;
        outfile.close();
    }
};

}
extern xmls::DroneParameters dparams;
extern xmls::PatsParameters pparams;


#define EXECUTOR_PACKAGE_PRE_HEADER '@'
enum executor_package_headers {
    header_SocketExecutorStatePackage = 's'
};



enum executor_states {
    es_daemon_disabled = 0,
    es_starting,
    es_hardware_check,
    es_init,
    es_init_vision,
    es_realsense_init,
    es_realsense_reset,
    es_realsense_not_found,
    es_wait_for_light_level,
    es_wait_for_cam_angle,
    es_wait_for_enable_window,
    es_locate_drone,
    es_pats_c,
    es_pats_x,
    es_closing,
    es_periodic_restart,
    es_watchdog_restart,
    es_user_restart,
    es_light_level_restart,
    es_enable_window_restart,
    es_drone_version_mismatch,
    es_drone_config_restart,
    es_realsense_fps_problem,
    es_realsense_frame_loss_problem,
    es_rc_problem,
    es_baseboard_problem,
    es_daemon_problen, // to be implemented
    es_realsense_error,
    es_xml_config_problem, // to be implemented
    es_runtime_error
};

enum drone_issues {
    drone_issues_no_drone,
    drone_issues_ok,
    drone_issues_telemetry_time_out,
    drone_issues_locate_time_out,
    drone_issues_crashed,
};
struct __attribute__((packed)) SocketExecutorStatePackage {
    const char pre_header = EXECUTOR_PACKAGE_PRE_HEADER;
    const char header = header_SocketExecutorStatePackage;
    uint8_t executor_state;
    uint8_t drone_issue;
    float light_level;
    double time;
    const char ender = '\n';
};
extern void communicate_state(executor_states s);
