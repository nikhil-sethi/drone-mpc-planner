#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "third_party/tinyxml/XMLSerialization.h"
#include <iostream>
#include <fstream>
#include <mutex>

#define IMG_W 848
#define IMG_H 480
#define IMG_Wf 848.f
#define IMG_Hf 480.f
#define GRAVITY 9.81f

int sign(float x);
vector
<string> split_csv_line(string);
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
void set_external_wdt_flag();
void set_no_realsense_flag();
void set_realsense_buf_overflow_flag();
std::string exec(const char *cmd);
void set_fps_warning_flag();
void set_frame_loss_warning_flag();

const float rad2deg = 180.f / M_PIf32;
const float deg2rad = M_PIf32 / 180.f;

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
    xBool _watchdog, _has_screen;
    xVideo_mode _video_raw, _video_result;
    xRC_type _joystick;
    xDrone_type _drone;
    xOp_mode _op_mode;
    xInt _wdt_timeout_us, _fps, _close_after_n_images;
    xInt _exposure_threshold, _gain_threshold, _brightness_threshold;
    xString _plukker_start;
    xFloat _min_hunt_size, _max_hunt_size;
    xInt _imscalef;
    xString _location;
    xString _flightplan;
    xString _flightplan_calib_thrust;
    xInt _live_image_frq;
    xFloat _max_cam_roll;
    xInt _n_cams;
    xBool _long_range_mode;

public:
    int exposure_threshold, gain_threshold, brightness_threshold;
    int wdt_timeout_us, close_after_n_images;
    uint fps;
    bool watchdog, has_screen;
    video_modes video_raw, video_result;
    rc_types joystick;
    drone_types drone;
    op_modes op_mode;
    std::string plukker_start;
    float min_hunt_size, max_hunt_size;
    int imscalef;
    std::string location;
    std::string flightplan;
    std::string flightplan_calib_thrust;
    int live_image_frq;
    float max_cam_roll;
    int n_cams;
    bool long_range_mode;

    PatsParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("PatsParameters");

        // Set class version
        setVersion("1.15");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("wdt_timeout_us", &_wdt_timeout_us);
        Register("exposure_threshold", &_exposure_threshold);
        Register("gain_threshold", &_gain_threshold);
        Register("brightness_threshold", &_brightness_threshold);
        Register("close_after_n_images", &_close_after_n_images);
        Register("plukker_start", &_plukker_start);
        Register("min_hunt_size", &_min_hunt_size);
        Register("max_hunt_size", &_max_hunt_size);
        Register("has_screen", &_has_screen);
        Register("op_mode", &_op_mode);
        Register("watchdog", &_watchdog);
        Register("fps", &_fps);
        Register("video_raw", &_video_raw);
        Register("video_result", &_video_result);
        Register("joystick", &_joystick);
        Register("drone", &_drone);
        Register("imscalef", &_imscalef);
        Register("location", &_location);
        Register("flightplan", &_flightplan);
        Register("flightplan_calib_thrust", &_flightplan_calib_thrust);
        Register("live_image_frq", &_live_image_frq);
        Register("max_cam_roll", &_max_cam_roll);
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

        wdt_timeout_us = _wdt_timeout_us.value();
        exposure_threshold = _exposure_threshold.value();
        gain_threshold = _gain_threshold.value();
        brightness_threshold = _brightness_threshold.value();
        close_after_n_images = _close_after_n_images.value();
        plukker_start = _plukker_start.value();
        min_hunt_size = _min_hunt_size.value();
        max_hunt_size = _max_hunt_size.value();
        has_screen = _has_screen.value();
        op_mode = _op_mode.value();
        watchdog = _watchdog.value();
        fps = _fps.value();
        video_raw = _video_raw.value();
        video_result = _video_result.value();
        joystick = _joystick.value();
        drone = _drone.value();
        imscalef = _imscalef.value();
        location = _location.value();
        flightplan = _flightplan.value();
        flightplan_calib_thrust = _flightplan_calib_thrust.value();
        live_image_frq = _live_image_frq.value();
        max_cam_roll = _max_cam_roll.value();
        n_cams = _n_cams.value();
        long_range_mode = _long_range_mode.value();
    }

    void serialize(std::string settings_file) {
        _wdt_timeout_us = wdt_timeout_us;
        _exposure_threshold = exposure_threshold;
        _gain_threshold = gain_threshold;
        _brightness_threshold = brightness_threshold;
        _close_after_n_images = close_after_n_images;
        _plukker_start = plukker_start;
        _max_hunt_size = max_hunt_size;
        _min_hunt_size = min_hunt_size;
        _has_screen = has_screen;
        _op_mode = op_mode;
        _watchdog = watchdog;
        _fps = fps;
        _video_raw = video_raw;
        _video_result = video_result;
        _joystick = joystick;
        _drone = drone;
        _imscalef = imscalef;
        _location = location;
        _flightplan = flightplan;
        _flightplan_calib_thrust = flightplan_calib_thrust;
        _live_image_frq = live_image_frq;
        _max_cam_roll = max_cam_roll;
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
    xFloat _throttle_bank_factor;
    xFloat _max_burn_time;
    xInt _min_throttle;
    xFloat _full_bat_and_throttle_im_effect;
    xFloat _full_bat_and_throttle_take_off_acc;
    xFloat _full_bat_and_throttle_spinup_duration;
    xFloat _hover_throttle_a;
    xFloat _hover_throttle_b;
    xFloat _blink_period;
    xFloat _radius;
    xFloat _pad_radius;
    xFloat _default_thrust;
    xInt _drone_blink_strength;
    xInt _drone_led_strength;
    xTx_protocol _tx;
    xBool _mode3d;
    xInt _spinup_throttle_non3d;
    xFloat _land_cell_v;
    xInt _max_flight_time;
    xFloat _min_hunt_cell_v;
    xInt _static_shakeit_throttle;
    xFloat _target_takeoff_velocity;
    xLed_types _led_type;

    xInt _kp_pos_roll, _kp_pos_pitch, _kp_pos_throttle;
    xInt _ki_pos_roll, _ki_pos_pitch, _ki_thrust;
    xInt _kd_pos_roll, _kd_pos_pitch, _kd_pos_throttle;
    xInt _kp_pos_roll_hover, _kp_pos_pitch_hover, _kp_pos_throttle_hover;
    xInt _ki_pos_roll_hover, _ki_pos_pitch_hover, _ki_thrust_hover;
    xInt _kd_pos_roll_hover, _kd_pos_pitch_hover, _kd_pos_throttle_hover;
    xInt _kp_v_roll, _kp_v_pitch, _kp_v_throttle;
    xInt _kd_v_roll, _kd_v_pitch, _kd_v_throttle;

public:
    std::string name;
    int initial_hover_throttle;
    float throttle_bank_factor;
    double max_burn_time;
    int min_throttle;
    float full_bat_and_throttle_im_effect; // how many pixels per second will the drone go up given full throttle
    float full_bat_and_throttle_take_off_acc;
    float full_bat_and_throttle_spinup_duration;
    float hover_throttle_a;
    float hover_throttle_b;
    float blink_period;
    float radius;
    float pad_radius;
    float default_thrust;
    int drone_blink_strength;
    int drone_led_strength;
    tx_protocols tx;
    bool mode3d;
    int spinup_throttle_non3d;
    float land_cell_v;
    int max_flight_time;
    float min_hunt_cell_v;
    int static_shakeit_throttle;
    float target_takeoff_velocity;
    led_types led_type;
    bool Telemetry() { return tx == tx_frskyd16; }

    int kp_pos_roll, kp_pos_throttle, kp_pos_pitch, ki_pos_roll, ki_thrust, ki_pos_pitch, kd_pos_roll, kd_pos_throttle, kd_pos_pitch;
    int kp_pos_roll_hover, kp_pos_throttle_hover, kp_pos_pitch_hover, ki_pos_roll_hover, ki_thrust_hover, ki_pos_pitch_hover, kd_pos_roll_hover, kd_pos_throttle_hover, kd_pos_pitch_hover;
    int kp_v_roll, kp_v_throttle, kp_v_pitch, kd_v_roll, kd_v_throttle, kd_v_pitch;

    DroneParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("DroneParameters");

        // Set class version
        setVersion("1.13");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("name", &_name);
        Register("initial_hover_throttle", &_initial_hover_throttle);
        Register("throttle_bank_factor", &_throttle_bank_factor);
        Register("max_burn_time", &_max_burn_time);
        Register("min_throttle", &_min_throttle);
        Register("full_bat_and_throttle_im_effect", &_full_bat_and_throttle_im_effect);
        Register("full_bat_and_throttle_take_off_acc", &_full_bat_and_throttle_take_off_acc);
        Register("full_bat_and_throttle_spinup_time", &_full_bat_and_throttle_spinup_duration);
        Register("hover_throttle_a", &_hover_throttle_a);
        Register("hover_throttle_b", &_hover_throttle_b);
        Register("blink_period", &_blink_period);
        Register("radius", &_radius);
        Register("pad_radius", &_pad_radius);
        Register("thrust", &_default_thrust);
        Register("drone_blink_strength", &_drone_blink_strength);
        Register("drone_led_strength", &_drone_led_strength);
        Register("tx", &_tx);
        Register("mode3d", &_mode3d);
        Register("spinup_throttle_non3d", &_spinup_throttle_non3d);
        Register("land_cell_v", &_land_cell_v);
        Register("max_flight_time", &_max_flight_time);
        Register("min_hunt_cell_v", &_min_hunt_cell_v);
        Register("static_shakeit_throttle", &_static_shakeit_throttle);
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

        Register("kp_v_roll", &_kp_v_roll);
        Register("kp_v_pitch", &_kp_v_pitch);
        Register("kp_v_throttle", &_kp_v_throttle);
        Register("kd_v_roll", &_kd_v_roll);
        Register("kd_v_pitch", &_kd_v_pitch);
        Register("kd_v_throttle", &_kd_v_throttle);


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
        throttle_bank_factor = _throttle_bank_factor.value();
        max_burn_time = _max_burn_time.value();
        min_throttle = _min_throttle.value();
        full_bat_and_throttle_im_effect = _full_bat_and_throttle_im_effect.value();
        full_bat_and_throttle_take_off_acc = _full_bat_and_throttle_take_off_acc.value();
        full_bat_and_throttle_spinup_duration = _full_bat_and_throttle_spinup_duration.value();
        hover_throttle_a = _hover_throttle_a.value();
        hover_throttle_b = _hover_throttle_b.value();
        blink_period = _blink_period.value();
        default_thrust = _default_thrust.value();
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
        static_shakeit_throttle = _static_shakeit_throttle.value();
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

        kp_v_roll = _kp_v_roll.value();
        kp_v_pitch = _kp_v_pitch.value();
        kp_v_throttle = _kp_v_throttle.value();
        kd_v_roll = _kd_v_roll.value();
        kd_v_pitch = _kd_v_pitch.value();
        kd_v_throttle = _kd_v_throttle.value();
    }

    void serialize(std::string filepath) {
        _name = name;
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
        _default_thrust = default_thrust;
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
        _static_shakeit_throttle = static_shakeit_throttle;
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
public: float thrust = -1;
public: string thrust_calib_date = "";
private: xmls::xString _pad_calib_date;
private: xmls::xFloat _pad_pos_x;
private: xmls::xFloat _pad_pos_y;
private: xmls::xFloat _pad_pos_z;
private: xmls::xFloat _pad_roll;
private: xmls::xFloat _pad_pitch;
private: xmls::xInt _drone_id;
private: xmls::xString _drone_name;
private: xmls::xFloat _thrust;
private: xmls::xString _thrust_calib_date;

public: cv::Point3f pad_pos() { return cv::Point3f(pad_pos_x, pad_pos_y, pad_pos_z);}

public: DroneCalibration() {
        setClassName("DroneCalibration");
        setVersion("1.2");
        Register("drone_id", &_drone_id);
        Register("drone_name", &_drone_name);
        Register("pad_calib_date", &_pad_calib_date);
        Register("pad_pos_x", &_pad_pos_x);
        Register("pad_pos_y", &_pad_pos_y);
        Register("pad_pos_z", &_pad_pos_z);
        Register("pad_roll", &_pad_roll);
        Register("pad_pitch", &_pad_pitch);
        Register("thrust", &_thrust);
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
        thrust = _thrust.value();
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
        _thrust = thrust;
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
