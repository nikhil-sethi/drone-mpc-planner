#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <unistd.h>
#include "common.h"

#include "rs232.h"
#include <thread>
#include <mutex>
#include "stopwatch.h"

#define MULTI_BINDBIT                       0x80
#define MULTI_AUTOBINDBIT                   0x40
#define MULTI_RANGECHECK                    0x20
#define MULTI_CHANS                         16
#define MULTI_CHAN_BITS                     11
#define RXBUFFER_SIZE                       36

#define RC_BOUND_MIN                       224   // 1000
#define RC_BOUND_MAX                       1824 // 2000
#define RC_BOUND_RANGE    (RC_BOUND_MAX -  RC_BOUND_MIN)
#define RC_MIN_THRESH                      300   // 1048
#define RC_MAX_THRESH                      1750 // 1800
#define RC_MIN                             0   //  1000
#define RC_MAX                             2048 // 2000
#define RC_MIDDLE                          1024 // 1500

#define BF_CHN_MIN                         1000
#define BF_CHN_MAX                         2000
#define BF_CHN_OFFSET                      1000


enum betaflight_mode {
    bf_angle = RC_BOUND_MIN,
    bf_acro = (RC_MIDDLE + RC_BOUND_MIN)
};
enum betaflight_headless_mode { // value_betaflight = 1000/1600 * value_here + 1000
                                // value_here = 1600/1000 * (value_betaflight - 1000)
    bf_headless_enabled = 0,
    bf_headless_disabled = 50, // semi random number that gives a nice detectable change in the mode switch (1030 in BF), but is small enough to not interfere with the normal modes (can be made smaller prolly if necessary)
    bf_yaw_reset = 100,
    bf_PID_loop_disabled = 150,
    bf_spin_motor = 200,
    bf_spin_motor_reversed = 250,
    bf_airmode = 300
};
static const char *armed_names[] = {"rc_disarmed", "rc_armed"};
enum betaflight_arming {
    bf_disarmed = RC_BOUND_MIN,
    bf_armed = RC_BOUND_MAX
};
enum betaflight_turtle {
    bf_turtle_disabled = RC_BOUND_MIN,
    bf_turtle_enabled = RC_BOUND_MAX
};

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky_hub.h
// accn, maxthrust, acc throttle mix, acc rpm mix, throttle and armining are additional messages
enum {
    FSSP_DATAID_VFAS       = 0x0210,
    FSSP_DATAID_CURRENT    = 0x0200,
    FSSP_DATAID_RSSI       = 0xF101,
    FSSP_DATAID_PITCH      = 0x5230,
    FSSP_DATAID_ROLL       = 0x5240,
    FSSP_DATAID_ACCX       = 0x0700,
    FSSP_DATAID_ACCY       = 0x0710,
    FSSP_DATAID_ACCZ       = 0x0720,

    FSSP_DATAID_A4         = 0x0910, // cell voltage

    //pats specifics:
    FSSP_DATAID_BF_VERSION              = 0x0743,
    FSSP_DATAID_BF_UID                  = 0x0744,
    FSSP_DATAID_ARMING                  = 0x0745,
    FSSP_DATAID_ROLL_PITCH              = 0x0746

};

enum arming_states {
    ARMING_DISABLED_NO_GYRO         = (1 << 0),
    ARMING_DISABLED_FAILSAFE        = (1 << 1),
    ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
    ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
    ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
    ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
    ARMING_DISABLED_CRASH_DETECTED  = (1 << 6),
    ARMING_DISABLED_THROTTLE        = (1 << 7),
    ARMING_DISABLED_ANGLE           = (1 << 8),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
    ARMING_DISABLED_NOPREARM        = (1 << 10),
    ARMING_DISABLED_LOAD            = (1 << 11),
    ARMING_DISABLED_CALIBRATING     = (1 << 12),
    ARMING_DISABLED_CLI             = (1 << 13),
    ARMING_DISABLED_CMS_MENU        = (1 << 14),
    ARMING_DISABLED_BST             = (1 << 15),
    ARMING_DISABLED_MSP             = (1 << 16),
    ARMING_DISABLED_PARALYZE        = (1 << 17),
    ARMING_DISABLED_GPS             = (1 << 18),
    ARMING_DISABLED_RESC            = (1 << 19),
    ARMING_DISABLED_RPMFILTER       = (1 << 20),
    ARMING_DISABLED_REBOOT_REQUIRED = (1 << 21),
    ARMING_DISABLED_DSHOT_BITBANG   = (1 << 22),
    ARMING_DISABLED_ACC_CALIBRATION = (1 << 23),
    ARMING_DISABLED_MOTOR_PROTOCOL  = (1 << 24),
    ARMING_DISABLED_ARM_SWITCH      = (1 << 25)
};

struct Telemetry {
    float           batt_v;
    float           batt_cell_v;
    uint32_t        batt_cell_v_package_id;
    float           batt_current;
    uint8_t         rssi;
    double          rssi_last_received;
    float           roll;
    float           pitch;
    uint32_t        roll_pitch_package_id;
    cv::Point3f     acc;
    arming_states   arming_state;
    uint32_t        arming_state_package_id;
    int             bf_major;
    int             bf_minor;
    int             bf_patch;
    std::string     bf_uid_str;
};
class RC {

public:
    uint16_t mode = RC_BOUND_MIN; // set to angle mode in BF
    int roll = RC_MIDDLE, pitch = RC_MIDDLE, yaw = RC_MIDDLE;
    int throttle = RC_BOUND_MIN;
    int arm_switch = RC_BOUND_MIN;
    int turtle_mode = RC_BOUND_MIN;
    Telemetry telemetry = {0};

    virtual void init(int drone_id) = 0;
    virtual void init_logger() = 0;
    virtual int drone_id() = 0;
    virtual void close() = 0;
    virtual bool connect() = 0;

    int LED_drone() {return _LED_drone;}
    void LED_drone(bool on, int strength_value) {
        if (on)
            _LED_drone = strength_value;
        else
            _LED_drone = 0;
    }
    void LED_drone(int value) { _LED_drone = value; } // led strength value between 0-100, where 0 is off off

    bool init_package_failure() { return _init_package_failure;}
    bool bf_version_error() { return _bf_version_error > 10;}
    bool bf_uid_error() { return _bf_uid_error > 10;}
    bool bf_telem_OK() { return _bf_uid_error < 0 && _bf_version_error < 0;}
    bool telemetry_time_out() {return _time - last_telemetry_time > 10;}
    std::string bf_uid_str() {return telemetry.bf_uid_str;}
    std::string armed_str() { return armed_names[arm_switch > RC_MIDDLE]; }
    bool arm_command() { return arm_switch == bf_armed; }
    bool connected() {return !notconnected;}

    void queue_commands(int new_throttle, int new_roll, int new_pitch, int new_yaw, int new_mode) {
        if (!exitSendThread) {
            g_lockData.lock();
            throttle = new_throttle;
            roll = new_roll;
            pitch = new_pitch;
            yaw = new_yaw;
            mode = new_mode;
            g_lockData.unlock();
        }
    }
    void send_commands(double time) {
        if (!exitSendThread) {
            _time = time;
            g_sendData.unlock();
        }
    }

    // counter used to make sure throttle and arming is safe for binding.
    //(it has happened that the drone takes off uncontrolled when the bind channel was activated)
    //So, 5 cycles before and after binding, the arm is set to false and throttle to 0.
    void bind(bool b) {
        if (b) {
            sw_bind.Restart();
            if (cycles_until_bind == 0)
                cycles_until_bind = 80;
        }
        if (!b) {
            if (cycles_until_bind == 0)
                cycles_until_bind = -80;
        }
    }
    void beep(bool b) { _beep = b; }
    void beep() { _beep = !_beep;}
    void arm(betaflight_arming v) {
        arm_switch = v;
        if (_time_disarmed < 0 and v == bf_disarmed)
            _time_disarmed = _time;
        else if (_time_disarmed >= 0 and v == bf_armed)
            _time_disarmed = -1;
    }
    void turtle(betaflight_turtle v) { turtle_mode = v; }
    void calibrate_acc() { calibrate_acc_cnt = 200; }
    double time_disarmed() { return _time_disarmed;}

    bool ok(double time) {
        if (dparams.Telemetry() && connected()) {
            return (telemetry.rssi > 40 && time - telemetry.rssi_last_received < 1); // TODO tune rssi
        } else
            return connected();
    }

    std::string arming_state_str() {
        std::string res = "";
        if (telemetry.arming_state & ARMING_DISABLED_NO_GYRO)
            res += "ARMING_DISABLED_NO_GYRO | ";
        if (telemetry.arming_state & ARMING_DISABLED_FAILSAFE)
            res += "ARMING_DISABLED_FAILSAFE | ";
        if (telemetry.arming_state & ARMING_DISABLED_RX_FAILSAFE)
            res += "ARMING_DISABLED_RX_FAILSAFE | ";
        if (telemetry.arming_state & ARMING_DISABLED_BAD_RX_RECOVERY)
            res += "ARMING_DISABLED_BAD_RX_RECOVERY | ";
        if (telemetry.arming_state & ARMING_DISABLED_BOXFAILSAFE)
            res += "ARMING_DISABLED_BOXFAILSAFE | ";
        if (telemetry.arming_state & ARMING_DISABLED_RUNAWAY_TAKEOFF)
            res += "ARMING_DISABLED_RUNAWAY_TAKEOFF | ";
        if (telemetry.arming_state & ARMING_DISABLED_CRASH_DETECTED)
            res += "ARMING_DISABLED_CRASH_DETECTED | ";
        if (telemetry.arming_state & ARMING_DISABLED_THROTTLE)
            res += "ARMING_DISABLED_THROTTLE | ";
        if (telemetry.arming_state & ARMING_DISABLED_ANGLE)
            res += "ARMING_DISABLED_ANGLE | ";
        if (telemetry.arming_state & ARMING_DISABLED_BOOT_GRACE_TIME)
            res += "ARMING_DISABLED_BOOT_GRACE_TIME | ";
        if (telemetry.arming_state & ARMING_DISABLED_NOPREARM)
            res += "ARMING_DISABLED_NOPREARM | ";
        if (telemetry.arming_state & ARMING_DISABLED_LOAD)
            res += "ARMING_DISABLED_LOAD | ";
        if (telemetry.arming_state & ARMING_DISABLED_CALIBRATING)
            res += "ARMING_DISABLED_CALIBRATING | ";
        if (telemetry.arming_state & ARMING_DISABLED_CLI)
            res += "ARMING_DISABLED_CLI | ";
        if (telemetry.arming_state & ARMING_DISABLED_CMS_MENU)
            res += "ARMING_DISABLED_CMS_MENU | ";
        if (telemetry.arming_state & ARMING_DISABLED_BST)
            res += "ARMING_DISABLED_BST | ";
        if (telemetry.arming_state & ARMING_DISABLED_MSP)
            res += "ARMING_DISABLED_MSP | ";
        if (telemetry.arming_state & ARMING_DISABLED_PARALYZE)
            res += "ARMING_DISABLED_PARALYZE | ";
        if (telemetry.arming_state & ARMING_DISABLED_GPS)
            res += "ARMING_DISABLED_GPS | ";
        if (telemetry.arming_state & ARMING_DISABLED_RESC)
            res += "ARMING_DISABLED_RESC | ";
        if (telemetry.arming_state & ARMING_DISABLED_RPMFILTER)
            res += "ARMING_DISABLED_RPMFILTER | ";
        if (telemetry.arming_state & ARMING_DISABLED_REBOOT_REQUIRED)
            res += "ARMING_DISABLED_REBOOT_REQUIRED | ";
        if (telemetry.arming_state & ARMING_DISABLED_DSHOT_BITBANG)
            res += "ARMING_DISABLED_DSHOT_BITBANG | ";
        if (telemetry.arming_state & ARMING_DISABLED_ACC_CALIBRATION)
            res += "ARMING_DISABLED_ACC_CALIBRATION | ";
        if (telemetry.arming_state & ARMING_DISABLED_ARM_SWITCH)
            res += "ARMING_DISABLED_ARM_SWITCH ";
        return res;
    }

protected:
    double _time = 0;
    double _time_disarmed = -1;
    double last_telemetry_time = 0;

    bool initialized = false;
    bool logger_initialized = false;
    int notconnected = 1;
    bool _init_package_failure = false;
    int16_t _bf_version_error = 0;
    int16_t _bf_uid_error = 0;
    bool mm_version_check_OK = false;
    uint init_package_nOK_cnt = 1;
    bool send_init_package_now = false;

    bool _beep = false;
    int _LED_drone = 5;

    bool _bind = false;
    int cycles_until_bind = 0;
    stopwatch_c sw_bind; // stop binding in max 20s

    int calibrate_acc_cnt = 0;

    std::mutex g_lockData;
    std::timed_mutex g_sendData;
    std::stringstream received;
    std::thread send_thread_mm;
    std::thread receive_thread_mm;
    bool exitSendThread = false;
    bool exitReceiveThread = false;

    std::ofstream telem_logger;

    void send_thread(void);
    void receive_thread(void);
    void send_rc_data(void);
    void receive_data(void);
    void convert_channels(uint16_t *channels, unsigned char *packet);
    void zerothrottle();
    void send_pats_init_package();
    void acc_throttle_pkg(int16_t accz, int16_t thr);
    void acc_rpm_pkg(int16_t accz, int16_t rpm);
    bool receive_telemetry(std::string buffer);
    void process_pats_init_packages(std::string bufs);
    void handle_bind();
    void watchdog_pats_init_package();

};
