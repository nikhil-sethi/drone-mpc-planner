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
    bf_acro =  (RC_MIDDLE + RC_BOUND_MIN)
};
enum betaflight_headless_mode {
    bf_headless_enabled = 0,
    bf_headless_disabled = 50, // semi random number that gives a nice detectable change in the mode switch (1030 in BF), but is small enough to not interfere with the normal modes (can be made smaller prolly if necessary)
    bf_yaw_reset = 100,
    bf_PID_loop_disabled = 150,
    bf_spin_motor = 200,
    bf_spin_motor_reversed = 250
};
static const char* armed_names[] = {"disarmed","armed"};
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
    FSSP_DATAID_RPM        = 0x0500,
    FSSP_DATAID_RSSI       = 0xF101,
    FSSP_DATAID_PITCH      = 0x5230,
    FSSP_DATAID_ROLL       = 0x5240,
    FSSP_DATAID_ACCX       = 0x0700,
    FSSP_DATAID_ACCY       = 0x0710,
    FSSP_DATAID_ACCZ       = 0x0720,

    FSSP_DATAID_A4         = 0x0910, // cell voltage

    //pats specifics:
    FSSP_DATAID_MAX_THRUST              = 0x0740,
    FSSP_DATAID_ACC_THROTTLE_MIX        = 0x0741, // acceleration on z axis and throttle in same pkg
    FSSP_DATAID_ACC_RPM_MIX             = 0x0742, // acceleration on z axis and throttle in same pkg
    FSSP_DATAID_BF_VERSION              = 0x0743,
    FSSP_DATAID_ARMING                  = 0x0745

};

// static const char* arming_states_str[] = {
//     "ARMING_DISABLED_NO_GYRO",
//     "ARMING_DISABLED_FAILSAFE",
//     "ARMING_DISABLED_RX_FAILSAFE",
//     "ARMING_DISABLED_BAD_RX_RECOVERY",
//     "ARMING_DISABLED_BOXFAILSAFE",
//     "ARMING_DISABLED_RUNAWAY_TAKEOFF",
//     "ARMING_DISABLED_CRASH_DETECTED",
//     "ARMING_DISABLED_THROTTLE",
//     "ARMING_DISABLED_ANGLE",
//     "ARMING_DISABLED_BOOT_GRACE_TIME",
//     "ARMING_DISABLED_NOPREARM",
//     "ARMING_DISABLED_LOAD",
//     "ARMING_DISABLED_CALIBRATING",
//     "ARMING_DISABLED_CLI",
//     "ARMING_DISABLED_CMS_MENU",
//     "ARMING_DISABLED_BST",
//     "ARMING_DISABLED_MSP",
//     "ARMING_DISABLED_PARALYZE",
//     "ARMING_DISABLED_GPS",
//     "ARMING_DISABLED_RESC",
//     "ARMING_DISABLED_RPMFILTER",
//     "ARMING_DISABLED_REBOOT_REQUIRED",
//     "ARMING_DISABLED_DSHOT_BITBANG",
//     "ARMING_DISABLED_ARM_SWITCH",
//     "ARMING_DISABLED_ALL_GOOD"
// };
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

struct telemetry_data {
    float           batt_v;
    float           batt_cell_v;
    float           batt_current;
    uint8_t         rssi;
    uint16_t        rpm;
    float           roll;
    float           pitch;
    cv::Point3f     acc;
    float           thrust_max = 4;     // 4G as start value
    uint16_t        throttle;
    float           throttle_scaled;
    arming_states   arming_state;
    float           thrust_rpm;
    int             bf_major;
    int             bf_minor;
    int             bf_patch;
};
class MultiModule {

public:
    uint16_t mode = RC_BOUND_MIN; // set to angle mode in BF
    int roll=RC_MIDDLE,pitch=RC_MIDDLE,yaw=RC_MIDDLE;
    int throttle = RC_BOUND_MIN;
    int arm_switch = RC_BOUND_MIN;
    int turtle_mode = RC_BOUND_MIN;
    telemetry_data telemetry;
    const int bf_major_required = 4;
    const int bf_minor_required = 2;
    const int bf_patch_required = 102;


    void init(int drone_id);
    void close();

    int LED_drone() {return _LED_drone;}
    void LED_drone(bool on, int strength_value) {
        if (on)
            _LED_drone = strength_value;
        else
            _LED_drone = 0;
    }
    void LED_drone(int value) { _LED_drone = value; } // led strength value between 0-100, where 0 is off off

    bool init_package_failure() { return _init_package_failure;}
    bool bf_version_error() { return _bf_version_error>10;}
    std::string Armed() { return armed_names[arm_switch>RC_MIDDLE]; }

    void queue_commands(int new_throttle,int new_roll, int new_pitch, int new_yaw, int new_mode) {
        if (!exitSendThread) {
            g_lockData.lock();
            throttle = new_throttle;
            roll = new_roll;
            pitch = new_pitch;
            yaw = new_yaw;
            mode = new_mode;
            g_lockData.unlock();
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
    void arm(betaflight_arming v) { arm_switch = v; }
    void turtle(betaflight_turtle v) { turtle_mode = v; }
    void calibrate_acc() { calibrate_acc_cnt = 200; }
    bool connected() {return !notconnected;}

private:
    int protocol;
    int sub_protocol;
    int tx_option;
    int tx_rate;
    int _drone_id_tx = 3; // 3 is the hardcoded default in the MM at the moment for D16 (--> MProtocol_id_master = 3; )
    int _drone_id_rxnum = 0;

    bool initialized = false;
    int notconnected = 1;
    bool _init_package_failure = false;
    uint16_t _bf_version_error = 0;
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
    std::mutex g_sendData;
    std::stringstream received;
    std::thread send_thread_mm;
    std::thread receive_thread_mm;
    bool exitSendThread = false;
    bool exitReceiveThread = false;

    void send_thread(void);
    void receive_thread(void);
    void send_rc_data(void);
    void receive_data(void);
    void convert_channels(uint16_t *channels, unsigned char * packet);
    void zerothrottle();
    void send_pats_init_package();
    void acc_throttle_pkg(int16_t accz, int16_t thr);
    void acc_rpm_pkg(int16_t accz, int16_t rpm);
    bool receive_telemetry(std::string buffer);
    void process_pats_init_packages(std::string bufs);
    void handle_bind();
    void watchdog_pats_init_package();
};
