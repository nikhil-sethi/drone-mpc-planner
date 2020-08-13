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
#define RXBUFFER_SIZE 36

#define JOY_BOUND_MIN                       224   // 1000
#define JOY_BOUND_MAX                       1824 // 2000
#define JOY_BOUND_RANGE    (JOY_BOUND_MAX -  JOY_BOUND_MIN)
#define JOY_MIN_THRESH                      300   // 1048
#define JOY_MAX_THRESH                      1750 // 1800
#define JOY_MIN                             0   //  1000
#define JOY_MAX                             2048 // 2000
#define JOY_MIDDLE                          1024 // 1500

#define THROTTLE_MIN                        1000
#define THROTTLE_MAX                        2000
#define THROTTLE_OFFSET                     1000
#define THROTTLE_HOVER                      1180

enum betaflight_mode {
    bf_angle = JOY_BOUND_MIN,
    bf_acro =  (JOY_MIDDLE + JOY_BOUND_MIN)
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
    bf_disarmed = JOY_BOUND_MIN,
    bf_armed = JOY_BOUND_MAX
};
enum betaflight_turtle {
    bf_turtle_disabled = JOY_BOUND_MIN,
    bf_turtle_enabled = JOY_BOUND_MAX
};

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky_hub.h
// accn, maxthrust, acc throttle mix, acc rpm mix, throttle and armining are additional messages
enum
{
    FSSP_DATAID_VFAS       = 0x0210,
    FSSP_DATAID_CURRENT    = 0x0200,
    FSSP_DATAID_RPM        = 0x0500,
    FSSP_DATAID_RSSI       = 0xF101,
    FSSP_DATAID_ACCX       = 0x0700,
    FSSP_DATAID_ACCY       = 0x0710,
    FSSP_DATAID_ACCZ       = 0x0720,
    FSSP_DATAID_ACCN       = 0x0730,  // Acc nominal
    FSSP_DATAID_MAX_THRUST = 0x0740,
    FSSP_DATAID_ACC_THROTTLE_MIX    = 0x0750,
    FSSP_DATAID_ACC_RPM_MIX         = 0x0760,
    FSSP_DATAID_THROTTLE   = 0x0120,
    FSSP_DATAID_ARMING     = 0x0130,
    FSSP_DATAID_A4         = 0x0910
};

enum
{
    X   =   0,
    Y   =   1,
    Z   =   2,
    N   =   3
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
    ARMING_DISABLED_NO_GYRO         = 0,
    ARMING_DISABLED_FAILSAFE        = 1,
    ARMING_DISABLED_RX_FAILSAFE     = 2,
    ARMING_DISABLED_BAD_RX_RECOVERY = 3,
    ARMING_DISABLED_BOXFAILSAFE     = 4,
    ARMING_DISABLED_RUNAWAY_TAKEOFF = 5,
    ARMING_DISABLED_CRASH_DETECTED  = 6,
    ARMING_DISABLED_THROTTLE        = 7,
    ARMING_DISABLED_ANGLE           = 8,
    ARMING_DISABLED_BOOT_GRACE_TIME = 9,
    ARMING_DISABLED_NOPREARM        = 10,
    ARMING_DISABLED_LOAD            = 11,
    ARMING_DISABLED_CALIBRATING     = 12,
    ARMING_DISABLED_CLI             = 13,
    ARMING_DISABLED_CMS_MENU        = 14,
    ARMING_DISABLED_BST             = 15,
    ARMING_DISABLED_MSP             = 16,
    ARMING_DISABLED_PARALYZE        = 17,
    ARMING_DISABLED_GPS             = 18,
    ARMING_DISABLED_RESC            = 19,
    ARMING_DISABLED_RPMFILTER       = 20,
    ARMING_DISABLED_REBOOT_REQUIRED = 21,
    ARMING_DISABLED_DSHOT_BITBANG   = 22,
    ARMING_DISABLED_ARM_SWITCH      = 23 // Needs to be the last element, since it's always activated if one of the others is active when arming
};

struct sensorvalue
{
    /* data */
    float       batt_v;
    float       batt_cell_v;
    float       batt_current;
    uint8_t     rssi;
    uint16_t    rpm;
    float       acc[4];             // X,Y,Z,Nominal
    float       thrust_max = 4;     // 4G as start value
    uint16_t    throttle;
    float       throttle_scaled;
    arming_states arming_state;
    float       thrust_rpm;
};
class MultiModule {

public:
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

    uint16_t mode = JOY_BOUND_MIN; // set to angle mode in BF
    int roll=JOY_MIDDLE,pitch=JOY_MIDDLE,yaw=JOY_MIDDLE;
    int throttle = JOY_BOUND_MIN;
    int arm_switch = JOY_BOUND_MIN;
    int turtle_mode = JOY_BOUND_MIN;
    sensorvalue sensor;

    std::string Armed() { return armed_names[arm_switch>JOY_MIDDLE]; }

    void queue_commands(int new_throttle,int new_roll, int new_pitch, int new_yaw, int new_mode) {
        g_lockData.lock();
        throttle = new_throttle;
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
        mode = new_mode;
        g_lockData.unlock();
        g_sendData.unlock();
    }

    bool init_package_failure() { return _init_package_failure;}

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
    void beep() { _beep = !_beep; }
    void arm(betaflight_arming v) { arm_switch = v; }
    void turtle(betaflight_turtle v) { turtle_mode = v; }
    int calibrate_acc_cnt = 0;
    void calibrate_acc() { calibrate_acc_cnt = 200; }
    bool connected() {return !notconnected;}
private:

    int protocol;
    int sub_protocol;
    int tx_option;
    int tx_rate;

    int cycles_until_bind = 0;
    stopwatch_c sw_bind; // stop binding in max 20s

    std::mutex g_lockData;
    std::mutex g_sendData;

    bool _init_package_failure = false;
    bool send_init_package_now = false;
    int _drone_id_tx = 3; // 3 is the hardcoded default in the MM at the moment for D16 (--> MProtocol_id_master = 3; )
    int _drone_id_rxnum = 0;
    uint init_package_nOK_cnt = 1;
    bool _beep = false;
    int _LED_drone = 5;

    bool initialized = false;
    bool version_check_OK = false;
    int notconnected;
    bool _bind = false;

    std::stringstream received;

    std::mutex lock_rs232;

    std::thread thread_mm;
    bool exitSendThread = false;
    void worker_thread(void);
    void send_data(void);
    void receive_data(void);
    void convert_channels(uint16_t *channels, unsigned char * packet);
    void zerothrottle();
    void send_init_package();
    void acc_throttle_pkg(uint32_t data);
    void acc_rpm_pkg(uint32_t data);
    void process_telem(uint16_t sensor_id, float data);
    void receive_sensor(std::string buffer);
};
