#ifndef MULTIMODULE_H
#define MULTIMODULE_H

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

#define JOY_BOUND_MIN                       224   // 1000
#define JOY_BOUND_MAX                       1824 // 2000
#define JOY_BOUND_RANGE    (JOY_BOUND_MAX -  JOY_BOUND_MIN)
#define JOY_MIN_THRESH                      300   // 1048
#define JOY_MAX_THRESH                      1750 // 1800
#define JOY_MIN                             0   //  1000
#define JOY_MAX                             2048 // 2000
#define JOY_MIDDLE                          1024 // 1500


enum betaflight_mode {
    bf_angle = JOY_BOUND_MIN,
    bf_acro =  (JOY_MIDDLE + JOY_BOUND_MIN)
};
enum betaflight_headless_mode {
    bf_headless_enabled = 0,
    bf_headless_disabled = 50 // semi random number that gives a nice detectable change in the mode switch (1030 in BF), but is small enough to not interfere with the normal modes (can be made smaller prolly if necessary)
};

enum betaflight_arming {
    bf_disarmed = JOY_BOUND_MIN,
    bf_armed = JOY_BOUND_MAX
};



static const char* armed_names[] = {"disarmed","armed"};

class MultiModule{
    int protocol;
    int sub_protocol;
    int tx_option;
    int tx_rate;
public:


    void init(int drone_id, bool fromfile);

    void close();

    void LED(bool value);

    bool led_on = true;

    int ledpower = 75;
    uint16_t mode = JOY_BOUND_MIN; // set to angle mode in BF
    int roll=JOY_MIDDLE,pitch=JOY_MIDDLE,yaw=JOY_MIDDLE;
    int throttle = JOY_BOUND_MIN;
    int arm_switch = JOY_BOUND_MIN;

    std::string Armed(){
        return armed_names[arm_switch>JOY_MIDDLE];
    }

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

    void check_bind_command(void);
    bool _bind = false;

    // counter used to make sure throttle and arming is safe for binding.
    //(it has happened that the drone takes off uncontrolled when the bind channel was activated)
    //So, 5 cycles before and after binding, the arm is set to false and throttle to 0.
    int cycles_until_bind = 0;
    stopwatch_c sw_bind; // stop binding in max 20s
    void bind(bool b) {
        if (b){
            sw_bind.Restart();
            if (cycles_until_bind == 0)
                cycles_until_bind = 80;
        }
        if (!b){
            if (cycles_until_bind == 0)
                cycles_until_bind = -80;
        }
    }
    void arm(betaflight_arming v) {
        arm_switch = v;
    }
private:

    stopwatch_c binding_sw;

    std::mutex g_lockData;
    std::mutex g_sendData;

    bool send_init_package_now = false;
    int _drone_id = 1;
    uint init_package_nOK_cnt = 1;

    bool initialized = false;
    bool version_check_OK = false;
    int notconnected;

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

};

#endif // MULTIMODULE_H
