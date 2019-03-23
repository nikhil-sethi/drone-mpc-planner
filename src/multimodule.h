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

#define JOY_BOUND_MIN                       228   // 1050
#define JOY_BOUND_MAX                       1820 // 1950
#define JOY_MIN_THRESH                      300   // 1080
#define JOY_MAX_THRESH                      1750 // 1800
#define JOY_MIN                             0   //  1000
#define JOY_MAX                             2048 // 2000
#define JOY_MIDDLE                          1024 // 1500

#if TX_TYPE == TX_NONE
#define TX_PROTOCOL 0
#define TX_SUB_PROTOCOL 0
#define TX_OPTION 0
#endif

#if TX_TYPE == TX_DSMX
#define TX_PROTOCOL 6
#define TX_SUB_PROTOCOL 3
#define TX_OPTION 7
#endif

#if TX_TYPE == TX_CX10
#define TX_PROTOCOL 12
#define TX_SUB_PROTOCOL 1
#define TX_RATE 2000
#endif

#if TX_TYPE == TX_FRSKYD
#define TX_PROTOCOL 3
#define TX_SUB_PROTOCOL 0
#endif

#if TX_TYPE == TX_FRSKYX
#define TX_PROTOCOL 15
#define TX_SUB_PROTOCOL 0
#endif


#ifndef TX_RATE
#define TX_RATE 0
#endif

#ifndef TX_OPTION
#define TX_OPTION 0
#endif

static const char* armed_names[] = {"disarmed","armed"};

class MultiModule{

public:

    void init(bool fromfile);

    void close();

    int ledpower = 75;
    uint16_t mode = JOY_MAX; // <min = mode 1, 1500 = mode 2, >max = mode 3
    int roll=JOY_MIDDLE,pitch=JOY_MIDDLE,yaw=JOY_MIDDLE;
    int throttle = JOY_BOUND_MIN;
    int arm_switch = JOY_MIN_THRESH;

    std::string Armed(){
        return armed_names[arm_switch>JOY_MIDDLE];
    }

    std::mutex g_lockData;

    void check_bind_command(void);
    bool _bind = false;
    stopwatch_c sw_bind;
    void bind(bool b) {
        if (b){
        sw_bind.Restart();
        _bind = true;
        } else {
            _bind = false;
        }
    }
    void arm(bool v) {
        if (v)
            arm_switch = JOY_BOUND_MAX;
        else
            arm_switch = JOY_BOUND_MIN;
    }
private:

    stopwatch_c binding_sw;

    bool initialised = false;
    int notconnected;
    enum bound_enum{
        cx10_unknown,
        cx10_bound,
        cx10_not_bound,
        cx10_binding
    };

    bound_enum bound = cx10_unknown;

    std::stringstream received;

    std::mutex lock_rs232;

    std::thread thread_mm;
    bool exitSendThread = false;
    void worker_thread(void);
    void send_data(void);
    void receive_data(void);
    void convert_channels(uint16_t *channels, unsigned char * packet);

};

#endif // MULTIMODULE_H
