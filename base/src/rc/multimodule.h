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
#include "rc.h"

#define RC_BOUND_MIN                       224   // 1000
#define RC_BOUND_MAX                       1824 // 2000
#define RC_BOUND_RANGE    (RC_BOUND_MAX -  RC_BOUND_MIN)
#define RC_MIN_THRESH                      300   // 1048
#define RC_MAX_THRESH                      1750 // 1800
#define RC_MIN                             0   //  1000
#define RC_MAX                             2048 // 2000
#define RC_MIDDLE                          1024 // 1500

class MultiModule : public RC {

public:
    void init(int drone_id);
    bool connect();
    void init_logger();
    void close();
    void bind(bool b);
    int drone_id() {return _drone_id_rxnum;}

    // value_betaflight = value_here + 1000
    int bf_headless_enabled() { return 0;}
    int bf_headless_disabled() { return 31;}
    int bf_yaw_reset() { return 63;}
    int bf_PID_loop_disabled() { return 94;}
    int bf_spin_motor() { return 125;}
    int bf_spin_motor_reversed() { return 156;}
    int bf_airmode() { return 187;}
    int bf_sleep() { return  255;}

private:
    int protocol;
    int sub_protocol;
    int tx_option;
    int tx_rate;
    int _drone_id_tx = 3; // 3 is the hardcoded default in the MM at the moment for D16 (--> MProtocol_id_master = 3; )
    int _drone_id_rxnum = 0;
    uint init_package_nOK_cnt = 1;

    bool mm_version_check_OK = false;
    bool send_init_package_now = false;
    bool _bind = false;

    float batt_v_accepted_max = 10.f;
    float batt_cell_v_accepted_max = 10.f;
    float roll_accepted_max = 180.f;
    float pitch_accepted_max = 180.f;

    void send_thread(void);
    void receive_thread(void);
    void send_rc_data(void);
    void receive_data(void);
    void convert_channels(uint16_t *channels, unsigned char *packet);
    void zerothrottle();
    void send_pats_init_package();
    bool receive_telemetry(std::string buffer);
    void process_pats_init_packages(std::string bufs);
    void handle_bind();
    void watchdog_tx_connect();
    int map(int inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange);
};