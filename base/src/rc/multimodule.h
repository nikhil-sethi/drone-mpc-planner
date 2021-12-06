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

class MultiModule : public RC {

public:
    void init(int drone_id);
    bool connect();
    void init_logger();
    void close();
    void bind(bool b);
    int drone_id() {return _drone_id_rxnum;}

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
    void watchdog_pats_init_package();
};
