#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "rc.h"
#include "airsim.h"

class AirSimController : public Rc {
private:
    AirSim sim;
    int airsim_arm_state = bf_disarmed;
    const std::string drone_name = "Hammer";

    void send_thread(void);

public:
    void init(int  __attribute__((unused)));
    bool connect();
    void close();
    void bind(bool __attribute__((unused))) {}
    void init_logger();
    void send_rc_data(void);
    void queue_commands(int new_throttle, int new_roll, int new_pitch, int new_yaw);
    float normalize_rc_input(float in_value, float lower_bound = -1, float upper_bound = 1);
    int drone_id() { return 0; }
};
