#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include "common.h"

#include "rs232.h"
#include <thread>
#include <mutex>
#include "stopwatch.h"

class Arduino{

public:

    void init(bool fromfile);

    void close();

    int ledpower = 75;
    uint16_t mode = 1900; // <min = mode 1, 1500 = mode 2, >max = mode 3
    int roll=1500,pitch=1500,yaw = 1500;
    int throttle = 1000;


    std::mutex g_lockData;

    void check_bind_command(void);
    void bind() {
        bind_next_cycle = true;
    }
private:

    int notconnected;
    stopwatch_c binding_sw;
    int baudrate;
    bool bind_next_cycle = false;
    enum bound_enum{
        cx10_unknown,
        cx10_bound,
        cx10_not_bound,
        cx10_binding
    };

    bound_enum bound = cx10_unknown;

    std::stringstream received;

    std::mutex lock_rs232;

    std::thread thread_nrf;
    bool exitSendThread = false;
    void workerThread(void);
    void sendData(void);

};

#endif // ARDUINO_H
