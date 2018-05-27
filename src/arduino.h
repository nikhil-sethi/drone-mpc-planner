#ifndef ARDUINO_H
#define ARDUINO_H

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

class Arduino{

public:

    void init(bool fromfile);
    void rebind(void);
    void close();

    int ledpower = 10;
    uint16_t mode = 1500; // <min = mode 1, 1500 = mode 2, >max = mode 3
    int roll,pitch,yaw = 1500;
    int throttle = 1000;
    int rebindValue = 0;

    std::mutex g_lockData;

private:

    int baudrate;

    std::thread thread_nrf;
    bool exitSendThread = false;
    void workerThread(void);
    void sendData(void);

};

#endif // ARDUINO_H
