#include "airsimcontroller.h"

bool AirSimController::connect() {
    if (port_unopened) {
        std::cout << "Connecting AirSimController" << std::endl;
        sim.init(drone_name);
        port_unopened = 0;
    }
    return !port_unopened;
}

void AirSimController::init(int __attribute__((unused))) {
    send_thread_tx = std::thread(&AirSimController::send_thread, this);
    sim.rc_data_valid(true);
    initialized = true;
    sim.set_led(5000);
}

void AirSimController::send_thread(void) {
    std::cout << "AirSimController send thread ready!" << std::endl;
    g_sendData.lock();
    std::cout << "AirSimController send thread started!" << std::endl;
    while (!exitSendThread) {
        send_rc_data();
    }
}

void AirSimController::init_logger() {
    logger_initialized = true;
}

float AirSimController::normalize_rc_input(float in_value, float lower_bound /*=-1*/, float upper_bound /*=1*/) {
    float x = (in_value - BF_CHN_MIN);
    return lower_bound + (upper_bound - lower_bound) * (x / BF_CHN_RANGE);
}

void AirSimController::send_rc_data() {
    g_sendData.lock();
    g_lockData.lock();;
    if (dparams.tx != tx_none) {
        if (calibrate_acc_cnt) {
            calibrate_acc_cnt--;
            roll = BF_CHN_MID;
            pitch = BF_CHN_MIN;
            yaw = BF_CHN_MIN;
            throttle = BF_CHN_MAX;
        }

        if (calibrate_acc_cnt)
            led = 5000;
        else
            // turn off the led below 5
            led = (_LED_drone > 5) ? _LED_drone * 50 : 0;
        sim.set_led(led);

        // set the correct arming state in airsim
        if (arm_switch != airsim_arm_state) {
            sim.arm(arm_switch == bf_armed);
            airsim_arm_state = arm_switch;
        }

        sim.move_by_rc(normalize_rc_input(throttle, 0, 1), normalize_rc_input(yaw), normalize_rc_input(pitch), normalize_rc_input(roll));
    }
    g_lockData.unlock();
}

void AirSimController::close() {
    if (initialized) {
        std::cout << "Closing AirSimController" << std::endl;
        exitSendThread = true;
        g_sendData.unlock();
        g_lockData.unlock();
        send_thread_tx.join();
        usleep(1e5);
        exitReceiveThread = true;

        // kill throttle when closing the module
        g_lockData.lock();
        mode = BF_CHN_MIN;
        arm_switch = BF_CHN_MIN;
        throttle = BF_CHN_MIN;
        roll = BF_CHN_MID;
        pitch = BF_CHN_MID;
        yaw = BF_CHN_MID;
        _LED_drone = 0;

        g_sendData.unlock();
        g_lockData.unlock();
        send_rc_data();
        port_unopened = 1;
    }
    initialized = false;
}