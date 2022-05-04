#pragma once
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <condition_variable>
#include "versions.h"
#include "common.h"
using namespace std::chrono_literals;


class DaemonLink {

private:

    bool _disabled = false;
    bool _replay_mode;
    bool _exit_now = false;
    std::thread thread_send;
    std::thread thread_receive;
    bool initialized = false;
    bool exit_thread = false;
    int read_timeouts = 0;
    const std::string daemon_disable_flag = "/home/pats/pats/flags/disable_daemonlink";

    int sock;
    sockaddr_un deamon_address;

    std::condition_variable cv_to_daemon;
    std::mutex cv_m_to_daemon;
    SocketExecutorStatePackage executor_state_pkg_to_daemon;

    void worker_receive();
    void worker_send();
    void send_over_socket(unsigned char *data, ssize_t len);

public:

    void init(bool replay_mode);
    void init_logger();
    void close_link();

    void time(double time) {executor_state_pkg_to_daemon.time = time;}
    void executor_state(executor_states s) {
        executor_state_pkg_to_daemon.executor_state = s;
        cv_to_daemon.notify_one();
    }

    bool disabled() {return _disabled;}
    bool exit_now() { return _exit_now;}
};
