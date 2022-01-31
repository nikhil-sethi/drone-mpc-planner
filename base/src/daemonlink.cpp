#include "daemonlink.h"
#include <unistd.h> //usleep
#include<iostream>

void DaemonLink::init(bool replay_mode) {
    _replay_mode = replay_mode;

    if (file_exist(daemon_disable_flag) && !_replay_mode) {
        _disabled = true;
        executor_state_pkg_to_daemon.executor_state = es_daemon_disabled;
        std::cout << "Daemon link DISABLED" << std::endl;
    }

    if (!_replay_mode && !_disabled) {
        deamon_address.sun_family = AF_UNIX;
        strncpy(deamon_address.sun_path, "/home/pats/pats/sockets/executor2daemon.socket", sizeof(deamon_address.sun_path) - 1);
        if ((sock = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
            throw std::runtime_error("Could not connect daemon link: socket failed.");
            exit(1);
        } else  if (connect(sock, reinterpret_cast< sockaddr *>(&deamon_address), sizeof(deamon_address)) < 0) {
            throw std::runtime_error("Could not connect daemon link");
            exit(1);
        }

        std::cout << "Connected daemon link" << std::endl;
        thread_receive = std::thread(&DaemonLink::worker_receive, this);
        thread_send = std::thread(&DaemonLink::worker_send, this);
        initialized = true;
    }
}

void DaemonLink::worker_receive() {

}

void DaemonLink::send_over_socket(unsigned char *data, ssize_t len) {
    auto ret = send(sock, data, len, MSG_NOSIGNAL);
    if (ret != len) {
        std::cout << "Error: Daemon comm send failed " << ret << "   " <<  len << std::endl;
        _exit_now = true;
        abort();
    }
}

void DaemonLink::worker_send() {
    std::unique_lock<std::mutex> lk(cv_m_to_daemon);
    while (!exit_thread) {
        cv_to_daemon.wait_until(lk, std::chrono::system_clock::now() + 1000ms);
        if (!exit_thread) {
            try {
                send_over_socket(reinterpret_cast<unsigned char *>(&executor_state_pkg_to_daemon), sizeof(SocketExecutorStatePackage));
            } catch (exception &exp) {
                throw std::runtime_error("Err: " + string(exp.what()));
            }
        }
    }
}

void DaemonLink::close() {
    exit_thread = true;
    if (initialized) {
        std::cout << "Closing daemon socket" << std::endl;
        thread_receive.join();
        thread_send.join();
    }
}