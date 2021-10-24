#include "baseboard.h"
#include <unistd.h> //usleep
#include<iostream>

void Baseboard::init(bool replay_mode) {
    _replay_mode = replay_mode;
    if (!_replay_mode) {

        deamon_address.sun_family = AF_UNIX;
        strncpy(deamon_address.sun_path, "/home/pats/pats/sockets/baseboard2pats.socket", sizeof(deamon_address.sun_path) - 1);
        if ((sock = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
            throw std::runtime_error("Could not connect to baseboard daemon: socket failed.");
            exit(1);
        } else  if (connect(sock, reinterpret_cast< sockaddr *>(&deamon_address), sizeof(deamon_address)) < 0) {
            throw std::runtime_error("Could not connect to baseboard daemon");
            exit(1);
        }

        // can be tested with running "socket -s /home/pats/pats/sockets/baseboard2pats.socket"
        std::cout << "Connected to baseboard socket" << std::endl;
        thread_receive = std::thread(&Baseboard::worker_receive, this);
        thread_send = std::thread(&Baseboard::worker_send, this);
        initialized = true;
    }
}

void Baseboard::worker_receive() {
    char buffer[sizeof(SerialPackage)] = {0};
    SerialPackage ori_pkg;
    while (!exit_thread) {
        usleep(5e5);
        auto valread = read(sock, buffer, sizeof(SerialPackage));
        if (valread > 0)
            read_timeouts = 0;
        if (valread == sizeof(SerialPackage)) {
            if (buffer[0] == ori_pkg.header && buffer[sizeof(SerialPackage) - 1] == ori_pkg.ender) {
                SerialPackage *pkg = reinterpret_cast<SerialPackage * >(&buffer);
                if (pkg->version != required_firmware_version_baseboard) {
                    std::cout << "Baseboard firmware version problem. Detected: " << pkg->version << ", required: " << ori_pkg.version << std::endl;
                    throw std::runtime_error("Baseboard error");
                    abort();
                }

                _uptime = static_cast<float>(pkg->up_duration) / 1000.f;
                _charging_state = static_cast<ChargingState>(pkg->charging_state);
            } else { // allign
                std::cout << "Warning: realligning baseboard socket" << std::endl;
                while (true) {
                    ssize_t dummy [[maybe_unused]]  = read(sock, buffer, 1);
                    if (buffer[0] == ori_pkg.ender) {
                        break;
                    }
                }
            }
        } else {
            read_timeouts++;
            if (read_timeouts > 6) {
                exit_thread = true;
                std::cout << "Baseboard comm receive failed" << std::endl;
                throw std::runtime_error("Baseboard comm receive failed");
                abort();
            }
        }
    }
}

void Baseboard::worker_send() {
    std::string msg = "Harrow from Pats!\n";
    while (!exit_thread) {
        ssize_t ret = send(sock, msg.c_str(), msg.length(), MSG_NOSIGNAL);
        if (ret != static_cast<ssize_t>(msg.length())) {
            std::cout << "Baseboard comm send failed" << std::endl;
            throw std::runtime_error("Baseboard comm send failed");
            abort();
        }
        usleep(1e6);
    }
}

void Baseboard::close() {
    std::cout << "Closing baseboard socket" << std::endl;
    exit_thread = true;
    if (initialized) {
        thread_receive.join();
        thread_send.join();
    }
}
