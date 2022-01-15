#include "baseboard.h"
#include <unistd.h> //usleep
#include<iostream>

void Baseboard::init(bool replay_mode) {
    _replay_mode = replay_mode;

    if (file_exist(disable_flag) && !_replay_mode) {
        _disabled = true;
        _charging_state = state_disabled;
    }

    if (!_replay_mode && !_disabled) {
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
void Baseboard::init_logger() {
    std::string logger_fn = data_output_dir  + "log_baseboard.csv";
    baseboard_logger.open(logger_fn, std::ofstream::out);
    baseboard_logger << "time" <<
                     ";" << "version" <<
                     ";" << "led_state" <<
                     ";" << "watchdog_state" <<
                     ";" << "up_duration" <<
                     ";" << "charging_state" <<
                     ";" << "battery_volts" <<
                     ";" << "charging_volts" <<
                     ";" << "charging_amps" <<
                     ";" << "setpoint_amp" <<
                     ";" << "mah_charged" <<
                     ";" << "charge_resistance" <<
                     ";" << "drone_amps_burn" <<
                     ";" << "charging_pwm" <<
                     ";" << "charging_duration" <<
                     ";" << "fan_speed" << std::endl;
    logger_initialized = true;
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
                _bat_voltage = pkg->battery_volts;
                baseboard_logger << _time <<
                                 ";" << pkg->version <<
                                 ";" << static_cast<uint16_t>(pkg->led_state) <<
                                 ";" << static_cast<uint16_t>(pkg->watchdog_state) <<
                                 ";" << pkg->up_duration <<
                                 ";" << static_cast<uint16_t>(pkg->charging_state) <<
                                 ";" << pkg->battery_volts <<
                                 ";" << pkg->charging_volts <<
                                 ";" << pkg->charging_amps <<
                                 ";" << pkg->setpoint_amp <<
                                 ";" << pkg->mah_charged <<
                                 ";" << pkg->charge_resistance <<
                                 ";" << pkg->drone_amps_burn <<
                                 ";" << static_cast<uint16_t>(pkg->charging_pwm) <<
                                 ";" << pkg->charging_duration <<
                                 ";" << pkg->fan_speed << "\n";
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
                _exit_now = true;
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
            std::cout << "Error: Baseboard comm send failed" << std::endl;
            _exit_now = true;
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
    if (logger_initialized) {
        baseboard_logger.flush();
        baseboard_logger.close();
    }
}
