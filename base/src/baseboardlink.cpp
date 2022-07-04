#include "baseboardlink.h"
#include <unistd.h> //usleep
#include<iostream>

void BaseboardLink::init(bool replay_mode) {
    _replay_mode = replay_mode;

    if (file_exist(disable_flag) && !_replay_mode) {
        _disabled = true;
        _charging_state = state_disabled;
        std::cout << "BaseboardLink DISABLED" << std::endl;
    }

    if (!_replay_mode && !_disabled) {
        deamon_address.sun_family = AF_UNIX;
        strncpy(deamon_address.sun_path, "/home/pats/pats/sockets/baseboard2executor.socket", sizeof(deamon_address.sun_path) - 1);
        if ((sock = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
            throw std::runtime_error("Could not connect to baseboard link: socket failed.");
            exit(1);
        }
        auto res = connect(sock, reinterpret_cast< sockaddr *>(&deamon_address), sizeof(deamon_address));
        if (res < 0) {
            close(sock);
            std::cout << "Socket connect error code: " << res <<  std::endl;
            throw std::runtime_error("Could not connect to baseboard link.");
            exit(1);
        }

        // can be tested with running "socket -s /home/pats/pats/sockets/baseboard2executor.socket"
        std::cout << "Connected to baseboard socket" << std::endl;
        thread_receive = std::thread(&BaseboardLink::worker_receive, this);
        thread_send = std::thread(&BaseboardLink::worker_send, this);
        initialized = true;
        allow_charging(true);
    }
}
void BaseboardLink::init_logger() {
    std::string logger_fn = data_output_dir  + "log_baseboard.csv";
    baseboard_logger.open(logger_fn, std::ofstream::out);
    baseboard_logger << "time" <<
                     ";" << "version" <<
                     ";" << "ir_led_state" <<
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
                     ";" << "charging_duration" << std::endl;
    logger_initialized = true;
}

void BaseboardLink::worker_receive() {
    char buffer[sizeof(SerialBaseboard2NUCPackage)] = {0};
    SerialBaseboard2NUCPackage ori_pkg;
    while (!exit_thread) {
        usleep(5e5);
        auto valread = read(sock, buffer, sizeof(SerialBaseboard2NUCPackage));
        if (valread > 0)
            read_timeouts = 0;
        if (valread == sizeof(SerialBaseboard2NUCPackage)) {
            if (buffer[0] == ori_pkg.pre_header && buffer[sizeof(SerialBaseboard2NUCPackage) - 1] == ori_pkg.ender) {
                if (buffer[3] == ori_pkg.header) {
                    SerialBaseboard2NUCPackage *pkg = reinterpret_cast<SerialBaseboard2NUCPackage * >(&buffer);
                    if (pkg->firmware_version != required_firmware_version_baseboard) {
                        std::cout << "BaseboardLink firmware version problem. Detected: " << pkg->firmware_version << ", required: " << ori_pkg.firmware_version << std::endl;
                        close(sock);
                        throw std::runtime_error("BaseboardLink error");
                        exit(1);
                    }

                    _uptime = static_cast<float>(pkg->up_duration) / 1000.f;
                    _charging_state = static_cast<charging_states>(pkg->charging_state);
                    _charging_duration = pkg->charging_duration / 1000.f;
                    _bat_voltage = pkg->battery_volts;
                    baseboard_logger << _time <<
                                     ";" << pkg->firmware_version <<
                                     ";" << static_cast<uint16_t>(pkg->ir_led_state) <<
                                     ";" << pkg->up_duration <<
                                     ";" << static_cast<uint16_t>(pkg->charging_state) <<
                                     ";" << pkg->battery_volts <<
                                     ";" << pkg->charging_volts <<
                                     ";" << pkg->charging_amps <<
                                     ";" << pkg->setpoint_amps <<
                                     ";" << pkg->mah_charged <<
                                     ";" << pkg->charge_resistance <<
                                     ";" << pkg->drone_amps_burn <<
                                     ";" << static_cast<uint16_t>(pkg->charging_pwm) <<
                                     ";" << pkg->charging_duration << "\n";
                } else {
                    std::cout << "Warning: unknown baseboard package header received: " << static_cast<uint16_t>(buffer[2]) << std::endl;
                }
            } else {
                std::cout << "Warning: re-aligning baseboard socket" << std::endl;
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
                std::cout << "BaseboardLink comm receive failed" << std::endl;
                _exit_now = true;
                close(sock);
                return;
            }
        }
    }
}

void BaseboardLink::send_over_socket(unsigned char *data, ssize_t len) {
    if (_exit_now)
        return;
    auto ret = send(sock, data, len, MSG_NOSIGNAL);
    if (ret != len) {
        std::cout << "Error: BaseboardLink comm send failed " << ret << "   " <<  len << std::endl;
        _exit_now = true;
        close(sock);
    }
}

void BaseboardLink::worker_send() {
    std::unique_lock<std::mutex> lk(cv_m_to_baseboard);
    while (!exit_thread) {
        auto timeout = cv_to_baseboard.wait_until(lk, std::chrono::system_clock::now() + 1000ms);
        if (update_allow_charging_pkg_to_baseboard || timeout ==  std::cv_status::timeout) {
            send_over_socket(reinterpret_cast<unsigned char *>(&allow_charging_pkg_to_baseboard), sizeof(SerialExecutor2BaseboardAllowChargingPackage));
            update_allow_charging_pkg_to_baseboard = false;
        }
        if (update_executor_state_pkg_to_baseboard || timeout ==  std::cv_status::timeout) {
            send_over_socket(reinterpret_cast<unsigned char *>(&executor_state_pkg_to_baseboard), sizeof(SocketExecutorStatePackage));
            update_executor_state_pkg_to_baseboard  = false;
        }
    }
}

void BaseboardLink::close_link() {

    exit_thread = true;
    if (initialized) {
        std::cout << "Closing baseboard socket" << std::endl;
        thread_receive.join();
        thread_send.join();
        close(sock);
        if (logger_initialized) {
            baseboard_logger.flush();
            baseboard_logger.close();
        }

    }
}
