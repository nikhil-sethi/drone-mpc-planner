#include "multimodule.h"
#include "versions.h"

bool MultiModule::connect() {
    std::cout << "Connecting multimodule" << std::endl;
    if (port_unopened) {
        switch (dparams.tx) {
            case tx_elrs:
                [[fallthrough]];
            case tx_none: {
                    protocol = 0;
                    sub_protocol = 0;
                    tx_option = 0;
                    tx_rate = 0;
                    break;
            } case tx_dsmx: {
                    protocol = 6;
                    sub_protocol = 3;
                    tx_option = 7;
                    tx_rate = 0;
                    break;
            } case tx_cx10: {
                    protocol = 12;
                    sub_protocol = 1;
                    tx_rate = 2000;
                    tx_option = 0;
                    break;
            } case tx_frskyd8: {
                    protocol = 3;
                    sub_protocol = 0;
                    tx_rate = 0;
                    tx_option = 0;
                    break;
            } case tx_frskyd16: {
                    protocol = 15;
                    sub_protocol = d16_ch8; // d16_eu8 seems to be generate buggy telemetry #431
                    tx_rate = 0;
                    tx_option = 0;
                    break;
            } case tx_redpine: {
                    protocol = 50;
                    sub_protocol = redpine_fast;
                    tx_rate = 0;
                    tx_option = 0;
                    break;
                }
        }

        zerothrottle();

        if (dparams.tx != tx_none) {
            port_unopened = RS232_OpenComport(115200, "/dev/pats_mm0");
            if (port_unopened)
                port_unopened = RS232_OpenComport(115200, "/dev/pats_mm1");

            if (port_unopened)
                std::cout << "Could not open multimodule port."  << std::endl;
            else
                std::cout << "Opened multimodule port."  << std::endl;
        }
    }
    return !port_unopened;
}

void MultiModule::init(int drone_id) {

    if (dparams.tx == tx_frskyd8 || drone_id > 50)
        drone_id %= 50; //TMP HACK

    _drone_id_rxnum = drone_id;

    if (!port_unopened) {
        send_init_package_now = true;
        receive_thread_tx = std::thread(&MultiModule::receive_thread, this);
        send_thread_tx = std::thread(&MultiModule::send_thread, this);
        initialized = true;
    }
}
void MultiModule::init_logger() {
    std::string logger_fn = data_output_dir  + "log_telemetry.txt";
    telem_logger.open(logger_fn, std::ofstream::out);
    logger_initialized = true;
}

void MultiModule::send_pats_init_package() {
    //send bind id to the multimodule. It must have this form: [66,67 mode3d,id2,id1,id0,68], otherwise the MM won't proceed.
    unsigned char packet[7];
    packet[0] = 66; //header 1
    packet[1] = 67; //header 2
    packet[2] = dparams.mode3d;
    packet[3] = 0;  //id2...
    packet[4] = 0; // id1
    packet[5] = _drone_id_tx; //id0
    packet[6] = 68; //header 3
    usleep(100);
    RS232_SendBuf(static_cast<unsigned char *>(packet), 7);
    usleep(100);
    send_init_package_now = false;
}

void MultiModule::zerothrottle() {
    if (dparams.mode3d)
        throttle = RC_MIDDLE;
    else
        throttle = RC_BOUND_MIN;
}

void MultiModule::send_thread(void) {
    usleep(100);
    std::cout << "Multimodule send thread ready!" << std::endl;
    g_sendData.lock();
    g_sendData.lock(); // wait for first frame to arrive to prevent triggering a wdt in the MM
    std::cout << "Multimodule send thread started!" << std::endl;
    while (!exitSendThread) {
        watchdog_tx_connect();
        if (send_init_package_now)
            send_pats_init_package();
        else {
            handle_bind();
            send_rc_data();
        }
    }
}

void MultiModule::receive_thread(void) {
    std::cout << "Multimodule receive thread started!" << std::endl;
    while (!exitReceiveThread) {
        receive_data();
        usleep(100);
    }
}

void MultiModule::watchdog_tx_connect() {
    if (init_package_nOK_cnt) {
        init_package_nOK_cnt++;
        if (init_package_nOK_cnt > 5 * pparams.fps) {
            _init_package_failure = true;
            std::cout << "MultiModule wouldn't receive init package within 10 seconds." << std::endl;
            exitSendThread = true;
            return;
        }
    }
}

void MultiModule::handle_bind() {
    if (cycles_until_bind > 0) {
        cycles_until_bind--;
        if (cycles_until_bind == 0)
            _bind = true;
        arm_switch = BF_CHN_MIN;
        zerothrottle();
    }
    if (cycles_until_bind < 0) {
        cycles_until_bind++;
        if (cycles_until_bind == 0)
            _bind = false;
        arm_switch = BF_CHN_MIN;
        zerothrottle();
    }
    if (_bind) {
        arm_switch = BF_CHN_MIN;
        zerothrottle();
        if (sw_bind.Read() > 20000)
            _bind = false;
    }
}

void MultiModule::convert_channels(uint16_t *channels, unsigned char *packet) {
    uint32_t bits = 0;
    uint8_t bitsavailable = 0;
    uint16_t ii = 0;
    // byte 4-25, channels 0..2047
    for (uint16_t i = 0; i < MULTI_CHANS; i++) {
        uint16_t value = channels[i];
        bits |= value << bitsavailable;
        bitsavailable += MULTI_CHAN_BITS;
        while (bitsavailable >= 8) {
            packet[ii++] = bits & 0xff;
            bits >>= 8;
            bitsavailable -= 8;
        }
    }
}

int MultiModule::map(int inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange) {
    float x = (inValue - minInRange) / (maxInRange - minInRange);
    return minOutRange + (maxOutRange - minOutRange) * x;
}

void MultiModule::send_rc_data(void) {
    auto now = std::chrono::steady_clock::now();
    g_sendData.try_lock_until(now + std::chrono::milliseconds(30));
    g_lockData.lock();
    if (dparams.tx != tx_none) {
        unsigned char packet[RXBUFFER_SIZE] = {0}; // a packet is 36 bytes, see multimodule.h
        if (protocol < 31) {
            packet[0] = 0x55; // headerbyte
            packet[1] = protocol; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
        } else {
            packet[0] = 0x54; // headerbyte
            packet[1] = (protocol - 32) & 0x1F ; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
        }

        if (_bind) {
            packet[1] |= MULTI_BINDBIT;
        }
        uint8_t drone_id_rxnum_low = _drone_id_rxnum &  0b00001111;
        uint8_t drone_id_rxnum_high = _drone_id_rxnum & 0b00110000;
        packet[2]   = drone_id_rxnum_low | 0 | sub_protocol << 4; //RxNum | Power | Type
        if (RXBUFFER_SIZE > 26)
            packet[26] = (protocol & 0xC0) | drone_id_rxnum_high ;
        if (dparams.tx == tx_dsmx)
            packet[3] = tx_option; //option_protocol, number of channels for DSM
        else
            packet[3] = 0;

        //packet[4] to [25] = Channels, 16 channels of 11 bits (-> which makes 22 bytes)

        uint16_t channels[16];
        for (uint16_t i = 0; i < 16; i++) {
            channels[i] = 0;
        }

        if (calibrate_acc_cnt) {
            calibrate_acc_cnt--;
            arm_switch = BF_CHN_MIN;
            roll = BF_CHN_MID;
            pitch = BF_CHN_MIN;
            yaw = BF_CHN_MIN;
            throttle = BF_CHN_MAX;
        }
        //AETR
        channels[0] = roll;
        if (dparams.tx == tx_cx10)
            channels[0] = BF_CHN_MAX - roll;
        channels[1] = pitch;
        channels[2] = throttle;
        channels[3] = yaw;
        if ((dparams.tx == tx_dsmx) || (dparams.tx == tx_frskyd8) || (dparams.tx == tx_frskyd16) | (dparams.tx == tx_redpine)) {
            channels[4] = arm_switch;
            channels[5] = mode;
        }
        if (_beep || calibrate_acc_cnt) {
            std::cout << "BEEP" << std::endl;
            channels[4] = bf_disarmed;
            channels[5] = BF_CHN_MAX;
        }

        if (dparams.tx == tx_cx10)
            channels[5] = tx_rate;

        channels[6] = turtle_mode;

        if (calibrate_acc_cnt)
            channels[7] = BF_CHN_MIN + BF_CHN_RANGE / 100 * 100;
        else
            channels[7] = BF_CHN_MIN + BF_CHN_RANGE / 100 * led;

        // map channels to RC range
        for (uint16_t i = 0; i < 16; i++) {
            channels[i] = map(channels[i], BF_CHN_MIN, BF_CHN_MAX, RC_BOUND_MIN, RC_BOUND_MAX);;
        }
        convert_channels(channels, &packet[4]);

        if (!port_unopened) {
            RS232_SendBuf(static_cast<unsigned char *>(packet), RXBUFFER_SIZE);
        }
    }
    g_lockData.unlock();
}

void MultiModule::receive_data() {
    if (!port_unopened) {
        unsigned char inbuf[1];
        std::stringstream tmp;
        int n = 1;
        int totn = 0;
        while (n)    {
            n = RS232_PollComport(inbuf, 1);
            if (n > 0) {
                if (inbuf[0] > 0)
                    tmp << inbuf[0];
                totn += n;
            } else if (n < 0) {
                std::cout << "MM read error: " << n << std::endl;
            }
        }
        if (totn > 0) {
            received << tmp.str();
            std::string bufs = received.str();

            process_pats_init_packages(bufs);
            bool telemetry_sensor_data_detected = receive_telemetry(bufs);

            auto found = bufs.find_last_of('\r');
            if (found != std::string::npos) {
                received.str(std::string()); // clear stringstream
                received << bufs.substr(found + 1, bufs.size());
            }

            if (!telemetry_sensor_data_detected || init_package_nOK_cnt) // during init some interesting info may be printed, after that its mostly telemetry sensor data.
                std::cout << "MM: " << tmp.str()  << std::flush;
        }
    }
}

void MultiModule::process_pats_init_packages(std::string bufs) {
    if (init_package_nOK_cnt) {
        uint str_length = 0;

        const std::string version_str = "Multiprotocol version: ";

        auto found_version = bufs.rfind(version_str) ;
        str_length = found_version + version_str.length() + required_firmware_version_multimodule.length();
        if (found_version != std::string::npos && str_length < bufs.size()) {

            std::string current_firmware_version = bufs.substr(found_version + version_str.length(), required_firmware_version_multimodule.length());

            if (current_firmware_version != required_firmware_version_multimodule) {
                if (current_firmware_version.length() >= required_firmware_version_multimodule.length()) {
                    std::cout << "Detected wrong MultiModule firmware version! Detected: " << current_firmware_version << ". Required: "  << required_firmware_version_multimodule << "." << std::endl;
                    throw std::runtime_error("Detected wrong MultiModule firmware version!");
                }
            } else {
                std::cout << "Detecting MultiProtocol version " << required_firmware_version_multimodule << ": OK" << std::endl;
                mm_version_check_OK = true;
            }
        }

        const std::string specify_id = "Specify bind ID...";
        auto found_specify_bind_id = bufs.rfind(specify_id);
        str_length = found_specify_bind_id + specify_id.length();
        if (found_specify_bind_id != std::string::npos && str_length < bufs.size())
            send_init_package_now = true;
        const std::string id_received = "ID received:";
        auto found_id_received = bufs.rfind(id_received);
        str_length = found_id_received + id_received.length();
        if (found_id_received != std::string::npos && str_length < bufs.size()) {


            std::cout << "Multimodule received init package!" << std::endl;

            if (!mm_version_check_OK) {
                std::cout << "MultiProtocol version was not received." << std::endl;
                std::cout << "We did receive:\n" << bufs << std::endl;
                std::cout << "However, I didn't really like that. So retry!:\n" << std::endl;
                return;
            }

            send_init_package_now = false;
            init_package_nOK_cnt = 0;

            //below checks for the error message the MM gives if it is (already) waiting for the init package, but receives wrong bytes.
            const std::string i_want_66 = "I want 66";
            auto found_66 = bufs.rfind(i_want_66);
            str_length = found_66 + i_want_66.length();
            if (found_66 != std::string::npos && str_length < bufs.size()) {
                bufs = bufs.substr(found_66 + i_want_66.length(), bufs.length() - (found_66 + i_want_66.length()));
                send_init_package_now = true;
            }
        }
    }
}
bool utf8_check_is_valid(const string &string)
{
    int c, i, ix, n, j;
    for (i = 0, ix = string.length(); i < ix; i++)
    {
        c = static_cast<unsigned char>(string[i]);
        //if (c==0x09 || c==0x0a || c==0x0d || (0x20 <= c && c <= 0x7e) ) n = 0; // is_printable_ascii
        if (0x00 <= c && c <= 0x7f) n = 0; // 0bbbbbbb
        else if ((c & 0xE0) == 0xC0) n = 1; // 110bbbbb
        else if (c == 0xed && i < (ix - 1) && (static_cast<unsigned char>(string[i + 1]) & 0xa0) == 0xa0) return false; //U+d800 to U+dfff
        else if ((c & 0xF0) == 0xE0) n = 2; // 1110bbbb
        else if ((c & 0xF8) == 0xF0) n = 3; // 11110bbb
        //else if (($c & 0xFC) == 0xF8) n=4; // 111110bb //byte 5, unnecessary in 4 byte UTF-8
        //else if (($c & 0xFE) == 0xFC) n=5; // 1111110b //byte 6, unnecessary in 4 byte UTF-8
        else return false;
        for (j = 0; j < n && i < ix; j++) { // n bytes matching 10bbbbbb follow ?
            if ((++i == ix) || ((static_cast<unsigned char>(string[i]) & 0xC0) != 0x80))
                return false;
        }
    }
    return true;
}

bool MultiModule::receive_telemetry(std::string buffer) {
    const std::string sensor_str = "sensor:";
    auto found = buffer.rfind(sensor_str);
    uint str_length = found + sensor_str.length();
    if (found != std::string::npos && str_length < buffer.size()) {
        last_telemetry_time = _time;
        buffer = buffer.substr(found + sensor_str.length(), buffer.length() - (found + sensor_str.length()));

        auto arr = split_csv_line(buffer);
        uint16_t sensor_id = std::stoi(arr.at(0));

        switch (sensor_id) {
            case FSSP_DATAID_RSSI: {
                    //four bytes:
                    // RX_RSSI,TX_RSSI,RX_LQI,TX_LQI;
                    //rx rssi is the only one we really want to know:
                    int tmp_rssi = std::stoi(arr.at(1));
                    telemetry.rssi = tmp_rssi;
                    telemetry.rssi_last_received = _time;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_RSSI;" << static_cast<int>(telemetry.rssi) << "\n";
                    break;
            } case FSSP_DATAID_BF_VERSION: {
                    if (_bf_version_error >= 0) {
                        uint32_t bf_v = std::stoi(arr.at(1));
                        telemetry.bf_major = (bf_v & 0x00FF0000) >> 16;
                        telemetry.bf_minor = (bf_v & 0x0000FF00) >> 8;
                        telemetry.bf_patch = bf_v & 0x000000FF;
                        if (telemetry.bf_major != required_version_bf_major || telemetry.bf_minor != required_version_bf_minor || telemetry.bf_patch != required_version_bf_patch) {
                            std::cout << "Wrong betaflight version detected: " << telemetry.bf_major << "." << telemetry.bf_minor << "." << telemetry.bf_patch <<
                                      ", required: " << required_version_bf_major << "." << required_version_bf_minor << "." << required_version_bf_patch << std::endl;
                            _bf_version_error++;
                        } else {
                            _bf_version_error--;
                            std::cout << "Determined bf version: " << telemetry.bf_major << "." << telemetry.bf_minor << "." << telemetry.bf_patch << std::endl;
                        }
                        if (logger_initialized)
                            telem_logger << _time << ";FSSP_DATAID_BF_VERSION;" << telemetry.bf_major << "." << telemetry.bf_minor << "." << telemetry.bf_patch << "\n";
                    }
                    break;
            } case FSSP_DATAID_BF_UID: {
                    if (_bf_uid_error >= 0) {
                        uint32_t bf_uid = std::stoi(arr.at(1));
                        char *bf_uid_arr = reinterpret_cast<char *>(&bf_uid);
                        telemetry.bf_uid_str = string(bf_uid_arr).substr(0, 4);
                        if (telemetry.bf_uid_str != dparams.name) {
                            if (!utf8_check_is_valid(telemetry.bf_uid_str))
                                telemetry.bf_uid_str = "????";
                            std::cout << "Wrong drone uid detected: " + telemetry.bf_uid_str + ", required: " + dparams.name + "." << std::endl;
                            _bf_uid_error++;
                        } else {
                            _bf_uid_error--;
                            std::cout << "Determined bf drone name: " << telemetry.bf_uid_str << std::endl;

                        }
                        if (logger_initialized)
                            telem_logger << _time << ";FSSP_DATAID_BF_UID;" << telemetry.bf_uid_str << "\n";
                    }
                    break;
            } case FSSP_DATAID_VFAS: {
                    float data = std::stof(arr.at(1)) / 100.f;
                    if (data >= 0 && data <= batt_v_accepted_max)
                        telemetry.batt_v = data;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_VFAS;" << telemetry.batt_v << "\n";
                    break;
            } case FSSP_DATAID_A4: {
                    float data = std::stof(arr.at(1)) / 100.f;
                    if (data >= 0 && data <= batt_cell_v_accepted_max) {
                        telemetry.batt_cell_v = data;
                        telemetry.batt_cell_v_package_id++;
                    }
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_A4;" << telemetry.batt_cell_v << "\n";
                    break;
            } case FSSP_DATAID_CURRENT: {
                    float data = std::stof(arr.at(1));
                    telemetry.batt_current = data / 100.f;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_CURRENT;" << telemetry.batt_current << "\n";
                    break;
            } case FSSP_DATAID_ROLL: {
                    float data = std::stof(arr.at(1)) / 10.f;
                    if (data > -roll_accepted_max && data <= roll_accepted_max)
                        telemetry.roll = data;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ROLL;" << telemetry.roll << "\n";
                    break;
            } case FSSP_DATAID_PITCH: {
                    float data = std::stof(arr.at(1)) / 10.f;
                    if (data > -pitch_accepted_max && data <= pitch_accepted_max)
                        telemetry.pitch = data;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_PITCH;" << telemetry.pitch << "\n";
                    break;
            } case FSSP_DATAID_ACCX: {
                    float data = std::stof(arr.at(1));
                    telemetry.acc.x = data / 100.f;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ACCX;" << telemetry.acc.x << "\n";
                    break;
            } case FSSP_DATAID_ACCY: {
                    float data = std::stof(arr.at(1));
                    telemetry.acc.y = data / 100.f;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ACCY;" << telemetry.acc.y << "\n";
                    break;
            } case FSSP_DATAID_ACCZ: {
                    float data = std::stof(arr.at(1));
                    telemetry.acc.z = data / 100.f;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ACCZ;" << telemetry.acc.z << "\n";
                    break;
            } case FSSP_DATAID_ARMING: {
                    int data = std::stoi(arr.at(1));
                    telemetry.arming_state = static_cast<arming_states>(data);
                    telemetry.arming_state_package_id++;
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ARMING;" << data << ";" << arming_state_str() << "\n";
                    break;
            } case FSSP_DATAID_ROLL_PITCH: {
                    uint32_t data = std::stoi(arr.at(1));
                    float received_roll = static_cast<float>(static_cast<int16_t>(data & 0xFFFF)) / 10.f;
                    float received_pitch = static_cast<float>(static_cast<int16_t>((data >> 16) & 0xFFFF)) / 10.f;
                    if (received_roll > -roll_accepted_max && received_roll < roll_accepted_max && received_pitch > -pitch_accepted_max && received_pitch < pitch_accepted_max) {
                        telemetry.roll = received_roll;
                        telemetry.pitch = received_pitch;
                        telemetry.roll_pitch_package_id++;
                        if (logger_initialized) {
                            telem_logger << _time << ";FSSP_DATAID_ROLL;" << telemetry.roll << "\n";
                            telem_logger << _time << ";FSSP_DATAID_PITCH;" << telemetry.pitch << "\n";
                        }
                    }
                    if (logger_initialized)
                        telem_logger << _time << ";FSSP_DATAID_ROLL_PITCH;" << data << ";" << received_roll << ";" << received_pitch << "\n";
                    break;
            } default:
                break;
        }
        return true;
    }
    return false;
}
void MultiModule::close() {
    if (initialized) {
        std::cout << "Closing multimodule" << std::endl;
        beep(false);
        exitSendThread = true;
        g_sendData.unlock();
        g_lockData.unlock();
        send_thread_tx.join();
        usleep(1e5);
        exitReceiveThread = true;
        receive_thread_tx.join();

        // kill throttle when closing the module
        g_lockData.lock();
        mode = RC_BOUND_MIN;
        arm_switch = RC_BOUND_MIN;
        throttle = RC_BOUND_MIN;
        roll = RC_MIDDLE;
        pitch = RC_MIDDLE;
        yaw = RC_MIDDLE;
        _LED_drone = 0;

        g_sendData.unlock();
        g_lockData.unlock();
        send_rc_data();
        port_unopened = 1;
        RS232_CloseComport();
        if (logger_initialized) {
            telem_logger.flush();
            telem_logger.close();
        }
    }
    initialized = false;
}