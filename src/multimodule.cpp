#include "multimodule.h"

void MultiModule::init(int drone_id) {
    _drone_id_rxnum = drone_id;

    switch (dparams.tx) {
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
        sub_protocol = d16_eu8;
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

// setup connection with MultiModule
    if (dparams.tx!=tx_none) {
        notconnected = RS232_OpenComport(115200,"/dev/pats_mm0");
        if (notconnected)
            notconnected = RS232_OpenComport(115200,"/dev/pats_mm1");
        if (!notconnected) {
            send_init_package_now = true;
            thread_mm = std::thread(&MultiModule::worker_thread,this);
            initialized = true;
        }
    }
}

void MultiModule::send_init_package() {
    //send bind id to the multimodule. It must have this form: [66,67 mode3d,id2,id1,id0,68], otherwise the MM won't proceed.
    unsigned char packet[7];
    packet[0] = 66; //header 1
    packet[1] = 67; //header 2
    packet[2] = dparams.mode3d;
    packet[3] = 0;  //id2...
    packet[4] = 0; // id1
    packet[5] = _drone_id_tx; //id0
    packet[6] = 68; //header 3
    RS232_SendBuf( static_cast<unsigned char*>( packet), 7);
}

void MultiModule::zerothrottle() {
    if (dparams.mode3d)
        throttle = JOY_MIDDLE;
    else
        throttle = JOY_BOUND_MIN;
}

void MultiModule::worker_thread(void) {
    std::cout << "Send multimodule thread started!" << std::endl;
    while (!exitSendThread) {

        receive_data();

        if (init_package_nOK_cnt) {
            init_package_nOK_cnt++;
            if (init_package_nOK_cnt > 5 * pparams.fps) {
                _init_package_failure = true;
                std::cout << "MultiModule wouldn't receive init package within 10 seconds." << std::endl;
                exitSendThread = true;
                return;
            }
        }

        if (send_init_package_now) {
            usleep(100);
            send_init_package();
            send_init_package_now = false;
            usleep(100);
        } else {
            if (cycles_until_bind > 0) {
                cycles_until_bind--;
                if (cycles_until_bind ==0)
                    _bind = true;
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }
            if (cycles_until_bind < 0) {
                cycles_until_bind++;
                if (cycles_until_bind == 0)
                    _bind = false;
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }
            if (_bind) {
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }

            g_lockData.lock();
            send_data();
            g_lockData.unlock();
            if (_bind) {
                if (sw_bind.Read() > 20000)
                    _bind =false;
            }
            g_sendData.lock();
        }
    }
}

void MultiModule::convert_channels(uint16_t * channels, unsigned char * packet) {
    uint32_t bits = 0;
    uint8_t bitsavailable = 0;
    uint16_t ii = 0;
    // byte 4-25, channels 0..2047
    for (int i = 0; i < MULTI_CHANS; i++) {
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

void MultiModule::send_data(void) {
    if (dparams.tx != tx_none) {
        lock_rs232.lock();
        unsigned char packet[RXBUFFER_SIZE] = {0}; // a packet is 26 bytes, see multimodule.h
        if (protocol < 31) {
            packet[0] = 0x55; // headerbyte
            packet[1] = protocol; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
        } else {
            packet[0] = 0x54; // headerbyte
            packet[1] = (protocol-32) & 0x1F ; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
        }

        if (_bind) {
            packet[1] |= MULTI_BINDBIT;
        }
        uint8_t drone_id_rxnum_low = _drone_id_rxnum &  0b00001111;
        uint8_t drone_id_rxnum_high = _drone_id_rxnum & 0b00110000;
        packet[2]   = drone_id_rxnum_low | 0 | sub_protocol << 4; //RxNum | Power | Type
        if (RXBUFFER_SIZE > 26)
            packet[26] = (protocol & 0xC0) | drone_id_rxnum_high ;
        if (dparams.tx==tx_dsmx)
            packet[3] = tx_option; //option_protocol, number of channels for DSM
        else
            packet[3] = 0;

        //packet[4] to [25] = Channels, 16 channels of 11 bits (-> which makes 22 bytes)

        uint16_t channels[16];
        for (int i = 0; i< 16; i++) {
            channels[i] = 0;
        }

        if (calibrate_acc_cnt) {
            calibrate_acc_cnt--;
            arm_switch = JOY_BOUND_MIN;
            roll = JOY_MIDDLE;
            pitch = JOY_BOUND_MIN;
            yaw = JOY_BOUND_MIN;
            throttle = JOY_BOUND_MAX;
        }
        //AETR
        channels[0] = roll;
        if (dparams.tx==tx_cx10)
            channels[0] = JOY_MAX-roll;
        channels[1] = pitch;
        channels[2] = throttle;
        channels[3] = yaw;
        if (dparams.tx==tx_dsmx || dparams.tx==tx_frskyd8 || dparams.tx==tx_frskyd16 | dparams.tx==tx_redpine) {
            channels[4] = arm_switch;
            channels[5] = mode;
        }
        if (_beep || calibrate_acc_cnt ) {
            std::cout << "BEEP" << std::endl;
            channels[4] = bf_disarmed;
            channels[5] = JOY_BOUND_MAX;
        }

        if (dparams.tx==tx_cx10)
            channels[5] = tx_rate;

        channels[6] = turtle_mode;

        if (calibrate_acc_cnt)
            channels[7] = JOY_BOUND_MIN + JOY_BOUND_RANGE / 100 * 100;
        else
            channels[7] = JOY_BOUND_MIN + JOY_BOUND_RANGE / 100 * _LED_drone;

        convert_channels(channels, &packet[4]);

        if (!notconnected) {
            RS232_SendBuf( static_cast<unsigned char*>( packet), RXBUFFER_SIZE);
        }

        lock_rs232.unlock();
    }
}

void MultiModule::receive_data() {
    if (!notconnected) {
        unsigned char inbuf[1];
        std::stringstream tmp;
        int n = 1;
        int totn = 0;
        while (n)    {
            n = RS232_PollComport(inbuf,1);
            if (n > 0) {
                if (inbuf[0]>0)
                    tmp << inbuf[0];
                totn += n;
            }
        }
        if (totn > 0 ) {
            uint str_length = 0;
            received << tmp.str();
            std::string bufs = received.str();
            const std::string version_str = "Multiprotocol version: ";
            const std::string required_firmwar_version = "1.3.1.45";
            auto found = bufs.rfind(version_str) ;
            str_length = found+version_str.length()+required_firmwar_version.length();
            if (found != std::string::npos && str_length < bufs.size()) {

                std::string current_firmware_version = bufs.substr(found+version_str.length(),required_firmwar_version.length());

                if (current_firmware_version != required_firmwar_version) {
                    if (current_firmware_version.length() >= required_firmwar_version.length()) {
                        std::cout << "Detected wrong MultiModule firmware version! Detected: " << current_firmware_version << ". Required: "  << required_firmwar_version << "." << std::endl;
                        exit(1);
                    }
                } else {
                    std::cout << "Detecting MultiProtocol version " << required_firmwar_version << ": OK" << std::endl;
                    version_check_OK = true;
                }
            }

            const std::string specify_id = "Specify bind ID...";
            found = bufs.rfind(specify_id);
            str_length = found+specify_id.length();
            if (found != std::string::npos && str_length < bufs.size())
                send_init_package_now = true;
            const std::string id_received = "ID received:";
            found = bufs.rfind(id_received);
            auto delete_until_id_received = found+id_received.length();
            if (found != std::string::npos && delete_until_id_received < bufs.size()) {
                send_init_package_now = false;

                std::cout << "Multimodule received init package!" << std::endl;
                init_package_nOK_cnt = 0;
                if (!version_check_OK) {
                    std::cout << "MultiProtocol version was not received." << std::endl;
                    exit(1);
                }
            }

            //below checks for the error message the MM gives if it is (already) waiting for the init package, but receives from bytes.
            const std::string i_want_66 = "I want 66";
            found = bufs.rfind(i_want_66);
            str_length = found+i_want_66.length();
            if (found != std::string::npos && str_length < bufs.size()) {
                bufs = bufs.substr(found+9,bufs.length() - (found+9));
                send_init_package_now = true;
            }


            found = bufs.find_last_of('\r');
            if (found != std::string::npos) {
                received.str(std::string());
                received << bufs.substr(found+1, bufs.size());
            }

            std::cout << tmp.str()  << std::flush;
        }
    }
}

void MultiModule::close() {
    if (initialized) {
        std::cout << "Closing multimodule" << std::endl;
        exitSendThread = true;
        g_lockData.lock();
        g_sendData.unlock();

        thread_mm.join();


        // kill throttle when closing the module
        mode = JOY_BOUND_MIN;
        arm_switch = JOY_BOUND_MIN;
        throttle = JOY_BOUND_MIN;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        _LED_drone = 0;
        send_data();
        notconnected = true;
        RS232_CloseComport();
        g_lockData.unlock();

    }
    initialized = false;
}
