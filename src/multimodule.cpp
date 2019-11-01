#include "multimodule.h"

void MultiModule::init(int drone_id, bool fromfile) {
    _drone_id = drone_id;

    if (dparams.tx == tx_none) {
        protocol = 0;
        sub_protocol = 0;
        tx_option = 0;
        tx_rate = 0;
    } else if (dparams.tx == tx_dsmx) {
        protocol = 6;
        sub_protocol = 3;
        tx_option = 7;
        tx_rate = 0;
    } else if (dparams.tx == tx_cx10) {
        protocol = 12;
        sub_protocol = 1;
        tx_rate = 2000;
        tx_option = 0;
    } else if (dparams.tx == tx_frskyd8) {
        protocol = 3;
        sub_protocol = 0;
        tx_rate = 0;
        tx_option = 0;
    } else if (dparams.tx == tx_frskyd16) {
        protocol = 15;
        sub_protocol = 1;
        tx_rate = 0;
        tx_option = 0;
    }

    zerothrottle();

    // setup connection with MultiModule
    notconnected = RS232_OpenComport(115200,"/dev/pats_mm0");
    if (dparams.tx!=tx_none) {
        if (notconnected && !fromfile) {
            throw my_exit("cannot connect the MultiModule");
        } else {
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
    packet[5] = _drone_id; //id0
    packet[6] = 68; //header 3
    RS232_SendBuf( static_cast<unsigned char*>( packet), 7);
}

void MultiModule::zerothrottle(){
    if (dparams.mode3d)
        throttle = JOY_MIDDLE;
    else
        throttle = JOY_BOUND_MIN;
}

void MultiModule::LED(bool value){
    if (value)
        led_on = true;
    else
        led_on = false;
}

void MultiModule::worker_thread(void) {
    std::cout << "Send multimodule thread started!" << std::endl;
    while (!exitSendThread) {

        receive_data();

        if (init_package_nOK_cnt) {
            init_package_nOK_cnt++;
            if (init_package_nOK_cnt > 5 * pparams.fps) {
                std::cout << "MultiModule wouldn't receive init package within 10 seconds." << std::endl;
                exit(1); // goal justifies the mean...
            }
        }

        if (send_init_package_now){
            usleep(100);
            send_init_package();
            send_init_package_now = false;
            usleep(100);
        } else {
            if (cycles_until_bind > 0){
                cycles_until_bind--;
                if (cycles_until_bind ==0)
                    _bind = true;
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }
            if (cycles_until_bind < 0){
                cycles_until_bind++;
                if (cycles_until_bind == 0)
                    _bind = false;
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }
            if (_bind){
                arm_switch = JOY_MIN_THRESH;
                zerothrottle();
            }

            g_lockData.lock();
            send_data();
            g_lockData.unlock();
            if (_bind){
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
    if (!pparams.insect_logging_mode) {
        lock_rs232.lock();

        unsigned char packet[26] = {0}; // a packet is 26 bytes, see multimodule.h
        packet[0] = 0x55; // headerbyte
        packet[1] = protocol; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;

        if (_bind) {
            packet[1] |= MULTI_BINDBIT;
        }
        packet[2]   = 0 | 0 | sub_protocol << 4; //RxNum | Power | Type
        if (dparams.tx==tx_dsmx)
            packet[3] = tx_option; //option_protocol, number of channels for DSM
        else
            packet[3] = 0;

        //packet[4] to [25] = Channels, 16 channels of 11 bits (-> which makes 22 bytes)

        uint16_t channels[16];
        for (int i = 0; i< 16;i++){
            channels[i] = 0;
        }
        //AETR
        channels[0] = roll;
        if (dparams.tx==tx_cx10)
            channels[0] = JOY_MAX-roll;
        channels[1] = pitch;
        channels[2] = throttle;
        channels[3] = yaw;
        if (dparams.tx==tx_dsmx || dparams.tx==tx_frskyd8 || dparams.tx==tx_frskyd16){
            channels[4] = arm_switch;
            channels[5] = JOY_BOUND_MIN; // set to angle mode in BF
        }
        if (dparams.tx==tx_cx10)
            channels[5] = tx_rate;

        if(led_on)
            channels[7] = JOY_BOUND_MAX;
        else
            channels[7] = JOY_BOUND_MIN;


        convert_channels(channels , &packet[4]);

        if (!notconnected) {
            RS232_SendBuf( static_cast<unsigned char*>( packet), 26);
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
            received << tmp.str();
            std::string bufs = received.str();
            auto found = bufs.find("Multiprotocol version:") ;
            if (found != std::string::npos) {
                bufs.replace(found,1,"*");
                received.clear();
                received << bufs;
                std::string current_firmware_version = bufs.substr(0, bufs.find(":"));
                std::string required_firmwar_version = "1.4.0.1";
                if (current_firmware_version != required_firmwar_version) {
                    std::cout << "Detected wrong MultiModule firmware version! Detected: " << current_firmware_version << ". Required: "  << required_firmwar_version << "." << std::endl;
                    exit(1); // goal justifies the mean...
                }
            }
            found = bufs.find("Specify bind ID...") ;
            if (found != std::string::npos) {
                bufs.replace(found,1,"*");
                received.clear();
                received << bufs;
                send_init_package_now = true;
            }
            found = bufs.find("ID received:") ;
            if (found != std::string::npos) {
                bufs.replace(found,1,"*");
                received.clear();
                received << bufs;
                std::cout << "Multimodule received init package!" << std::endl;
                init_package_nOK_cnt = 0;
            }

            if (bufs.size() > 500) {
                received.clear();
                received << bufs.substr(250, bufs.size());
            }


            std::cout << tmp.str()  << std::flush;
        }
    }
}

void MultiModule::close() {
    if (initialized && !pparams.insect_logging_mode) {
        std::cout << "Closing multimodule" << std::endl;
        // kill throttle when closing the module
        throttle = JOY_MIN_THRESH;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;

        g_lockData.lock();
        send_data();
        g_lockData.unlock();
        g_sendData.unlock();

        exitSendThread = true;
        g_lockData.unlock();
        g_sendData.unlock();
        thread_mm.join();
    }
    initialized = false;
}
