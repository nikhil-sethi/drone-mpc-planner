#include "multimodule.h"




void MultiModule::init(bool fromfile) {
    // setup connection with MultiModule
    notconnected = RS232_OpenComport(115200,"/dev/ttyACM0");

    if (notconnected && !fromfile) {
        std::cout << "MultiModule failed." << std::endl;
        //exit(1);
    } else {
        thread_mm = std::thread(&MultiModule::worker_thread,this);
        initialised = true;
    }

}

void MultiModule::worker_thread(void) {
    std::cout << "Send multimodule thread started!" << std::endl;
    while (!exitSendThread) {
        receive_data();
        g_lockData.lock();
        send_data();
        g_lockData.unlock();
        if (_bind){
            if (sw_bind.Read() > 20000)
                _bind =false;
        }
        usleep(20000); //TODO: replace for conditional wait
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
#ifndef INSECT_LOGGING_MODE
    lock_rs232.lock();

    unsigned char packet[26] = {0}; // a packet is 26 bytes, see multimodule.h
    packet[0] = 0x55; // headerbyte
    packet[1] = TX_PROTOCOL; //sub_protocol|BindBit|RangeCheckBit|AutoBindBit;

    if (_bind) {
        packet[1] |= MULTI_BINDBIT;
    }
    packet[2]   = 6 | 0 | TX_SUB_PROTOCOL << 4; //RxNum | Power | Type
    if (TX_TYPE==TX_DSMX)
        packet[3] = TX_OPTION; //option_protocol, number of channels for DSM
    else
        packet[3] = 0;

    //packet[4] to [25] = Channels, 16 channels of 11 bits (-> which makes 22 bytes)

    uint16_t channels[16];
    for (int i = 0; i< 16;i++){
        channels[i] = 0;
    }
    //AETR
    channels[0] = roll;
    if (TX_TYPE==TX_CX10)
        channels[0] = JOY_MAX-roll;
    channels[1] = pitch;
    channels[2] = throttle;
    channels[3] = yaw;
    if (TX_TYPE==TX_DSMX || TX_TYPE==TX_FRSKYD)
        channels[4] = arm_switch;
    if (TX_TYPE==TX_CX10)
        channels[5] = TX_RATE;

    convert_channels(channels , &packet[4]);

    if (!notconnected) {
        RS232_SendBuf( static_cast<unsigned char*>( packet), 26);
    }

    lock_rs232.unlock();
#endif
}

void MultiModule::receive_data() {
    if (!notconnected) {
        unsigned char inbuf[1];
        inbuf[1] = 0;
        std::stringstream tmp;
        int n = 1;
        int totn = 0;
        while (n)    {
            n = RS232_PollComport(inbuf,1);
            if (n > 0) {
                tmp << inbuf[0];
                totn += n;
            }
        }
        if (totn > 0 ) {
            received << tmp.str();
            std::string bufs = received.str();
            if (bufs.find("cx10-initialized and bound!") != std::string::npos) {
                bound = cx10_bound;
                received.clear();
            }
            std::cout << tmp.str()  << std::flush;
        }else {
            received.clear();
        }
    }
}




void MultiModule::close() {
    if (initialised) {
#ifndef INSECT_LOGGING_MODE

        // kill throttle when closing the module
        throttle = JOY_MIN_THRESH;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;

        g_lockData.lock();
        send_data();
        g_lockData.unlock();

        exitSendThread = true;
        g_lockData.unlock();
        thread_mm.join();
#endif
    }
}
