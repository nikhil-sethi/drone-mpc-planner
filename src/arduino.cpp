#include "arduino.h"


int notconnected;

void Arduino::init(bool fromfile) {
    // setup connection with Arduino
    baudrate = 115200;
    notconnected = RS232_OpenComport(baudrate);

    if (notconnected && !fromfile) {
        std::cout << "Arduino failed." << std::endl;
        //        exit(1);
    }

    thread_nrf = std::thread(&Arduino::workerThread,this);

    std::cout << "led power: " << ledpower << std::endl;
}

void Arduino::workerThread(void) {
    std::cout << "Send nrf thread started!" << std::endl;
    while (!exitSendThread) {
        g_lockData.lock();
        sendData();
        g_lockData.unlock();
        usleep(20000);
    }
}

void Arduino::sendData(void) {
    lock_rs232.lock();

    if (false) {
        //tmp led power hack
        char buff[3];
        buff[0] = 'B';
        buff[1] = 't';
        buff[2] = ledpower;
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 3);
        }
        usleep(100000);
    } else {

        char buff[64];
        sprintf( (char*) buff,"%u,%u,%u,%u,%u,%u,0,0,0,0,0,0\n",throttle,roll,pitch,yaw,mode,ledpower);
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 63);
        }

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
            if (totn > 0 && bound == cx10_binding) {
                received << tmp.str();
                std::string bufs = received.str();
                if (bufs.find("cx10-initialized and bound!") != std::string::npos) {
                    bound = cx10_bound;
                    received.clear();
                }
                std::cout << "***Arduino***" << std::endl << totn << ": " << tmp.str()  << "***********" << std::endl;
            }else {
                received.clear();
            }


        }

        //TODO: disable uart polling after bind confirmed!
        //check if input string is the same as the commanded strning, if so bind was succesfull!
        //	for (int i = 0 to tmp.count;i++) {
        //if tmp.str.c_str[i] == buff[i] ...
        //	}

    }
    lock_rs232.unlock();
}

void Arduino::check_bind_command(void){

    if (bind_next_cycle) {
                    alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Amsterdam.ogg &");
        lock_rs232.lock();
        bound = cx10_binding;
        bind_next_cycle=false;
        binding_sw.Start();

        char buff[64];
        sprintf( (char*) buff,"1050,1500,1500,1500,0,0,0,0,0,0,0,2000\n");
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 63);
            RS232_CloseComport();
        }
        usleep(100000);
        notconnected = RS232_OpenComport(baudrate);
        lock_rs232.unlock();
        if (notconnected) {
            bound = cx10_not_bound;
            std::cout << "Bind failure, could not reconnect to arduino" << std::endl;
            exit(1);
        }
    }
    if (binding_sw.Read() > 2000 && bound == cx10_binding) {
        std::cout << "Binding failed... retrying" << std::endl;
        bind_next_cycle = true;
    }

}


void Arduino::close() {
    exitSendThread = true;
    g_lockData.unlock();
    thread_nrf.join();
}
