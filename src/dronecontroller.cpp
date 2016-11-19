#include "dronecontroller.h"

bool DroneController::init(void) {

baudrate = 115200;
RS232_OpenComport(baudrate);
   
}

void DroneController::control(float pos_X, float pos_Y, float pos_Z) {



unsigned char buff[21];

sprintf( (char*) buff,"1300,1500,1500,1500\n");
RS232_SendBuf( (unsigned char*) buff, 20);

}


void DroneController::close () {

RS232_CloseComport();
   
}
