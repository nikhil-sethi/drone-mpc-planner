#include "dronecontroller.h"


#include "defines.h"

const string paramsFile = "../controlParameters.dat";
unsigned int thrust,roll,pitch,yaw = 0;

bool DroneController::init(void) {

    // setup connection with Arduino
    baudrate = 115200;
    RS232_OpenComport(baudrate);

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(params);
    }

    //#define TUNING
#ifdef TUNING
    // create GUI to set control parameters
    namedWindow("Control parameters", WINDOW_NORMAL); //create a window called "Control"
    // height control
    createTrackbar("Height - P", "Control parameters", &params.heightP, 255);
    createTrackbar("Height - I", "Control parameters", &params.heightI, 255);
    createTrackbar("Height - D", "Control parameters", &params.heightD, 255);
    // roll control
    createTrackbar("Roll - P", "Control parameters", &params.rollP, 255);
    createTrackbar("Roll - I", "Control parameters", &params.rollI, 255);
    createTrackbar("Roll - D", "Control parameters", &params.rollD, 255);
    // pitch control
    createTrackbar("Pitch - P", "Control parameters", &params.pitchP, 255);
    createTrackbar("Pitch - I", "Control parameters", &params.pitchI, 255);
    createTrackbar("Pitch - D", "Control parameters", &params.pitchD, 255);


#endif
}

void DroneController::control(trackData data) {

    //tmp
    roll = 0;
    pitch = 0;
    yaw = 0;


    std::cout << data.posX  << "   " << data.posY << std::endl;

    thrust = data.posY * params.heightP + data.velY * params.heightD +  params.heightI;

    char buff[21];
    sprintf( (char*) buff,"%ud,%ud,%ud,%ud\n",thrust,roll,pitch,yaw);
    RS232_SendBuf( (unsigned char*) buff, 20);

    std::string hoer (buff);
    std::cout << hoer << std::endl;

}




void DroneController::close () {

    RS232_CloseComport();

    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );

}
