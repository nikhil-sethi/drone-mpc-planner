#include "dronecontroller.h"


#include "defines.h"

const string paramsFile = "../controlParameters.dat";
unsigned int roll,pitch,yaw = 1500;
unsigned int thrust = 1000;

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

    #define TUNING
#ifdef TUNING
    // create GUI to set control parameters
    namedWindow("Control parameters", WINDOW_NORMAL); //create a window called "Control"
    // height control
    createTrackbar("Height - P", "Control parameters", &params.heightP, 2000);
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
    // yaw control
    createTrackbar("Yaw - P", "Control parameters", &params.yawP, 255);
    createTrackbar("Yaw - I", "Control parameters", &params.yawI, 255);
    createTrackbar("Yaw - D", "Control parameters", &params.yawD, 255);


#endif
}

void DroneController::control(trackData data) {

    if ( data.valid ) {
	

    //tmp
    roll = 1000 + params.rollI*3.92;
    pitch = 1000 + params.pitchI*3.92;
    yaw = 1000 + params.yawI*3.92;


    //std::cout << data.posX  << "   " << data.posY << std::endl;

    thrust = -data.posY * params.heightP - data.velY * params.heightD +  (1000 + params.heightI*3.92);

    }

    thrust = params.heightP;

    if ( thrust < 1000 )
	thrust = 1000;
    if ( thrust > 2000 )
	thrust = 2000;

   

    char buff[21];
    sprintf( (char*) buff,"%u,%u,%u,%u\n",thrust,roll,pitch,yaw);
    //sprintf( (char*) buff,"1200,1500,1500,1500\n");
    
    if ( data.valid ) {

    std::setprecision(2);
    std::cout << data.posY << " " << data.velY << " - ";
    std::cout << std::string(buff) << std::flush;
    }

    RS232_SendBuf( (unsigned char*) buff, 20);


    unsigned char inbuf[1];
    inbuf[1] = 0;
    std::stringstream tmp;
    int n = 1;
    while (n)    {
        n = RS232_PollComport(inbuf,1);
        tmp << inbuf[0];
    }
    tmp << std::endl;

}




void DroneController::close () {
    char buff[21];
    sprintf( (char*) buff,"1000,1500,1500,1500\n");
    RS232_SendBuf( (unsigned char*) buff, 20);
    RS232_CloseComport();

    std::cout << "closing controller" << std::endl;

    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );

}
