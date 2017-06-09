#include "dronecontroller.h"
#include "defines.h"

#define TUNING

const string paramsFile = "../controlParameters.dat";
unsigned int roll,pitch,yaw = 1500;
unsigned int thrust = 1000;
// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

int notconnected;

bool DroneController::init(void) {
	std::cout << "Initialising control." << std::endl;
    // setup connection with Arduino
    baudrate = 115200;
    notconnected = RS232_OpenComport(baudrate);

	// Ensure that it was found and that we can use it
	if (!joystick.isFound()) {
		printf("joystick failed.\n");
		exit(1);
	}

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(params);
    }

#ifdef TUNING
	std::cout << "Creating control tuning window." << std::endl;
    // create GUI to set control parameters
    namedWindow("Control", WINDOW_NORMAL);
    // height control
    createTrackbar("Height P", "Control", &params.heightP, 2000);
    createTrackbar("Height I", "Control", &params.heightI, 255);
    createTrackbar("Height D", "Control", &params.heightD, 255);

    // roll control
    createTrackbar("Roll P", "Control", &params.rollP, 5000);
    createTrackbar("Roll I", "Control", &params.rollI, 255);
    createTrackbar("Roll D", "Control", &params.rollD, 255);
    // pitch control
    createTrackbar("Pitch P", "Control", &params.pitchP, 255);
    createTrackbar("Pitch I", "Control", &params.pitchI, 255);
    createTrackbar("Pitch D", "Control", &params.pitchD, 255);
    // yaw control
    createTrackbar("Yaw P", "Control", &params.yawP, 255);
    createTrackbar("Yaw I", "Control", &params.yawI, 255);
    createTrackbar("Yaw D", "Control", &params.yawD, 255);

#endif
}

void DroneController::control(trackData data) {

    if ( data.valid ) {
		//thrust = -data.posY * params.heightP - data.velY * params.heightD +  (1000 + params.heightI*3.92);
        roll -= data.posX * params.rollP + data.velX * params.rollD;
    }

	// joystick 
	while (joystick.sample(&event))
    {
      if (event.isAxis())
      {		
		if ( event.number == 0 ) {// roll
			std::cout <<  "Roll override!" << std::endl;	
			roll = 1500 + (event.value >> 6);			
		}
		if ( event.number == 1 ) {// pitch
			pitch = 1500 - (event.value >> 6);			
		}
		if ( event.number == 2 ) {// thrust
			thrust = 1500 - (event.value >> 6);		
			
		}
		if ( event.number == 5 ) {// yaw
			yaw = 1500 + (event.value >> 6);			
		}
      }
    }

    if ( thrust < 1050 )
		thrust = 1050;
    if ( thrust > 1950 )
		thrust = 1950;

    if ( roll < 1050 )
		roll = 1050;
    if ( roll > 1950 )
		roll = 1950;
    commandedRoll = roll;

	int mode = 1500; // <min = mode 1, 1500 = mode 2, >max = mode 3 

    char buff[64];
    sprintf( (char*) buff,"%u,%u,%u,%u,1500,0,0,0,0,0,0,0\n",thrust,roll,pitch,yaw);    
    if (!notconnected) {
        RS232_SendBuf( (unsigned char*) buff, 63);
    }


    //if ( data.valid ) {
		//std::setprecision(2);
		//std::cout << data.posY << " " << data.velY << " - ";
		std::cout << "JoyCommands:" << std::string(buff) << std::flush;
    //}

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
	if (totn > 0) {
	   // std::cout << totn << ": " << tmp.str() << std::endl;
	}
}

	//TODO: disable uart polling after bind confirmed!
//check if input string is the same as the commanded strning, if so bind was succesfull!
//	for (int i = 0 to tmp.count;i++) {
		//if tmp.str.c_str[i] == buff[i] ...
//	}


}

void DroneController::close () {
    char buff[64];
    sprintf( (char*) buff,"1050,1500,1500,1500,0,0,0,0,0,0,0,2000\n");
    if (!notconnected) {
    RS232_SendBuf( (unsigned char*) buff, 63);
    RS232_CloseComport();
    }

    std::cout << "closing controller" << std::endl;

    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );

}
