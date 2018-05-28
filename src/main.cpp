#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ctime>

#include "common.h"
#include "defines.h"
#include "smoother.h"

#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "gstream.h"
#include "vizs.h"
#include "insect.h"
#include "logreader.h"
#include "cam.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
GStream outputVideoColor,outputVideoRawLR;
cv::VideoWriter outputVideoDisp;

stopwatch_c stopWatch_break;
std::string file;
std::string data_output_dir;
std::string calib_folder;

int breakpause =-1;

std::ofstream logger;
Arduino arduino;
DroneTracker dtrkr;
DroneController dctrl;
Insect insect;
Visualizer visualizer;
LogReader logreader;
Cam cam;
bool fromfile = false;


//tmp for capturing moth data
#define FRAME_BUF_SIZE 20
int frame_buffer_write_id = 0;
int frame_buffer_read_id = 2*FRAME_BUF_SIZE;
int frame_write_id_during_event = 0;
cv::Mat frameBL[FRAME_BUF_SIZE];
cv::Mat frameBR[FRAME_BUF_SIZE];

/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void handleKey();

/************ code ***********/
void process_video() {


    float start_time = cam.frame_time;

    //main while loop:
    while (key != 27) // ESC
    {

        static int breakpause_prev =-1;
        if (breakpause == 0 && breakpause_prev!=0) {
            cam.pause();
            stopWatch_break.Resume();
            dtrkr.breakpause = true;
        } else if (breakpause != 0 && breakpause_prev==0) {
            cam.resume();
            stopWatch_break.Stop();
            dtrkr.breakpause = false;
        }
        breakpause_prev = breakpause;

        if (breakpause != 0) {

            cam.update();

            if (breakpause > 0)
                breakpause--;
        }


        static float time =0;
        float dt = cam.frame_time - time;
        float break_time = ((float)stopWatch_break.Read())/1000.0;
        if (breakpause_prev != 0)
            time = cam.frame_time - break_time;


        logger << imgcount << ";" << cam.frame_number << ";" ;
        if (!INSECT_DATA_LOGGING_MODE) {
            if (dtrkr.track(cam.frameL,cam.frameR, cam.Qf, cam.frame_time-start_time, cam.frame_number)) {
                breakpause = 0;
            }
            dctrl.control(&(dtrkr.data));
        }
#if INSECT_DATA_LOGGING_MODE
        insect.track(frameL,frameR, Qf);
#endif


#ifdef HASSCREEN
        if (fromfile) {
            int rs_id = cam.frame_number;
            LogReader::Log_Entry tmp  = logreader.getItem(rs_id);
            if (tmp.RS_ID == rs_id) {
                dctrl.joyRoll = tmp.joyRoll;
                dctrl.joyPitch = tmp.joyPitch;
                dctrl.joyYaw = tmp.joyYaw;
                dctrl.joyThrottle = tmp.joyThrottle;
                dctrl.joySwitch = tmp.joySwitch;
            }
        }

#if !INSECT_DATA_LOGGING_MODE
        if (breakpause_prev != 0)
            visualizer.addSample();
#endif
#endif

        frameBL[frame_buffer_write_id] = cam.frameL.clone();
        frameBR[frame_buffer_write_id] = cam.frameR.clone();

        frame_buffer_write_id = (frame_buffer_write_id + 1) % FRAME_BUF_SIZE;
        if (insect.data.valid) { //
            frame_buffer_read_id = 0;
            frame_write_id_during_event = (frame_buffer_write_id + FRAME_BUF_SIZE - 1 ) % FRAME_BUF_SIZE;
        }

        if (frame_buffer_read_id<2*FRAME_BUF_SIZE) {
            detectcount++;
            frame_buffer_read_id++;
            int id = (frame_write_id_during_event + frame_buffer_read_id)  % FRAME_BUF_SIZE;
            int frameWritten = 0;
#if VIDEORAWLR
            frameWritten = outputVideoRawLR.write(frameBL[id],frameBR[id]);
#endif
            if (frameWritten == 0) {
#if VIDEODISPARITY
                outputVideoDisp.write(cam.get_disp_frame());
#endif
#if VIDEORESULTS
                resFrame = dtrkr.resFrame;
                outputVideoColor.write(resFrame);
#endif
            }
        }

        std::cout << "Frame: " <<imgcount << ", " << cam.frame_number << ". FPS: " << imgcount / (time-start_time-break_time ) << ". Time: " << time << ", dt " << dt << std::endl;
        imgcount++;

        handleKey();
        if (imgcount > 60000)
            break;
        logger << std::endl;
    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}

void handleKey() {

    //#ifdef HASSCREEN
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        //        cam.stopcam();
        return; // don't clear key, just exit
    }
    //#endif

    switch(key) {    
    case ' ': // [r]: reset stopwatch
        //dtrkr.breakpause = true;
        if (breakpause >-1) {
            breakpause =-1;
        } else {
            breakpause = 0;
        }
        break;
    case 'n': // next frame
        //dtrkr.breakpause = true;
        breakpause = 1;
        break;

    } // end switch key
#ifndef HASSCREEN
    if (key!=0) {
        //std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

void my_handler(int s){
    std::cout << "Caught ctrl-c:" << s << std::endl;
}

int init(int argc, char **argv) {

#if INSECT_DATA_LOGGING_MODE
    if (argc !=2 ) {
        cout << "Error: command line argument missing. Missing argument Output dir." << endl;
        exit(1);
    }
    data_output_dir = string(argv[1]) + "/";
#else
    if (argc ==2 ) {
        fromfile = true;
        logreader.init(string(argv[1]) + ".txt");
    }
    data_output_dir = "./";
#endif

    cout << "data_output_dir: " << data_output_dir << endl;

    logger.open(data_output_dir  + "log.txt",std::ofstream::out);
    logger << "ID;RS_ID;";

    arduino.init(fromfile);

    if (!INSECT_DATA_LOGGING_MODE) {
        dtrkr.init(&logger);
        dctrl.init(&logger,fromfile,&arduino);
    }
    insect.init(&logger,&arduino);
    logger << std::endl;

    std::cout << "Frame buf size: " << FRAME_BUF_SIZE << std::endl;

    /*****Start capturing images*****/
    cam.init(argc,argv);

    /*****init the video writer*****/
#if VIDEORESULTS
    if (outputVideoColor.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",IMG_W,IMG_H,VIDEOFPS,"192.168.1.10",5004,true)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,data_output_dir + "videoRawLR.avi",IMG_W*2,IMG_H,VIDEOFPS, "127.0.0.1",5000,false)) {return 1;}
#endif

#if VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    outputVideoDisp.open(data_output_dir + "videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!outputVideoDisp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif

    visualizer.init(&dctrl,&dtrkr);

    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //init pre-capture buffer
    for (int i = 0; i<FRAME_BUF_SIZE;i++){
        cam.update(); // Wait for next set of frames from the camera
        frameBL[i] = cam.frameL.clone();
        frameBR[i] = cam.frameR.clone();
    }

    std::cout << "Main init successfull" << std::endl;

    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;
    /*****Close everything down*****/
    if (!INSECT_DATA_LOGGING_MODE) {
        dtrkr.close();
        dctrl.close();
    }
    insect.close();
    arduino.close();
    visualizer.close();
    cam.close();

#if VIDEORESULTS   
    outputVideoColor.close();
#endif
#if VIDEORAWLR
    outputVideoRawLR.close();
#endif

    std::cout <<"Closed"<< std::endl;
}

int main( int argc, char **argv )
{
    if (init(argc,argv)) {return 1;}
    process_video();
    close();
    return 0;
}
