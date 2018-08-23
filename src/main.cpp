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
#include <sys/stat.h>

#include "common.h"
#include "defines.h"
#include "smoother.h"



#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "gstream.h"
#include "vizs.h"
#include "insecttracker.h"
#include "logreader.h"
#if CAMMODE == CAMMODE_FROMVIDEOFILE
#include "filecam.h"
#elif CAMMODE == CAMMODE_AIRSIM
#include "airsim.h"
#elif CAMMODE == CAMMODE_REALSENSE
#include "cam.h"
#elif CAMMODE == CAMMODE_GENERATOR
#include "generatorcam.h"
#endif
#include "visiondata.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#if CV_MAJOR_VERSION==2
#include <opencv2/contrib/contrib.hpp>
#endif

using namespace cv;
using namespace std;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
GStream output_video_results,output_video_LR;
cv::VideoWriter output_video_disp;

stopwatch_c stopWatch_break;
std::string file;
std::string data_output_dir;
std::string calib_folder;

int breakpause =-1;

std::ofstream logger;
Arduino arduino;
DroneTracker dtrkr;
DroneController dctrl;
DroneNavigation dnav;
InsectTracker itrkr;
Visualizer visualizer;
LogReader logreader;
#if CAMMODE == CAMMODE_FROMVIDEOFILE
FileCam cam;
#define Cam FileCam //wow that is pretty hacky :)
#elif CAMMODE == CAMMODE_AIRSIM
Airsim cam;
#define Cam Airsim //wow that is pretty hacky :)
#elif CAMMODE == CAMMODE_REALSENSE
Cam cam;
#elif CAMMODE == CAMMODE_GENERATOR
GeneratorCam cam;
#endif
VisionData visdat;
bool fromfile = false;


/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void handleKey();

/************ code ***********/
void process_video() {

    visdat.update(cam.frameL,cam.frameR,cam.get_frame_time(),cam.get_frame_id());

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
        float dt = cam.get_frame_time() - time;
        float break_time = ((float)stopWatch_break.Read())/1000.0f;
        if (breakpause_prev != 0)
            time = cam.get_frame_time() - break_time;

        logger << imgcount << ";" << cam.get_frame_id() << ";" ;

        visdat.update(cam.frameL,cam.frameR,cam.get_frame_time(),cam.get_frame_id());

        //WARNING: changing the order of the functions with logging must be matched with the init functions!
        dtrkr.track(cam.get_frame_time(),itrkr.predicted_pathL,dctrl.getDroneIsActive());
        //itrkr.track(cam.get_frame_time(), dtrkr.pathL,dctrl.getDroneIsActive());

        std::cout << "Found drone location:      [" << dtrkr.find_result.best_image_locationL.pt.x << "," << dtrkr.find_result.best_image_locationL.pt.y << "]" << std::endl;
#ifdef HASSCREEN
        if (breakpause_prev != 0) {
            visualizer.addPlotSample();
            visualizer.draw_tracker_viz(visdat.frameL,dnav.setpoint);
        }
#endif

        dnav.update();
        dctrl.control(dtrkr.get_last_track_data(),dnav.setpoint_world);

#ifdef HASSCREEN
        if (fromfile) {
            int rs_id = cam.get_frame_id();
            LogReader::Log_Entry tmp  = logreader.getItem(rs_id);
            if (tmp.RS_ID == rs_id) {
                dctrl.joyRoll = tmp.joyRoll;
                dctrl.joyPitch = tmp.joyPitch;
                dctrl.joyYaw = tmp.joyYaw;
                dctrl.joyThrottle = tmp.joyThrottle;
                dctrl.joySwitch = tmp.joySwitch;
            }
        }
#endif

        int frameWritten = 0;
#if VIDEORAWLR
        frameWritten = output_video_LR.write(cam.frameL,cam.frameR);
#endif
        if (frameWritten == 0) {
#if VIDEODISPARITY
            output_video_disp.write(cam.get_disp_frame());
#endif
#if VIDEORESULTS            
            output_video_results.write(visualizer.trackframe);
#endif
        }

        std::cout << "Frame: " <<imgcount << ", " << cam.get_frame_id() << ". FPS: " << imgcount / (time-break_time ) << ". Time: " << time-break_time  << ", dt " << dt << std::endl;
        imgcount++;

#ifdef HASSCREEN
        handleKey();
#endif
        if (imgcount > 60000)
            break;
        logger << std::endl;
    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}

void handleKey() {

    key = cv::waitKey(0);
    key = key & 0xff;
    if (key == 27) {  //esc
        //        cam.stopcam();
        return; // don't clear key, just exit
    }    

    switch(key) {
    case ' ':
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
    key=0;
}

void my_handler(int s){
    std::cout << "Caught ctrl-c:" << s << std::endl;
    key=27;
}

int init(int argc, char **argv) {

    if (argc ==2 ) {
        fromfile = true;
        logreader.init(string(argv[1]) + ".log");
    }
    data_output_dir = "./logging/";
    mkdir("./logging/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "data_output_dir: " << data_output_dir << endl;
    if (fromfile)
        logger.open(data_output_dir  + "test_fromfile.log",std::ofstream::out);
    else
        logger.open(data_output_dir  + "test.log",std::ofstream::out);

    logger << "ID;RS_ID;";

#if CAMMODE == CAMMODE_REALSENSE
    if (!fromfile)
        arduino.init(fromfile);
#endif

    /*****Start capturing images*****/
    cam.init(argc,argv);
    cam.update(); // wait for first frames

    visdat.init(cam.Qf, cam.frameL,cam.frameR); // do after cam update to populate frames

    //WARNING: changing the order of the inits with logging must be match with the process_video functions!    
    dtrkr.init(&logger,&visdat);
    itrkr.init(&logger,&visdat);
    dnav.init(&logger,&dtrkr,&dctrl);
    dctrl.init(&logger,fromfile,&arduino);

    logger << std::endl;
#ifdef HASSCREEN
    visualizer.init(&dctrl,&dtrkr,&itrkr,&dnav);
#endif

    /*****init the video writer*****/
#if VIDEORESULTS
    if (output_video_results.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",IMG_W,IMG_H,VIDEOFPS,"192.168.1.255",5000,true)) {return 1;}
#endif
#if VIDEORAWLR
    if (output_video_LR.init(argc,argv,VIDEORAWLR,data_output_dir + "test_videoRawLR.avi",IMG_W*2,IMG_H,VIDEOFPS, "192.168.1.255",5000,false)) {return 1;}
#endif

#if VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    output_video_disp.open(data_output_dir + "videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!output_video_disp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif

    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    std::cout << "Main init successfull" << std::endl;

    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;
    /*****Close everything down*****/
    dtrkr.close();
    dctrl.close();
    dnav.close();
    itrkr.close();
#if CAMMODE == CAMMODE_REALSENSE
    if (!fromfile)
        arduino.close();
#endif
#ifdef HASSCREEN
    visualizer.close();
#endif
    visdat.close();
    cam.close();

#if VIDEORESULTS   
    output_video_results.close();
#endif
#if VIDEORAWLR
    output_video_LR.close();
#endif

    std::cout <<"Closed"<< std::endl;
}

int main( int argc, char **argv )
{
    if (!init(argc,argv)) {
        process_video();
    }
    close();
    return 0;
}
