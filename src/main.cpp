#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>

#include "common.h"

#include "defines.h"
#include "smoother.h"
#ifdef _PC
#include "kalamosfilecam.h"
#else
#include "kalamoscam.h"
#endif
#include "stopwatch.h"
#include "stereoalg.h"
#include "dronetracker.h"
#include "dronecontroller.h"

#include "opencv2/features2d/features2d.hpp"

/************gst stuff *****/
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>

int want = 1;
GstElement *pipeline,*appsrc;

int initgst(int argc, char **argv);
static void cb_need_data (GstElement *appsrc, guint unused_size, gpointer user_data);
static void prepare_buffer(GstAppSrc* appsrc, cv::Mat * image);


using namespace cv;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
std::string msg;
int imgcount; // to measure fps
cv::Mat resFrame;
cv::VideoWriter outputVideoResults;
cv::VideoWriter outputVideoRawL,outputVideoRawR;
cv::VideoWriter outputVideoDisp;
stopwatch_c stopWatch;
std::string file;
std::string imageOutputDir;

int mouseX, mouseY;
int mouseLDown;
int mouseMDown;
int mouseRDown;
bool pausecam = false;

#ifdef _PC
KalamosFileCam cam;
#else
KalamosCam cam;
#endif
stereoAlg stereo;
DroneTracker dtrk;
DroneController dctrl;

/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void handleKey();

/************ code ***********/

void process_video() {
    std::cout << "Running...\n";
    stopWatch.Start();


    //main while loop:
    while (key != 27 && cam.getCamRunning()) // ESC
    {


        if (!pausecam) {
            cam.waitForImage();
        }
        //stereo.rectify(cam.frameL, cam.frameR);

        //dtrk.track(stereo.frameLrect,stereo.frameRrect);
        //dctrl.control(dtrk.data);
        //resFrame = dtrk.resFrame;
#if defined(HASSCREEN) || defined(VIDEORESULTS)		
#ifdef HASSCREEN
        //cv::imshow("Results", resFrame);

#endif
#ifdef VIDEORESULTS
        outputVideoResults.write(resFrame);
#endif
#endif
#ifdef VIDEORAW
        outputVideoRawL.write(cam.frameL);
        outputVideoRawR.write(cam.frameR);
#endif
#ifdef VIDEODISPARITY
        outputVideoDisp.write(cam.get_disp_frame());
#endif
#ifdef VIDEOSTREAM
    prepare_buffer((GstAppSrc*)appsrc,&cam.frameL);
    g_main_context_iteration(g_main_context_default(),FALSE);
#endif

        handleKey();

        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        std::cout << "Frame: " <<imgcount << ". FPS: " << imgcount / time << std::endl;

    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    mouseX = x;
    mouseY = y;

    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        mouseLDown=1;
    } else if  ( event == cv::EVENT_LBUTTONUP ) {
        mouseLDown=0;
    } else if  ( event == cv::EVENT_RBUTTONDOWN ) {
        mouseRDown=1;
    } else if  ( event == cv::EVENT_RBUTTONUP ) {
        mouseRDown=0;
    } else if  ( event == cv::EVENT_MBUTTONDOWN ) {
        mouseMDown=1;
    } else if  ( event == cv::EVENT_MBUTTONUP ) {
        mouseMDown=0;
    } else if ( event == cv::EVENT_MOUSEMOVE )  {

    }
}

void handleKey() {

#ifdef HASSCREEN
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        cam.stopcam();
        return; // don't clear key, just exit
    }
#endif

    switch(key) {
    case 114: // [r]: reset stopwatch
        imgcount=0;
        stopWatch.Restart();
        msg="fps Reset";
        break;
    case ' ':
        pausecam=!pausecam;
        break;

    } // end switch key
#ifndef HASSCREEN
    if (key!=0) {
        std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

static gboolean bus_call (GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;

  switch (GST_MESSAGE_TYPE (msg)) {

    case GST_MESSAGE_EOS:
      std::cout << "Error: End of stream" << std::endl;
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;

    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;

      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);

      std::cout << "Error: " << error->message << std::endl;
      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);

      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }

  return TRUE;
}

static void prepare_buffer(GstAppSrc* appsrc, cv::Mat *image) {

  static GstClockTime timestamp = 0;
  GstBuffer *buffer;
  GstFlowReturn ret;

  if (!want) return;
  want = 0;

  gsize size = image->size().width * image->size().height*3 ;

  buffer = gst_buffer_new_allocate (NULL, size, NULL);
  GstMapInfo info;
  gst_buffer_map(buffer, &info, (GstMapFlags)GST_MAP_READ);
  memcpy(info.data, (guint8*)image->data, size);
  gst_buffer_unmap(buffer, &info);

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 4);

  timestamp += GST_BUFFER_DURATION (buffer);

  ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
  if (ret != GST_FLOW_OK) {
    std::cout << "GST ERROR DE PERROR" << std::endl;    
       
  }


}


static void cb_need_data (GstElement *appsrc, guint unused_size, gpointer user_data) {
  want = 1;
}

int initgst(int argc, char **argv) {
   GstElement *conv, *capsf, *encoder, *videosink;

  //for (int i = 0; i < 385*288; i++) { b_black[i] = 0; b_white[i] = 0xFFFF; }

  /* init GStreamer */
  gst_init (&argc, &argv);

  /* setup pipeline */
  //gst-launch-1.0 videotestsrc ! omxh264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! filesink location=test.h264  
  pipeline = gst_pipeline_new ("pipeline");
  appsrc = gst_element_factory_make ("appsrc", "source");
  conv = gst_element_factory_make ("videoconvert", "conv");
  encoder = gst_element_factory_make ("omxh264enc", "encoder");
  capsf = gst_element_factory_make ("capsfilter", "capsf");  
  videosink = gst_element_factory_make ("filesink", "videosink");
  

  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
  		gst_caps_new_simple ("video/x-raw",
				     "format", G_TYPE_STRING, "BGR",
				     "width", G_TYPE_INT, 1280,
				     "height", G_TYPE_INT, 960,
				     "framerate", GST_TYPE_FRACTION, 15, 1,NULL), NULL);
  g_object_set (G_OBJECT (capsf), "caps",
  		gst_caps_new_simple ("video/x-h264",
				     "stream-format", G_TYPE_STRING, "byte-stream", NULL), NULL);
  g_object_set (G_OBJECT (videosink), "location", "testv.h264", NULL);
  gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, encoder, capsf, videosink, NULL);
  gst_element_link_many (appsrc, conv, encoder, capsf, videosink, NULL);

  /* setup appsrc */
  g_object_set (G_OBJECT (appsrc),
		"stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
		"format", GST_FORMAT_TIME,
    "is-live", TRUE,
    NULL);
  g_signal_connect (appsrc, "need-data", G_CALLBACK (cb_need_data), NULL);

  /* play */
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  
  return 0;
}


int init(int argc, char **argv) {
initgst(argc,argv);

#ifdef _PC
    if (argc != 3) {
        std::cout << "Wrong arguments. Specify the location to load images..." << std::endl;
        return 1;
    }
    std::cout << "Loading images from: " << std::string(argv[1]) << std::endl;
    if (cam.init(std::string(argv[1]))) {
        return 1;
    }

    std::string calib_folder = std::string(argv[2]);
#else
    cam.init();
    std::string calib_folder = "/factory/";

#endif


    /*****Start capturing images*****/
    std::cout << "Start cam\n";
    cam.start();
    std::cout << "Started cam\n";
    /***init the stereo vision (groundtruth) algorithm ****/
    std::cout << "Initialising manual stereo algorithm\n";
    stereo.init(calib_folder);

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::resizeWindow("Results", 1280, 720); //makes it slower

    //cv::namedWindow("Results", CV_WINDOW_FULLSCREEN); // faster, but it is still windowed

    //real fullscreen opencv hack:
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::setWindowProperty("Results", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::setMouseCallback("Results", CallBackFunc, NULL);

#endif

#if defined(HASSCREEN) || defined(VIDEORESULTS)
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);
#endif


    /*****init the video writer*****/
#ifdef VIDEORESULTS    
    std::cout << "Opening video file for processed results at " << resFrame.cols << "x" << resFrame.rows << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeRes(resFrame.cols,resFrame.rows);
    outputVideoResults.open("videoResults.avi",CV_FOURCC('H','J','P','G'),cam.getFPS(),sizeRes,true);

    if (!outputVideoResults.isOpened())
    {
        std::cerr << "Output result video could not be opened!" << std::endl;
        return 1;
    }
#endif

#ifdef VIDEORAW
    std::cout << "Opening video file for raw video input at " << cam.getImWidth() << "x" << cam.getImHeight() << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeRaw(cam.getImWidth(),cam.getImHeight());
    outputVideoRawL.open("videoRawL.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeRaw,true);

    if (!outputVideoRawL.isOpened())
    {
        std::cerr << "Raw result video could not be opened!" << std::endl;
        return 1;
    }

    outputVideoRawR.open("videoRawR.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeRaw,true);

    if (!outputVideoRawR.isOpened())
    {
        std::cerr << "Raw result video could not be opened!" << std::endl;
        return 1;
    }
#endif
#ifdef VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    outputVideoDisp.open("videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!outputVideoDisp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif
    dtrk.init();
    dctrl.init();

    msg="";
    return 0;
}

void close() {

    /*****Close everything down*****/
    dtrk.close();
    cam.close();
    dctrl.close();


    /* clean up */
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline));

}




int main( int argc, char **argv )
{
    if (init(argc,argv)) {return 1;}
    process_video();
    close();

    return 0;
}


