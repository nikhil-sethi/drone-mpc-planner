#include "gstream.h"
#include "defines.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>

#include <iostream>

#include "stopwatch.h"
#include "common.h"

stopwatch_c stopwatch;
int want = 1;
int want_cnt = 0;
std::mutex lock_var;

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
cv::VideoWriter cvvideo;
int videomode;
int colormode;

static void cb_need_data (GstElement *appsrc __attribute__((unused)), guint unused_size __attribute__((unused)), gpointer user_data __attribute__((unused))) {
    lock_var.lock();
    want = 1;
    lock_var.unlock();
    want_cnt++;
}

int GStream::init(int argc, char **argv, int mode, std::string file, int sizeX, int sizeY,int fps, std::string ip, int port, bool color) {
    videomode = mode;
    gstream_fps  =fps;
    if (videomode == VIDEOMODE_STREAM ) {
        if (stream_resize_f > 1) {
            sizeX = sizeX/stream_resize_f;
            sizeY = sizeY/stream_resize_f;
        }
        if (max_gstream_fps < fps) {
            gstream_fps = max_gstream_fps;
        }
    }

    _cols = sizeX;
    _rows = sizeY;

    colormode = color;

    if (mode == VIDEOMODE_AVI_OPENCV) {

        std::cout << "Opening video file for processed results at " << sizeX << "x" << sizeY << " pixels with " << fps << "fps " << std::endl;
        cv::Size sizeRes(sizeX,sizeY);
        cvvideo.open(file,CV_FOURCC('X','2','6','4'),fps,sizeRes,color);
        if (!cvvideo.isOpened())
        {
            std::cerr << "Output result video could not be opened!" << std::endl;
            return 1;
        }
        return 0;

    } else {

        GstElement *conv, *capsfilter,*encoder, *mux, *rtp, *videosink;

        /* init GStreamer */
        gst_init (&argc, &argv);

        /* setup pipeline */
        if (mode == VIDEOMODE_AVI) {
            //write to file:
            //gst-launch-1.0 videotestsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=RGB,framerate=\(fraction\)15/1,width=1920,height=1080 ! videoconvert ! x264enc speed-preset=ultrafast bitrate=16000 ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1280,height=720 ! videoconvert ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            _pipeline = gst_pipeline_new ("pipeline");

            _appsrc = gst_element_factory_make ("appsrc", "source");
            g_object_set (G_OBJECT (_appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          NULL);

            g_signal_connect (_appsrc, "need-data", G_CALLBACK(cb_need_data), NULL);
            if (color) {
                g_object_set (G_OBJECT (_appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "BGR",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL), NULL);
            } else {
                g_object_set (G_OBJECT (_appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "GRAY8",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL), NULL);
            }

            conv = gst_element_factory_make ("videoconvert", "conv");

            //for compatibility with play back on e.g. phones, we need to have yuv420p
            //this will make the color rather ugly though :(
            capsfilter = gst_element_factory_make ("capsfilter", NULL);
            if (color) {
                g_object_set (G_OBJECT (capsfilter), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "I420",
                                                   NULL), NULL);
            }

            //the vaapi encoder uses way less CPU, but with the result is larger and there are more artifcats then the x264enc
            //            encoder = gst_element_factory_make ("vaapih264enc", "encoder"); // hardware encoding
            encoder = gst_element_factory_make ("x264enc", "encoder");
            g_object_set (G_OBJECT (encoder),  "speed-preset", 4,"bitrate", 32000, NULL); // higher quality soft encoder

            //preferably we would end up with an mp4 file, because phones understand that, but for some reason the stream is not valid when playing it back
            // parse = gst_element_factory_make ("h264parse", "parse");
            //mux = gst_element_factory_make ("mp4mux", "mux");
            //file += ".mp4";

            mux = gst_element_factory_make ("avimux", "mux");
            videosink = gst_element_factory_make ("filesink", "videosink");
            g_object_set (G_OBJECT (videosink), "location", file.c_str(), NULL);

            gst_bin_add_many (GST_BIN (_pipeline), _appsrc, conv, capsfilter,encoder,  mux, videosink, NULL);
            gst_element_link_many (_appsrc, conv, capsfilter, encoder, mux, videosink, NULL);

        } else if (mode == VIDEOMODE_STREAM) {
            //streaming:
            //from: gst-launch-1.0 videotestsrc pattern=snow ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1696,height=484 ! videoconvert ! videorate ! video/x-raw,framerate=15/1 ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5000
            //to: gst-launch-1.0 udpsrc port=5004 ! 'application/x-rtp, encoding-name=H264, payload=96' ! queue2 max-size-buffers=1 ! rtph264depay ! avdec_h264 ! videoconvert ! xvimagesink sync=false
            //to test fps: fpsdisplaysink
            _pipeline = gst_pipeline_new ("pipeline");

            _appsrc = gst_element_factory_make ("appsrc", "source");
            g_object_set (G_OBJECT (_appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          NULL);
            g_signal_connect (_appsrc, "need-data", G_CALLBACK(cb_need_data), NULL);
            if (color) {
                g_object_set (G_OBJECT (_appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "BGR",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL), NULL);
            } else {
                g_object_set (G_OBJECT (_appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "GRAY8",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL), NULL);
            }

            GstElement *rate = gst_element_factory_make("videorate", "videorate-element");
            GstElement *caps_rate = gst_element_factory_make("capsfilter", "videorate-caps");
            g_object_set (G_OBJECT (caps_rate), "caps",
                          gst_caps_new_simple ("video/x-raw", "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL), NULL);

            conv = gst_element_factory_make ("videoconvert", "conv");

#ifdef _PC
            //                        encoder = gst_element_factory_make ("x264enc", "encoder");
            //                        g_object_set (G_OBJECT (encoder),  "speed-preset", 1 ,"bitrate", 2048, "tune", 0x00000004,NULL);
            encoder = gst_element_factory_make ("vaapih264enc", "encoder");
#else
            encoder = gst_element_factory_make ("x264enc", "encoder");
#endif

            rtp = gst_element_factory_make ("rtph264pay", "rtp");

            videosink = gst_element_factory_make ("udpsink", "videosink");
            g_object_set (G_OBJECT (videosink), "host", ip.c_str(), "port", port, NULL);

            gst_bin_add_many (GST_BIN (_pipeline), _appsrc, rate,caps_rate ,conv, encoder, rtp, videosink, NULL);
            gst_element_link_many (_appsrc, rate,caps_rate ,conv, encoder,rtp, videosink, NULL);
        }


        /* play */
        gst_element_set_state (_pipeline, GST_STATE_PLAYING);

        if (mode != VIDEOMODE_STREAM) {
            if (!checkFileExist(file)) {
                std::cout << "Error creating video file: " << file << std::endl << "Does the folder exist?" << std::endl;
                return 1;
            }
        }

        stopwatch.Start();
        return 0;
    }
}

int GStream::prepare_buffer(GstAppSrc* appsrc, cv::Mat *image) {

    static GstClockTime timestamp = 0;
    GstBuffer *buffer;
    GstFlowReturn ret;

    lock_var.lock();
    if (!want) {
        lock_var.unlock();
        return 1;
    }

    int cmult = 1;
    if (colormode) {
        cmult =3;
    }
    gsize size = image->size().width * image->size().height*cmult ;

    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    memcpy(info.data, image->data, size);
    gst_buffer_unmap(buffer, &info);

    GST_BUFFER_PTS (buffer) = timestamp;
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, gstream_fps);

    timestamp += GST_BUFFER_DURATION (buffer);

    ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
    want = 0;
    lock_var.unlock();
    if (ret != GST_FLOW_OK) {
        std::cout << "GST ERROR DE PERROR" << std::endl;
        return 2;

    }
    return 0;
}

int GStream::write(cv::Mat frameL,cv::Mat frameR) {
    cv::Mat frame(frameL.rows,frameL.cols+frameR.cols,CV_8UC1);

    frameL.copyTo(frame(cv::Rect(0,0,frameL.cols, frameL.rows)));
    frameR.copyTo(frame(cv::Rect(frameL.cols,0,frameR.cols, frameR.rows)));

    if (videomode == VIDEOMODE_AVI_OPENCV) {
        cvvideo.write(frame);
        return 0;
    }
    else {
        int res = prepare_buffer(reinterpret_cast<GstAppSrc*>(_appsrc),&frame);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

int GStream::write(cv::Mat frame) {
    if (frame.empty())
        return 1;
    cv::Mat tmpframe = frame.clone();
    if (videomode == VIDEOMODE_STREAM && stream_resize_f > 1) {
        cv::resize(tmpframe,tmpframe,cv::Size(frame.cols/stream_resize_f,frame.rows/stream_resize_f));
    }
    if (tmpframe.cols != _cols || tmpframe.rows != _rows) {
        std::cout << "Warning: video sizes don't match in recorder!?" << std::endl;
    }
    if (videomode == VIDEOMODE_AVI_OPENCV) {
        cvvideo.write(tmpframe);
        return 0;
    } else {
        int res = prepare_buffer(reinterpret_cast<GstAppSrc*>(_appsrc),&tmpframe);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

void GStream::close () {
    if (videomode != VIDEOMODE_AVI_OPENCV) {
        gst_element_set_state (_pipeline, GST_STATE_NULL);
        gst_object_unref (GST_OBJECT (_pipeline));
    }
    std::cout << "Average gstream video (processing) fps: " << want_cnt / (stopwatch.Read()*0.001f) << std::endl;
    //max_stream_fps can be lower!
}
