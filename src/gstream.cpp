#include "gstream.h"
#include "defines.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>

#include <iostream>

int want = 1;

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
cv::VideoWriter cvvideo;
int videomode;
int colormode;

static void cb_need_data (GstElement *appsrc, guint unused_size, gpointer user_data) {
    want = 1;
}

int GStream::getWanted(void) {
    return want;
}

int GStream::init(int argc, char **argv, int mode, std::string file, int sizeX, int sizeY,int fps, std::string ip, int port, bool color) {
    videomode = mode;
    colormode = color;

    if (mode == VIDEOMODE_AVI_OPENCV) {

        std::cout << "Opening video file for processed results at " << sizeX << "x" << sizeY << " pixels with " << fps << "fps " << std::endl;
        cv::Size sizeRes(sizeX,sizeY);
        cvvideo.open(file,CV_FOURCC('F','M','P','4'),fps,sizeRes,color);
        if (!cvvideo.isOpened())
        {
            std::cerr << "Output result video could not be opened!" << std::endl;
            return 1;
        }
        return 0;

    } else {

        GstElement *conv, *rate, *encoder, *mux, *rtp, *videosink;

        /* init GStreamer */
        gst_init (&argc, &argv);

        /* setup pipeline */
        if (mode == VIDEOMODE_AVI) {
#ifdef _PC
            //write to file:
            //gst-launch-1.0 videotestsrc ! videocovert ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1280,height=720 ! videoconvert ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            pipeline = gst_pipeline_new ("pipeline");
            appsrc = gst_element_factory_make ("appsrc", "source");
            conv = gst_element_factory_make ("videoconvert", "conv");
            encoder = gst_element_factory_make ("x264enc", "encoder");
            mux = gst_element_factory_make ("avimux", "mux");
            videosink = gst_element_factory_make ("filesink", "videosink");

            /* setup */
            if (color) {
                g_object_set (G_OBJECT (appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "BGR",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, VIDEOFPS, 1,NULL), NULL);
            } else {
                g_object_set (G_OBJECT (appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "GRAY8",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, VIDEOFPS, 1,NULL), NULL);
            }
//            g_object_set (G_OBJECT (capsf), "caps",
//                          gst_caps_new_simple ("video/x-h264",
//                                               "stream-format", G_TYPE_STRING, "byte-stream", NULL), NULL);
            g_object_set (G_OBJECT (videosink), "location", file.c_str(), NULL);
            gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, encoder,  mux, videosink, NULL);
            gst_element_link_many (appsrc, conv, encoder,  mux, videosink, NULL);

            /* setup appsrc */
            g_object_set (G_OBJECT (appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          NULL);
#else
            //write to file:
            //gst-launch-1.0 videotestsrc ! omxh264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            pipeline = gst_pipeline_new ("pipeline");
            appsrc = gst_element_factory_make ("appsrc", "source");
            conv = gst_element_factory_make ("videoconvert", "conv");
            encoder = gst_element_factory_make ("omxh264enc", "encoder");
            capsf = gst_element_factory_make ("capsfilter", "capsf");
            mux = gst_element_factory_make ("avimux", "mux");
            videosink = gst_element_factory_make ("filesink", "videosink");

            /* setup */
            g_object_set (G_OBJECT (appsrc), "caps",
                          gst_caps_new_simple ("video/x-raw",
                                               "format", G_TYPE_STRING, "BGR",
                                               "width", G_TYPE_INT, sizeX,
                                               "height", G_TYPE_INT, sizeY,
                                               "framerate", GST_TYPE_FRACTION, VIDEOFPS, 1,NULL), NULL);
            g_object_set (G_OBJECT (capsf), "caps",
                          gst_caps_new_simple ("video/x-h264",
                                               "stream-format", G_TYPE_STRING, "byte-stream", NULL), NULL);
            g_object_set (G_OBJECT (videosink), "location", file.c_str(), NULL);
            gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, encoder, capsf, mux, videosink, NULL);
            gst_element_link_many (appsrc, conv, encoder, capsf, mux, videosink, NULL);

            /* setup appsrc */
            g_object_set (G_OBJECT (appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          NULL);
#endif

        } else if (mode == VIDEOMODE_STREAM) {
            //streaming:
            //from: gst-launch-1.0 videotestsrc pattern=snow ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1696,height=484 ! videoconvert ! videorate ! video/x-raw,framerate=15/1 ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5000
            //to: gst-launch-1.0 udpsrc port=5000 ! 'application/x-rtp, encoding-name=H264, payload=96' ! queue2 max-size-buffers=1 ! rtph264depay ! avdec_h264 ! videoconvert ! xvimagesink sync=false
            pipeline = gst_pipeline_new ("pipeline");
            appsrc = gst_element_factory_make ("appsrc", "source");
            conv = gst_element_factory_make ("videoconvert", "conv");
            rate = gst_element_factory_make ("videorate", "rate");
            encoder = gst_element_factory_make ("x264enc", "encoder");
            rtp = gst_element_factory_make ("rtph264pay", "rtp");
            videosink = gst_element_factory_make ("udpsink", "videosink");

            /* setup */
            if (color) {
                g_object_set (G_OBJECT (appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "BGR",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, VIDEOFPS, 1,NULL), NULL);
            } else {
                g_object_set (G_OBJECT (appsrc), "caps",
                              gst_caps_new_simple ("video/x-raw",
                                                   "format", G_TYPE_STRING, "GRAY8",
                                                   "width", G_TYPE_INT, sizeX,
                                                   "height", G_TYPE_INT, sizeY,
                                                   "framerate", GST_TYPE_FRACTION, VIDEOFPS, 1,NULL), NULL);
            }
            g_object_set (G_OBJECT (rate), "caps",
                          gst_caps_new_simple ("video/x-raw",
                                               "framerate", GST_TYPE_FRACTION, 15, 1,NULL), NULL);
            g_object_set (G_OBJECT (encoder), "caps",
                          gst_caps_new_simple ("video/x-h264",
                                               "stream-format", G_TYPE_STRING, "byte-stream", NULL), NULL);
            g_object_set (G_OBJECT (videosink), "host", ip.c_str(), "port", port, NULL);

            gst_bin_add_many (GST_BIN (pipeline), appsrc, rate, conv, encoder, rtp, videosink, NULL);
            gst_element_link_many (appsrc, rate, conv, encoder, rtp, videosink, NULL);

            /* setup appsrc */
            g_object_set (G_OBJECT (appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          NULL);
        }

        g_signal_connect (appsrc, "need-data", G_CALLBACK(cb_need_data), NULL);

        /* play */
        gst_element_set_state (pipeline, GST_STATE_PLAYING);

        return 0;
    }
}

int GStream::prepare_buffer(GstAppSrc* appsrc, cv::Mat *image) {

    static GstClockTime timestamp = 0;
    GstBuffer *buffer;
    GstFlowReturn ret;

    if (!want) return 1;
    want = 0;

    int cmult = 1;
    if (colormode) {
        cmult =3;
    }
    gsize size = image->size().width * image->size().height*cmult ;

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
        int res = prepare_buffer((GstAppSrc*)appsrc,&frame);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

int GStream::write(cv::Mat frame) {
    if (videomode == VIDEOMODE_AVI_OPENCV) {
        cvvideo.write(frame.clone());
        return 0;
    }
    else {
        int res = prepare_buffer((GstAppSrc*)appsrc,&frame);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

void GStream::close () {
    if (videomode != VIDEOMODE_AVI_OPENCV) {
        gst_element_set_state (pipeline, GST_STATE_NULL);
        gst_object_unref (GST_OBJECT (pipeline));
    }
}
