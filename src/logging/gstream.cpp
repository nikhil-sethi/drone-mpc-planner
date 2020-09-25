#include "gstream.h"
#include "third_party/stopwatch.h"
#include "common.h"

#include <gst/app/gstappsrc.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
cv::VideoWriter cvvideo;
int videomode;
int bgr_mode;

int enough = 0;
int want = 1;
int want_cnt = 0;
std::mutex lock_var;
std::mutex wait_for_want;


static void cb_need_data (GstElement *appsrc __attribute__((unused)), guint unused_size __attribute__((unused)), gpointer user_data __attribute__((unused))) {
    lock_var.lock();
    want = 1;
    enough = 0;
    lock_var.unlock();
    want_cnt++;
    wait_for_want.unlock();
}

static void cb_enough_data (GstElement *appsrc __attribute__((unused)), guint unused_size __attribute__((unused)), gpointer user_data __attribute__((unused))) {
    enough = 1;
}

void GStream::block() {
    wait_for_want.lock();
}
void GStream::manual_unblock() {
    wait_for_want.unlock();
}

int GStream::init(int mode, std::string file, int sizeX, int sizeY,int fps, std::string ip, int port, bool color) {
    videomode = mode;
    gstream_fps  =fps;
    _file = file;
    wait_for_want.unlock();
    if (videomode == video_stream ) {
        if (stream_resize_f > 1) {
            sizeX = sizeX/stream_resize_f;
            sizeY = sizeY/stream_resize_f;
        }
    }

    _cols = sizeX;
    _rows = sizeY;

    bgr_mode = color;

    if (mode == video_mp4_opencv) {

        std::cout << "Opening video file for processed results at " << sizeX << "x" << sizeY << " pixels with " << fps << "fps " << std::endl;
        cv::Size sizeRes(sizeX,sizeY);
        cvvideo.open(file,cv::VideoWriter::fourcc('X','2','6','4'),fps,sizeRes,color);
        if (!cvvideo.isOpened())
        {
            std::cerr << "Output result video could not be opened!" << std::endl;
            return 1;
        }
        initialised = true;
        return 0;

    } else {

        GstElement *capsfilter,*conv,*encoder, *mux, *rtp, *videosink, *parse;
        GstElement *colorbalance,*videoconvert;
        /* init GStreamer */
        gst_init (NULL, NULL);

        /* setup pipeline */
        if (mode == video_mkv) {
            //write to file:
            //gst-launch-1.0 videotestsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=RGB,framerate=\(fraction\)15/1,width=1920,height=1080 ! videoconvert ! x264enc speed-preset=ultrafast bitrate=16000 ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1280,height=720 ! videoconvert ! x264enc ! 'video/x-h264, stream-format=(string)byte-stream'  ! avimux ! filesink location=test.avi
            //gst-launch-1.0 videotestsrc ! video/x-raw,format=GRAY8,framerate=\(fraction\)90/1,width=1696,height=480 ! videoconvert ! vaapih265enc ! h265parse ! matroskamux ! filesink location=test.mkv

            _pipeline = gst_pipeline_new ("pipeline");

            _appsrc = gst_element_factory_make ("appsrc", "source");
            videoconvert = gst_element_factory_make ("videoconvert", "videoconvert");
            capsfilter = gst_element_factory_make ("capsfilter", NULL);
            encoder = gst_element_factory_make ("vaapih265enc", "encoder"); // hardware encoding
            parse = gst_element_factory_make ("h265parse", "parse");
            if (file.back() == '4')
                mux = gst_element_factory_make ("mp4mux", "mux");
            else
                mux = gst_element_factory_make ("matroskamux", "mux");

            videosink = gst_element_factory_make ("filesink", "videosink");


            g_object_set (G_OBJECT (_appsrc),
                          "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                          "format", GST_FORMAT_TIME,
                          "is-live", TRUE,
                          "max-bytes", 5000000, // buffer size before enough-data fires. Default 200000
                          NULL);
            g_signal_connect (_appsrc, "need-data", G_CALLBACK(cb_need_data), NULL);
            g_signal_connect (_appsrc, "enough-data", G_CALLBACK(cb_enough_data), NULL);
            g_object_set (G_OBJECT (videosink), "location", file.c_str(), NULL);


            if(bgr_mode) {
                //at the moment the pipeline below does not work on the basestations because the gl elements do understnad the X situation.
                // Swapping with normal videoconvert probably solves the problem, at the cost of cpu usage. Since the color videos atm are only
                //for rendering afterwards I leave it like this. Hopefully the main-444 profile will be supported in vaapih265enc soon and the gl
                //components are less needed anyway.
                auto caps = gst_caps_new_simple ("video/x-raw",
                                                 "format", G_TYPE_STRING, "BGR",
                                                 "width", G_TYPE_INT, sizeX,
                                                 "height", G_TYPE_INT, sizeY,
                                                 "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL);
                g_object_set (G_OBJECT (_appsrc), "caps",
                              caps, NULL);
                gst_caps_unref(caps);

                // the colorspace conversion to I420 doesn't play nice with our viz. So up the saturation so it looks a bit the same as before.

                colorbalance = gst_element_factory_make ("videobalance", "videobalance");
                g_object_set (G_OBJECT (colorbalance), "brightness", 0.15, "contrast", 1.15,"saturation",2.0, NULL);

                // vaapih265enc in gstreamer 1.17 supports main-444 which looks much better for our colored vizs. But currently we are on gstreamer 1.14 and often players dont support 444 either.
                //(this is why we hack it with the colorbalance above)
                g_object_set (G_OBJECT (capsfilter), "caps",
                              gst_caps_new_simple ("video/x-h265",
                                                   "profile", G_TYPE_STRING, "main",
                                                   NULL), NULL);

                gst_bin_add_many (GST_BIN (_pipeline), _appsrc,colorbalance,videoconvert,encoder,capsfilter,parse,mux,videosink, NULL);
                gst_element_link_many (                _appsrc,colorbalance,videoconvert,encoder,capsfilter,parse,mux,videosink, NULL);

            } else {
                auto caps = gst_caps_new_simple ("video/x-raw",
                                                 "format", G_TYPE_STRING, "I420",
                                                 "width", G_TYPE_INT, sizeX,
                                                 "height", G_TYPE_INT, sizeY,
                                                 "framerate", GST_TYPE_FRACTION, gstream_fps, 1,NULL);
                g_object_set (G_OBJECT (_appsrc), "caps",
                              caps, NULL);
                gst_caps_unref(caps);

                //The cqp rate-control setting seems to leave noticable noice, so we set a fixed bitrate. 5000 seems to be a nice compromise between quality and size.
                //For logging (with stringent size and download constraints), cqp could be better though. It is about 5x smaller and the noise does not really influence our algorithms.
                //To have a similar size as the intel rs bag one would need to increase to 5000000 (5M), but they are using mjpeg which is much less efficient
                g_object_set (G_OBJECT (encoder),  "rate-control", 2,"bitrate", 5000, NULL);

                gst_bin_add_many (GST_BIN (_pipeline),_appsrc,encoder,mux,videosink,NULL);
                gst_element_link_many (_appsrc,encoder,mux,videosink,NULL);
            }

        } else if (mode == video_stream) { // streaming has not be updated and tested for a while now...doubt it still works optimal
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

            //for compatibility with play back on e.g. phones, we need to have yuv420p
            //this does not seem to be necessary for streaming between computers
            capsfilter = gst_element_factory_make ("capsfilter", NULL);
            //if (color) {
            //    g_object_set (G_OBJECT (capsfilter), "caps",
            //                  gst_caps_new_simple ("video/x-raw",
            //                                       "format", G_TYPE_STRING, "I420",
            //                                       NULL), NULL);
            ///}

            encoder = gst_element_factory_make ("x264enc", "encoder");
            g_object_set (G_OBJECT (encoder),  "speed-preset", 1,"bitrate", 1000, NULL);
            // encoder = gst_element_factory_make ("vaapih264enc", "encoder");

            rtp = gst_element_factory_make ("rtph264pay", "rtp");

            videosink = gst_element_factory_make ("udpsink", "videosink");
            g_object_set (G_OBJECT (videosink), "host", ip.c_str(), "port", port, NULL);

            gst_bin_add_many (GST_BIN (_pipeline), _appsrc, rate,caps_rate,conv, capsfilter, encoder, rtp, videosink, NULL);
            gst_element_link_many (_appsrc, rate,caps_rate,conv, capsfilter, encoder,rtp, videosink, NULL);
        }

        /* play */
        gst_element_set_state (_pipeline, GST_STATE_PLAYING);

        if (mode != video_stream) {
            if (!file_exist(file)) {
                std::cout << "Error creating video file: " << file << std::endl << "Does the folder exist?" << std::endl;
                return 1;
            }
        }

        initialised = true;
        return 0;
    }
}

//accepts a grayscale stereo pair, which is converted to YUV I420 to be accepted by the vaapi encoder
int GStream::prepare_buffer(GstAppSrc* appsrc, cv::Mat frameL, cv::Mat frameR) {

    static GstClockTime timestamp = 0;
    GstBuffer *buffer;
    GstFlowReturn ret;

    lock_var.lock();
    if (enough) {
        std::cout << "Skip recording a frame because buffer is full" << std::endl;
        lock_var.unlock();
        return 1;
    }

    gsize size = frameL.cols * frameL.rows;

    buffer = gst_buffer_new_allocate (NULL, 3*size, NULL); //Would be nice if we could somehow use gst_buffer_new_wrapped_full instead to wrap existing memory
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_WRITE);
    memcpy(info.data, frameL.data, size);
    memcpy(info.data + size, frameR.data, size);
    memset(info.data + 2*size, 128, size); // this converts gray to YV12
    gst_buffer_unmap(buffer, &info);

    GST_BUFFER_PTS (buffer) = timestamp;
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, gstream_fps);

    timestamp += GST_BUFFER_DURATION (buffer);

    ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
    want = 0;
    lock_var.unlock();
    if (ret != GST_FLOW_OK) {
        std::cout << "GST ERROR DE PERROR from " << _file << std::endl;
        return 2;

    }
    return 0;
}

int GStream::prepare_buffer(GstAppSrc* appsrc, cv::Mat image) {

    static GstClockTime timestamp = 0;
    GstBuffer *buffer;
    GstFlowReturn ret;

    lock_var.lock();
    if (enough) {
        std::cout << "Skip recording a frame because buffer is full" << std::endl;
        lock_var.unlock();
        return 1;
    }

    int cmult = 1;
    if (bgr_mode) {
        cmult =3;
    }
    gsize size = image.size().width * image.size().height*cmult ;

    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_WRITE);
    memcpy(info.data, image.data, size);
    gst_buffer_unmap(buffer, &info);

    GST_BUFFER_PTS (buffer) = timestamp;
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, gstream_fps);

    timestamp += GST_BUFFER_DURATION (buffer);

    ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
    want = 0;
    lock_var.unlock();
    if (ret != GST_FLOW_OK) {
        std::cout << "GST ERROR DE PERROR from " << _file << std::endl;
        return 2;
    }
    return 0;
}

int GStream::write(cv::Mat frameL,cv::Mat frameR) {
    if (videomode == video_mp4_opencv) {
        cv::Mat frame(frameL.rows,frameL.cols+frameR.cols,CV_8UC1);

        frameL.copyTo(frame(cv::Rect(0,0,frameL.cols, frameL.rows)));
        frameR.copyTo(frame(cv::Rect(frameL.cols,0,frameR.cols, frameR.rows)));
        cvvideo.write(frame);
        return 0;
    } else {
        int res = prepare_buffer(reinterpret_cast<GstAppSrc*>(_appsrc),frameL,frameR);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

int GStream::write(cv::Mat frame) {
    if (frame.empty()) {
        wait_for_want.unlock();
        return 1;
    }
    cv::Mat tmpframe = frame.clone();

    if (videomode == video_stream && stream_resize_f > 1) {
        cv::resize(tmpframe,tmpframe,cv::Size(frame.cols/stream_resize_f,frame.rows/stream_resize_f));
    }
    if (tmpframe.cols != _cols || tmpframe.rows != _rows) {
        std::cout << "Warning: video sizes don't match in recorder!?" << std::endl;
    }
    if (videomode == video_mp4_opencv) {
        cvvideo.write(tmpframe);
        return 0;
    } else {
        int res = prepare_buffer(reinterpret_cast<GstAppSrc*>(_appsrc),tmpframe);
        g_main_context_iteration(g_main_context_default(),FALSE);
        return res;
    }
}

void GStream::close () {
    if (initialised) {
        std::cout << "Closing video recorder" << std::endl;
        if (videomode != video_mp4_opencv) {
            gst_element_send_event(_pipeline,gst_event_new_eos());
            GstClockTime timeout = 10 * GST_SECOND;
            GstMessage *msg;

            std::cout << "Waiting for EOS..." << std::endl;
            msg = gst_bus_timed_pop_filtered (GST_ELEMENT_BUS (_pipeline),
                                              timeout,static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

            if (msg == NULL) {
                std::cout << "Error: gstreamer videorecorder did not get an EOS after 10 seconds!" << std::endl;
            } else if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR) {
                std::cout << "Error: gstreamer" << msg->type <<  std::endl;
            }

            if (msg)
                gst_message_unref (msg);

            gst_element_set_state (_pipeline, GST_STATE_NULL);
            gst_object_unref (GST_OBJECT (_pipeline));
        }
        //max_stream_fps can be lower!
        initialised = false;
    }
}
