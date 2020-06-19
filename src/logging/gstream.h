#pragma once
#include "opencv2/features2d/features2d.hpp"
#include <gst/app/gstappsrc.h>
#include <condition_variable>

/*
 * This class wraps video writer / streamer through gstreamer
 *
 */
class GStream {

public:
private:
    GstElement *_pipeline,*_appsrc;

    int _cols,_rows;
    int prepare_buffer(GstAppSrc* appsrc, cv::Mat image);
    int prepare_buffer(GstAppSrc* appsrc, cv::Mat frameL, cv::Mat frameR);
    int stream_resize_f = 1;
    int gstream_fps;
    int max_gstream_fps = 30;
    bool initialised = false;


public:
    int init(int mode, std::string file, int sizeX, int sizeY, int fps, std::string ip, int port, bool color);
    int write(cv::Mat frame);
    int write(cv::Mat frameL,cv::Mat frameR);
    void close (void);
    void block();
};
