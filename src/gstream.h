#ifndef GSTREAM_H
#define GSTREAM_H

#include "opencv2/features2d/features2d.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>

/*
 * This class wraps video writer / streamer through gstreamer
 *
 */
class GStream {


private:
    GstElement *_pipeline,*_appsrc;


    int _cols,_rows;
    int prepare_buffer(GstAppSrc* appsrc, cv::Mat * image);
   	//void cb_need_data (GstElement *appsrc, guint unused_size, gpointer user_data);

public:
    int init(int argc, char **argv, int mode, std::string file, int sizeX, int sizeY, int fps, std::string ip, int port, bool color);
    int write(cv::Mat frame);
    int write(cv::Mat frameL,cv::Mat frameR);
    void close (void);
};




#endif //GSTREAM_H
