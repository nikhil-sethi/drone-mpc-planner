#include "filecam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "third_party/stopwatch.h"
#include <experimental/filesystem>

#include "stopwatch.h"
static stopwatch_c swc;
GstElement *_pipeline,*_appsink;


void FileCam::init () {
    if (!file_exist(video_fn)) {
        std::stringstream serr;
        serr << "cannot not find " << video_fn;
        throw my_exit(serr.str());
    }
    std::cout << "Reading video from " << video_fn << std::endl;
    init_gstream();

    if (file_exist(calib_rfn))
        camparams.deserialize(calib_rfn);
    else
        camparams.deserialize(calib_template_rfn);
    calibration();

    if (!file_exist(depth_map_rfn)) {
        //todo: make gui warning of this:
        std::cout << "Warning: could not find " << depth_map_rfn << std::endl;
        depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
        depth_background = 10000; // basically disable the depth background map if it is not found
    } else {
        depth_background = cv::imread(depth_map_rfn,cv::IMREAD_ANYDEPTH);
    }

    convert_depth_background_to_world();

    def_volume();
    read_frame_ids();
    swc.Start();
    update();

    initialized = true;
}

void FileCam::read_frame_ids() {
    string framesfile = replay_dir + "/frames.csv";
    ifstream infile(framesfile);
    string line;
    while (getline(infile, line)) {
        try {
            auto data = logging::split_csv_line(line);
            frame_id_entry entry;
            entry.raw_video_frame_counter = stoi(data.at(0));
            entry.imgcount = stoi(data.at(1));
            entry.RS_id = stol(data.at(2));
            entry.time = stod(data.at(3));
            frames_ids.push_back(entry);
        } catch (exception& exp ) {
            throw my_exit("Could not read log! File: " +framesfile + '\n' + "Line: " + string(exp.what()) + " at: " + line);
        }
    }
    infile.close();
}

void FileCam::calibration() {
    intr = new rs2_intrinsics();
    intr->fx = camparams.fx;
    intr->fy = camparams.fy;
    intr->ppx = camparams.ppx;
    intr->ppy = camparams.ppy;
    intr->height = camparams.height;
    intr->width = camparams.width;
    intr->model = static_cast<rs2_distortion>(camparams.model);
    intr->coeffs[0] = camparams.coeffs[0];
    intr->coeffs[1] = camparams.coeffs[1];
    intr->coeffs[2] = camparams.coeffs[2];
    intr->coeffs[3] = camparams.coeffs[3];
    intr->coeffs[4] = camparams.coeffs[4];

    float focal_length = camparams.fx; // same as fy
    float cx = camparams.ppx; // same for both cameras
    float cy = camparams.ppy;
    float baseline = camparams.baseline;
    baseline = fabs(baseline);
    Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
}

void FileCam::update() {
    GstSample * sample;
    for (uint i = 0; i < replay_skip_n_frames+1; i++) {
        sample = gst_app_sink_pull_sample(GST_APP_SINK(_appsink));
        frame_cnt++;
    }
    replay_skip_n_frames = 0;

    auto buffer = gst_sample_get_buffer(sample);
    if (buffer == NULL || gst_app_sink_is_eos (reinterpret_cast<GstAppSink *>(_appsink)) ) {
        std::cout << "Log end, exiting. Video frames left: " << nFrames - frame_cnt << std::endl;
        gst_sample_unref(sample);
        throw bag_video_ended();
    }

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat frameLR(cv::Size(im_width, im_height), CV_8UC1, map.data, cv::Mat::AUTO_STEP);
    if (im_width > im_height) {
        frameLR(cv::Rect(cv::Point(0,0),cv::Point(frameLR.cols/2,frameLR.rows))).copyTo(frameL);
        frameLR(cv::Rect(cv::Point(frameLR.cols/2,0),cv::Point(frameLR.cols,frameLR.rows))).copyTo(frameR);
    } else {
        frameLR(cv::Rect(cv::Point(0,0),cv::Point(frameLR.cols,frameLR.rows/2))).copyTo(frameL);
        frameLR(cv::Rect(cv::Point(0,frameLR.rows/2),cv::Point(frameLR.cols,frameLR.rows))).copyTo(frameR);
    }
    _frame_number = frames_ids.at(frame_cnt-1).RS_id;
    _frame_time = frames_ids.at(frame_cnt-1).time;
    if (_frame_number==ULONG_MAX) {
        std::cout << "Log end, exiting. Video frames left: " << nFrames - frame_cnt << std::endl;
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        throw bag_video_ended();
    }
    if (!turbo) {
        while(swc.Read() < (1.f/pparams.fps)*1e3f) {
            usleep(1000);
        }
        swc.Restart();
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    while(frame_by_frame) {
        unsigned char k = cv::waitKey(0);
        if (k == 'f')
            break;
        else if (k== ' ') {
            frame_by_frame = false;
            break;
        }
    }
}

static void on_pad_added (GstElement *element, GstPad *src_pad, gpointer data)
{
    gchar *name_src,*name_sink;
    GstElement *other = static_cast<GstElement *>(data);
    GstPad *sink_pad = gst_element_get_static_pad (other, "sink");

    name_src = gst_pad_get_name (src_pad);
    name_sink = gst_pad_get_name (sink_pad);
    g_print ("A new src pad %s was created for %s\n", name_src, gst_element_get_name(element));
    g_print ("A new sink pad %s was created to %s\n", name_sink, gst_element_get_name(other));
    g_free (name_src);

    gst_pad_link (src_pad, sink_pad);

    gst_object_unref (sink_pad);

}

void FileCam::init_gstream() {
    //export GST_DEBUG=4 #excecute in terminal for extra debug info


    //pipeline to be created (not all elements are necessary when working through the api):
    //gst-launch-1.0 filesrc location=XXX ! matroskademux ! h265parse ! vaapih265dec ! videoconvert ! video/x-raw,format=GRAY8 ! videoconvert ! appsink;

    gst_init (NULL, NULL);

    GstElement *src = gst_element_factory_make ("filesrc", "filesrc");
    g_object_set (G_OBJECT (src), "location", video_fn.c_str(), NULL);

    _appsink = gst_element_factory_make ("appsink", "sink");
    g_object_set (G_OBJECT (_appsink), "sync",FALSE,"max-buffers", 10,NULL);

    _pipeline = gst_pipeline_new ("pipeline");


#if USE_DECODEBIN // easier, but seems to be slightly slower
    GstElement *decodebin;
    decodebin = gst_element_factory_make ("decodebin","decodebin");
    g_signal_connect (decodebin, "pad-added", G_CALLBACK (on_pad_added), _appsink);
    gst_bin_add_many (GST_BIN (_pipeline), src,decodebin,_appsink, NULL);
    gst_element_link_many (                src,decodebin,_appsink, NULL);

#else
    GstElement  *demux,*decoder;
    demux = gst_element_factory_make ("matroskademux", "demux");
    decoder = gst_element_factory_make ("vaapih265dec", "decoder");
    g_signal_connect (demux, "pad-added", G_CALLBACK (on_pad_added), decoder);

    gst_bin_add_many (GST_BIN (_pipeline), src,demux,decoder,_appsink, NULL);
    gst_element_link (src, demux);
    gst_element_link_many (decoder,_appsink, NULL);

#endif

    gst_element_set_state (_pipeline, GST_STATE_PLAYING);


    auto sample = gst_app_sink_pull_sample(GST_APP_SINK(_appsink));
    frame_cnt = 1;
    auto caps  = gst_sample_get_caps (sample);

    auto structure = gst_caps_get_structure(caps, 0);
    im_width = g_value_get_int(gst_structure_get_value(structure, "width"));
    im_height = g_value_get_int(gst_structure_get_value(structure, "height"));
    gst_caps_unref(caps);

    //for duration need to connect to on_discovered_cb and then use gst_discoverer_info_get_duration

    std::cout << "FileCam opened: " << video_fn <<  std::endl;
    std::cout << "Size: " <<  im_width << " x " << im_height << std::endl;


    gst_sample_unref(sample);

    // for benchmarking only:

    // cv::namedWindow("test", cv::WINDOW_OPENGL | cv::WINDOW_AUTOSIZE);
    // stopwatch_c sw_test;
    // sw_test.Start();
    // int frame_cnt = 0;
    // while (true) {
    //     sample = gst_app_sink_pull_sample(GST_APP_SINK(_appsink));
    //     auto buffer = gst_sample_get_buffer (sample);


    //     frame_cnt++;
    //     std::cout << frame_cnt << " fps: " << frame_cnt / (sw_test.Read()/1000.f) << std::endl;

    //     if (buffer == NULL) {
    //         exit(0);
    //     }

    //     if (frame_cnt %20 == 1) {
    //         GstMapInfo map;
    //         gst_buffer_map(buffer, &map, GST_MAP_READ);
    //         cv::Mat frame(cv::Size(im_width, im_height), CV_8UC1, map.data, cv::Mat::AUTO_STEP);
    //         cv::imshow("test",frame);
    //         cv::waitKey(1);
    //         gst_buffer_unmap(buffer, &map);
    //     }
    //     gst_sample_unref(sample);
    // }

}


void FileCam::close () {
    // gst_element_set_state (_pipeline, GST_STATE_NULL);
    // gst_object_unref (GST_OBJECT (_pipeline));
    Cam::close();
}
