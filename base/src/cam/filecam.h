#pragma once
#include "common.h"
#include "cam.h"
#include "logreader.h"

#include <gst/app/gstappsink.h>

class FileCam : public Cam {
private:
    logging::LogReader * logreader;
    std::string video_fn;
    int im_width=0, im_height=0, nFrames=0,frame_cnt = 0;
    void read_frame_ids();
    void calibration();
    void init_gstream();
    std::string replay_dir;

    struct Frame_ID_Entry {
        long raw_video_frame_counter;
        long imgcount;
        long long rs_id;
        double time;
    };
    std::vector<Frame_ID_Entry> frames_ids;

public:
    static std::string playback_filename() { return "videoRawLR.mkv"; }
    FileCam(string replay_dir_, logging::LogReader * log) {
        replay_dir = replay_dir_;
        logreader = log;
        set_file_paths(replay_dir);
        video_fn = replay_dir + '/' + playback_filename();
    }
    FileCam(string replay_dir_,string monitor_video_fn) {
        video_fn = monitor_video_fn;
        replay_dir = replay_dir_;
        set_file_paths(replay_dir);
    }
    void skip(float duration) {
        //This function calculates how many frames need to be skip in order to just duration seconds.
        //This is complicated because there are two sources of error:
        //1. the realsense sometimes skips frames
        //2. the videowriter sometimes skips frames
        //But we have frames.csv in which all frame ids are written, including a frames_written_in_video cnt.
        //So we simply loop through this list to find where we are now, and where we need to be, and use the diff between
        //the two frames_written_in_video counnts
        auto frame = last();
        auto current_time = frame->time;
        uint video_id_now = 0;
        uint video_id_skip = 0;
        for (auto id : frames_ids) {
            if (id.time> static_cast<double>(duration)+current_time) {
                video_id_skip = id.raw_video_frame_counter;
                break;
            }
            if (id.time> current_time && !video_id_now)
                video_id_now = id.raw_video_frame_counter;
        }
        replay_skip_n_frames = video_id_skip - video_id_now;
    }

    void init();
    void close();
    void update();

    void back_one_sec() {}
};
