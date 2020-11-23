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
        long long RS_id;
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

    void init();
    void close();
    void update();

    void back_one_sec() {}
};
