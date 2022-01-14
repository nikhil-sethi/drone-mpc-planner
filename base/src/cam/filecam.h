#pragma once
#include "common.h"
#include "cam.h"
#include "logreader.h"

#include <gst/app/gstappsink.h>

class FileCam : public Cam {
private:
    logging::LogReader *logreader;
    std::string video_fn;
    uint im_width = 0, im_height = 0, nFrames = 0, frame_cnt = 0;
    void read_frame_ids();
    void calibration();
    void init_gstream();
    std::string replay_dir;
    bool skip_first_frame = false;

    struct Frame_ID_Entry {
        long encoded_img_count;
        long imgcount;
        long long rs_id;
        double time;
    };
    std::vector<Frame_ID_Entry> frame_ids;

public:
    static std::string default_playback_filename() { return "videoRawLR.mkv"; }
    FileCam(string replay_dir_, logging::LogReader *log) {
        replay_dir = replay_dir_;
        logreader = log;
        set_read_file_paths(replay_dir);
        set_write_file_paths(data_output_dir);
        video_fn = replay_dir + '/' + default_playback_filename();
    }
    FileCam(string replay_dir_, string replay_video_fn) {
        video_fn = replay_video_fn;
        replay_dir = replay_dir_;
        set_read_file_paths(replay_dir);
        set_write_file_paths(data_output_dir);
    }
    void skip(float duration) {
        //This function calculates how many frames need to be skip in order to just duration seconds.
        //This is complicated because there are two sources of error:
        //1. the realsense sometimes skips frames
        //2. the videowriter sometimes skips frames
        //But we have frames.csv in which all frame ids are written, including a frames_written_in_video cnt.
        //So we simply loop through this list to find where we are now, and where we need to be, and use the diff between
        //the two frames_written_in_video counnts
        auto frame = current();
        auto current_time = frame->time;
        uint video_id_now = 0;
        uint video_id_skip = 0;
        for (auto id : frame_ids) {
            if (id.time > static_cast<double>(duration) + current_time) {
                video_id_skip = id.encoded_img_count;
                break;
            }
            if (id.time > current_time && !video_id_now)
                video_id_now = id.encoded_img_count;
        }
        replay_skip_n_frames = video_id_skip - video_id_now;
    }
    void start_cnt(int rs_id) {
        for (auto frame_id  : frame_ids) {
            if (frame_id.rs_id == rs_id) {
                frame_cnt = frame_id.encoded_img_count;
                break;
            }
        }
        update();
        skip_first_frame = true;
    }

    void init();
    void close();
    void inject_log(logging::LogEntryMain entry);
    StereoPair *update();

    void back_one_sec() {}
};
