#include "vizs.h"
using namespace cv;
using namespace std;
using namespace tracking;

cv::Scalar white(255, 255, 255);
cv::Scalar black(0, 0, 0);
cv::Scalar green(60, 255, 60);
cv::Scalar blue(255, 60, 60);
cv::Scalar red(60, 60, 255);
cv::Scalar pink(147, 20, 255);
cv::Scalar linecolors[] = {green, blue, red, cv::Scalar(0, 255, 255), cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(128, 128, 255), cv::Scalar(255, 128, 128)};

cv::Scalar fore_color(255, 255, 255);
cv::Scalar background_color(0, 0, 0);

void Visualizer::init(VisionData *visdat, Patser *patser, bool fromfile) {
    _visdat = visdat;
    _patser = patser;

    _fromfile = fromfile;
    thread_viz = std::thread(&Visualizer::workerThread, this);
    initialized = true;
}

void Visualizer::add_plot_sample(void) {
    if (enable_plots) {
        lock_plot_data.lock();
        roll_joystick.push_back(static_cast<float>(_patser->drone.control.joy_roll));
        pitch_joystick.push_back(static_cast<float>(_patser->drone.control.joy_pitch));
        yaw_joystick.push_back(static_cast<float>(_patser->drone.control.joy_pitch));
        throttle_joystick.push_back(static_cast<float>(_patser->drone.control.joy_throttle));

        roll_calculated.push_back(static_cast<float>(_patser->drone.control.auto_roll));
        pitch_calculated.push_back(static_cast<float>(_patser->drone.control.auto_pitch));
        //    yaw_calculated.push_back(static_cast<float>(_patser->drone.control.commandedYaw));
        throttle_calculated.push_back(static_cast<float>(_patser->drone.control.auto_throttle));
        throttle_min_bound.push_back(static_cast<float>(RC_BOUND_MIN));
        throttle_max_bound.push_back(static_cast<float>(RC_BOUND_MAX));


        auto dtrkr = &_patser->drone.tracker;
        TrackData data = dtrkr->last_track_data();
        dt.push_back(data.dt);
        dt_target.push_back(1.f / pparams.fps);

        TrackData data_target = _patser->interceptor.target_last_trackdata();
        auto nav = &_patser->drone.nav;

        if (data.pos_valid) {
            posX_drone.push_back(-data.state.pos.x);
            posY_drone.push_back(data.state.pos.y);
            posZ_drone.push_back(-data.state.pos.z);
            im_posX_drone.push_back(dtrkr->image_item().x / im_scaler);
            im_posY_drone.push_back(dtrkr->image_item().y / im_scaler);
            im_size_drone.push_back(dtrkr->image_item().size / im_scaler);
            im_disp_drone.push_back(dtrkr->image_item().disparity);
            sposX.push_back(-data.state.spos.x);
            sposY.push_back(data.state.spos.y);
            sposZ.push_back(-data.state.spos.z);

            if (generator_cam_set) {
                gen_posX_drone.push_back(-generator_cam->generated_world_pos.x);
                gen_posY_drone.push_back(generator_cam->generated_world_pos.y);
                gen_posZ_drone.push_back(-generator_cam->generated_world_pos.z);
                gen_im_posX_drone.push_back(generator_cam->generated_im_pos.x / im_scaler);
                gen_im_posY_drone.push_back(generator_cam->generated_im_pos.y / im_scaler);
                gen_im_size_drone.push_back(generator_cam->generated_im_size / im_scaler);
                gen_im_disp_drone.push_back(generator_cam->generated_im_pos.z);
            }

            if (_patser->drone.control.Joy_State() != DroneController::js_hunt) {
                posX_target.push_back(-nav->setpoint().pos().x);
                posY_target.push_back(-nav->setpoint().pos().y);
                posZ_target.push_back(-nav->setpoint().pos().z);
            } else {
                posX_target.push_back(-data_target.state.pos.x);
                posY_target.push_back(data_target.state.pos.y);
                posZ_target.push_back(-data_target.state.pos.z);
            }

            setposX.push_back(-nav->setpoint().pos().x);
            setposY.push_back(nav->setpoint().pos().y);
            setposZ.push_back(-nav->setpoint().pos().z);
        }
        if (data.vel_valid) {
            svelX.push_back(-data.state.vel.x);
            svelY.push_back(data.state.vel.y);
            svelZ.push_back(-data.state.vel.z);

            velX.push_back(-data.state.vel_unfiltered.x);
            velY.push_back(data.state.vel_unfiltered.y);
            velZ.push_back(-data.state.vel_unfiltered.z);

        }
        if (data.acc_valid) {
            saccX.push_back(-data.state.acc.x);
            saccY.push_back(data.state.acc.y);
            saccZ.push_back(-data.state.acc.z);
        }


        lock_plot_data.unlock();
        newdata.notify_all();
    }
}

void Visualizer::plot(void) {
    std::vector<cv::Mat> ims_trk;
    // ims_trk.push_back(plot_xyd());
    ims_trk.push_back(plot_all_im_drone_pos());
    ims_trk.push_back(plot_all_position());
    ims_trk.push_back(plot_all_velocity());
    // ims_trk.push_back(plot_all_acceleration());
    // ims_trk.push_back(plot_all_control());
    plotframe = create_row_image(ims_trk, CV_8UC3);
}

cv::Mat Visualizer::plot_all_im_drone_pos(void) {
    std::vector<cv::Mat> ims;
    if (gen_im_posX_drone.rows == im_posX_drone.rows) {
        ims.push_back(plot({im_posX_drone, gen_im_posX_drone}, "Im drone X"));
        ims.push_back(plot({im_posY_drone, gen_im_posY_drone}, "Im drone Y"));
        ims.push_back(plot({im_disp_drone, gen_im_disp_drone}, "Disparity"));
    } else {
        ims.push_back(plot({im_posX_drone}, "Im drone X"));
        ims.push_back(plot({im_posY_drone}, "Im drone Y"));
        ims.push_back(plot({im_disp_drone}, "Disparity"));
    }
    // ims.push_back(plot({im_size_drone,gen_im_size_drone},"Size"));
    return create_column_image(ims, CV_8UC3);
}
cv::Mat Visualizer::plot_xyd(void) {
    std::vector<cv::Mat> ims_xyd;
    ims_xyd.push_back(plot({im_disp_drone}, "Disparity"));


    cv::Point sp1(-_patser->drone.control.viz_drone_pos_after_burn.x * 1000.f, -_patser->drone.control.viz_drone_pos_after_burn.z * 1000.f);
    cv::Point min_xz_range, max_xz_range;
    min_xz_range.x = -2000;
    max_xz_range.x = 2000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 4000; // z
    ims_xyd.push_back(plotxy(posX_drone, posZ_drone, posX_target, posZ_target, sp1, "PosXZ", min_xz_range, max_xz_range));

    cv::Point sp2(-_patser->drone.control.viz_drone_pos_after_burn.x * 1000.f, _patser->drone.control.viz_drone_pos_after_burn.y * 1000.f);
    cv::Point min_xy_range, max_xy_range;
    min_xy_range.x = -2000;
    max_xy_range.x = 2000;
    min_xy_range.y = -2000;
    max_xy_range.y = 2000;
    ims_xyd.push_back(plotxy(posX_drone, posY_drone, posX_target, posY_target, sp2, "PosXY", min_xy_range, max_xy_range));

    return create_column_image(ims_xyd, CV_8UC3);
}

cv::Mat Visualizer::plot_all_control(void) {
    std::vector<cv::Mat> ims_joy;
    ims_joy.push_back(plot({roll_joystick, roll_calculated}, "Roll"));
    ims_joy.push_back(plot({pitch_joystick, pitch_calculated}, "Pitch"));
    ims_joy.push_back(plot({throttle_joystick, throttle_calculated, throttle_min_bound, throttle_max_bound}, "Throttle"));
    return create_column_image(ims_joy, CV_8UC3);
}

cv::Mat Visualizer::plot_all_velocity(void) {
    std::vector<cv::Mat> ims_vel;
    ims_vel.push_back(plot({velX, svelX}, "VelX"));
    ims_vel.push_back(plot({velY, svelY}, "VelY"));
    ims_vel.push_back(plot({velZ, svelZ}, "VelZ"));
    return create_column_image(ims_vel, CV_8UC3);
}

cv::Mat Visualizer::plot_all_acceleration(void) {
    std::vector<cv::Mat> ims_acc;
    ims_acc.push_back(plot({saccX}, "AccX"));
    ims_acc.push_back(plot({saccY}, "AccY"));
    ims_acc.push_back(plot({saccZ}, "AccZ"));
    return create_column_image(ims_acc, CV_8UC3);
}

cv::Mat Visualizer::plot_all_position(void) {
    std::vector<cv::Mat> ims_pos;
    ims_pos.push_back(plot({posX_drone, sposX, setposX}, "PosX"));
    ims_pos.push_back(plot({posY_drone, sposY, setposY}, "PosY"));
    ims_pos.push_back(plot({posZ_drone, sposZ, setposZ}, "PosZ"));
    return create_column_image(ims_pos, CV_8UC3);
}

cv::Mat Visualizer::plot(std::vector<cv::Mat> data, const std::string name) {
    cv::Mat frame(fsizey + 4 * line_width, fsizex + 4 * line_width, CV_8UC3);
    frame.setTo(background_color);
    if (data.at(0).rows > 0) {
        plot(data, &frame, name);
    }
    return frame;
}

void Visualizer::plot(std::vector<cv::Mat> data, cv::Mat *frame, std::string name) {
    putText(*frame, name, cv::Point(0, 13), cv::FONT_HERSHEY_SIMPLEX, text_size, fore_color);
    cv::line(*frame, cv::Point(0, frame->rows - 1), cv::Point(frame->cols, frame->rows - 1), fore_color);
    cv::line(*frame, cv::Point(frame->cols - 1, 0), cv::Point(frame->cols - 1, frame->rows - 1), fore_color);

    int current_buf_size = bufsize;
    if (current_buf_size > data.at(0).rows)
        current_buf_size = data.at(0).rows;
    int start = data.at(0).rows - current_buf_size;

    double mind, maxd;
    cv::Mat tmp;
    for (size_t i = 0 ; i < data.size(); i++) {
        cv::Mat vec = data.at(i);
        cv::Mat vect = cv::Mat(vec, cv::Rect(cv::Point(0, start), cv::Point(1, start + current_buf_size)));
        tmp.push_back(vect);
    }
    cv::minMaxIdx(tmp, &mind, &maxd, NULL, NULL);

    float min, max;
    min = static_cast<float>(mind);
    max = static_cast<float>(maxd);
    std::stringstream ss;
    ss << std::setprecision(4);
    ss << "[" << min << " - " << max << "]";
    putText(*frame, ss.str(), cv::Point(0, 28), cv::FONT_HERSHEY_SIMPLEX, text_size, red);

    float range = max - min;
    float amplify_y = 1;
    amplify_y = fsizey / range;
    min *= amplify_y;
    max *= amplify_y;
    min--;
    max++;
    const float scaleX = static_cast<float>(fsizex) / (bufsize);
    const float scaleY = static_cast<float>(fsizey) / (max - min);

    for (size_t i = 0 ; i < data.size(); i++) {
        int prev_y = 0;
        int prev_x = 0;
        for (int j = start; j < data.at(i).rows - 1; j++)  {
            int y = data.at(i).at<float>(j, 1) * amplify_y - min;
            int x = (j - start) * scaleX + 2 * line_width;
            cv::line(*frame, cv::Point(prev_x, fsizey - prev_y * scaleY + line_width * 2), cv::Point(x, fsizey - y * scaleY + 2 * line_width), linecolors[i], line_width);
            prev_y = y;
            prev_x = x;
        }
    }
}

cv::Mat Visualizer::plotxy(cv::Mat data1x, cv::Mat data1y, cv::Mat data2x, cv::Mat data2y, cv::Point setpoint, std::string name, cv::Point minaxis, cv::Point maxaxis) {
    cv::Mat frame = cv::Mat::zeros((fsizey + 4 * line_width), fsizex + 4 * line_width, CV_8UC3);
    frame.setTo(background_color);
    std::stringstream ss;
    ss.precision(2);
    ss << name << " " << data1x.at<float>(data1x.rows - 1) << "; " << data1y.at<float>(data1y.rows - 1);

    putText(frame, ss.str(), cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, text_size, fore_color);
    cv::line(frame, cv::Point(0, frame.rows - 1), cv::Point(frame.cols, frame.rows - 1), fore_color);
    cv::line(frame, cv::Point(frame.cols - 1, 0), cv::Point(frame.cols - 1, frame.rows - 1), fore_color);

    double minx, maxx;
    double miny, maxy;

    cv::Mat x1S = data1x * 1000;
    cv::Mat y1S = data1y * 1000;

    cv::Mat x2S = data2x * 1000;
    cv::Mat y2S = data2y * 1000;

    minx = minaxis.x;
    maxx = maxaxis.x;
    miny = minaxis.y;
    maxy = maxaxis.y;

    const float scaleX = (fsizex) / (maxx - minx);
    const float scaleY = (fsizey) / (maxy - miny);

    int prev_x = 0;
    int prev_y = 0;
    int start = x1S.rows - bufsize;
    if (start < 0)
        start = 0;

    float x = 0, y = 0;
    for (int j = start; j < x1S.rows - 1; j++)  {
        x = x1S.at<float>(j, 1) - static_cast<float>(minx);
        x = x * scaleX + 2 * line_width;
        y = y1S.at<float>(j, 1) - static_cast<float>(miny);
        y = fsizey - y * scaleY + 2 * line_width;
        if (j > start)
            cv::line(frame, cv::Point(prev_x, prev_y), cv::Point(x, y), green, line_width, cv::LINE_AA, 0);
        prev_x = x;
        prev_y = y;
    }

    for (int j = start; j < x2S.rows - 1; j++)  {
        x = x2S.at<float>(j, 1) - static_cast<float>(minx);
        x = x * scaleX + 2 * line_width;
        y = y2S.at<float>(j, 1) - static_cast<float>(miny);
        y = fsizey - y * scaleY + 2 * line_width;
        if (j > start)
            cv::line(frame, cv::Point(prev_x, prev_y), cv::Point(x, y), pink, line_width, cv::LINE_AA);
        prev_x = x;
        prev_y = y;
    }

    //draw current position more clearly
    cv::circle(frame, cv::Point(x, y), 2, white);

    for (int j = start; j < x2S.rows - 1; j++)  {
        x = x2S.at<float>(j, 1) - static_cast<float>(minx);
        x = x * scaleX + 2 * line_width;
        y = y2S.at<float>(j, 1) - static_cast<float>(miny);
        y = fsizey - y * scaleY + 2 * line_width;
        if (j > start)
            cv::line(frame, cv::Point(prev_x, prev_y), cv::Point(x, y), red, line_width, cv::LINE_AA, 0);
        prev_x = x;
        prev_y = y;
    }

    cv::circle(frame, cv::Point(x, y), 2, white);

    //draw the setpoint
    x = setpoint.x - minx;
    x = x * scaleX + 2 * line_width;
    y = setpoint.y - miny;
    y = fsizey - y * scaleY + 2 * line_width;
    cv::line(frame, cv::Point(0, y), cv::Point(frame.cols, y), blue);
    cv::line(frame, cv::Point(x, 0), cv::Point(x, frame.rows), blue);


    float dist_th_x = (_patser->drone.nav.distance_threshold_mm()) * scaleX + 2 * line_width;
    float dist_th_y = (_patser->drone.nav.distance_threshold_mm()) * scaleY + 2 * line_width;
    cv::rectangle(frame, Point(x - dist_th_x, y - dist_th_y), Point(x + dist_th_x, y + dist_th_y), red);

    return frame;
}

void Visualizer::draw_target_text(cv::Mat resFrame, double time, float dis, float min_dis) {
    std::stringstream ss_time, ss_dis, ss_min;
    closest_dist = (roundf(min_dis * 100) / 100);
    ss_time << "T: " << (round(time * 100) / 100);
    ss_min << "Closest: " << (roundf(min_dis * 100) / 100) << " [m]";
    ss_dis << "|" << (roundf(dis * 100) / 100) << "|";

    putText(resFrame, ss_time.str(), cv::Point(220, 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
    if (min_dis < 9999) {
        putText(resFrame, ss_dis.str(), cv::Point(300, 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        putText(resFrame, ss_min.str(), cv::Point(360, 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
    }

    putText(resFrame, _patser->state_str(), cv::Point(220, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
    putText(resFrame, _patser->trackers.mode_str(), cv::Point(220, 82), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
    if (pparams.op_mode == op_mode_x) {
        putText(resFrame, _patser->drone.drone_state_str(), cv::Point(450, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        putText(resFrame, _patser->baseboard()->charging_state_str(), cv::Point(450, 82), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        putText(resFrame, _patser->interceptor.Interceptor_State(), cv::Point(220, 94), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        if (pparams.joystick != rc_none)
            putText(resFrame, _patser->drone.control.Joy_State_str(), cv::Point(220, 118), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));

        if (_patser->drone.in_flight()) {
            putText(resFrame, _patser->drone.tracker.drone_tracking_state(), cv::Point(450, 106), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
            putText(resFrame, _patser->drone.control.flight_mode_str(), cv::Point(450, 94), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        }
        putText(resFrame, _patser->rc()->armed_str(), cv::Point(220, 106), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
    }

    std::string warnings = "";
    static int warning_color_cnt = 0;
    warning_color_cnt++;
    warning_color_cnt = warning_color_cnt % 255;

    if (not _visdat->no_recent_brightness_events(time))
        warnings += "BRIGHTNESS ";
    if (_patser->trackers.monster_alert())
        warnings += "MONSTER ALERT ";
    if (_fromfile)
        warnings += "POPCORN TIME ";

    if (warnings != "") {
        int baseline = 0;
        auto s = getTextSize(warnings, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        putText(resFrame, warnings, cv::Point(400 - s.width / 2, 28), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255 - warning_color_cnt, warning_color_cnt, 255));
    }
}

cv::Mat Visualizer::draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<tracking::TrackData> path) {

    cv::Mat frameL_small_drone;
    if (path.size() > 0) {
        std::vector<cv::KeyPoint> keypoints;
        for (size_t i = 0; i < path.size(); i++) {
            if (path.at(i).predicted_image_item.valid) {
                cv::KeyPoint k(path.at(i).predicted_image_item.pt.x / im_scaler, path.at(i).predicted_image_item.pt.y / im_scaler, 24 / im_scaler);
                keypoints.push_back(k);
            }
        }
        drawKeypoints(frameL_small, keypoints, frameL_small_drone, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    } else {
        cvtColor(frameL_small, frameL_small_drone, cv::COLOR_GRAY2BGR);
    }

    if (path.size() > 0) {
        std::vector<cv::KeyPoint> keypoints;
        for (size_t i = 0; i < path.size(); i++) {
            if (path.at(i).world_item.image_item.valid) {
                cv::KeyPoint k(path.at(i).world_item.image_item.x / im_scaler, path.at(i).world_item.image_item.y / im_scaler, 12 / im_scaler);
                keypoints.push_back(k);
            }
        }
        drawKeypoints(frameL_small_drone, keypoints, frameL_small_drone, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
    cv::circle(frameL_small_drone, cv::Point(setpoint.x, setpoint.y), 2, cv::Scalar(150, 255, 200));
    cv::resize(frameL_small_drone, frameL_small_drone, vizsizeL);

    return frameL_small_drone;
}

void Visualizer::update_tracker_data(cv::Mat frameL, cv::Point3f setpoint, double time, bool draw_plots) {
    enable_plots = draw_plots;
    if (new_tracker_viz_data_requested) {
        lock_frame_data.lock();

        static float min_dis = 9999;
        float dis = 0;
        auto dtrkr = &_patser->drone.tracker;
        auto itrkr = _patser->interceptor.target_insecttracker();
        if (dtrkr->tracking() && itrkr) {

            auto pd = dtrkr->world_item().pt;
            auto pi = itrkr->world_item().pt;
            dis = normf(pd - pi);

            if (dis < min_dis)
                min_dis = dis;
        }

        if (dtrkr->pad_location_valid()) {
            ground_y = dtrkr->pad_location().y;
        }

        tracker_viz_base_data.frameL = frameL;
        tracker_viz_base_data.dis = dis;
        tracker_viz_base_data.min_dis = min_dis;
        tracker_viz_base_data.setpoint = setpoint;
        tracker_viz_base_data.time = time;

        new_tracker_viz_data_requested = false;
        lock_frame_data.unlock();
        newdata.notify_all();
    }

}

void Visualizer::draw_tracker_viz() {
    cv::Mat frameL = tracker_viz_base_data.frameL;
    float dis = tracker_viz_base_data.dis;
    float min_dis = tracker_viz_base_data.min_dis;
    cv::Point3f setpoint = tracker_viz_base_data.setpoint;
    double time = tracker_viz_base_data.time;

    std::vector<tracking::TrackData> drn_path = _patser->drone.tracker.track();
    tracking::TrackData last_drone_detection;
    if (drn_path.size())
        last_drone_detection = drn_path.back();

    std::vector<tracking::TrackData> ins_path;
    if (_patser->interceptor.target_insecttracker())
        ins_path = _patser->interceptor.target_insecttracker()->track();
    tracking::TrackData last_insect_detection;
    if (ins_path.size())
        last_insect_detection = ins_path.back();

    cv::Size resFrame_size = viz_frame_size();
    resFrame_size.width -= IMG_W;
    cv::Mat resFrame = cv::Mat::zeros(resFrame_size, CV_8UC3);
    cv::Mat frameL_color;
    cvtColor(frameL, frameL_color, cv::COLOR_GRAY2BGR);
    cv::Rect rect(0, frameL.rows / 4, frameL.cols, frameL.rows);
    cv::Mat roi = resFrame(rect);
    cv::Size size(frameL.cols, frameL.rows);

    if (_patser->drone.tracker.taking_off()) {
        cv::Point2f template_matching_im_location = _patser->drone.tracker.image_template_item().pt();
        float template_size = _patser->drone.tracker.image_template_item().size;
        cv::Rect template_match(static_cast<int>(template_matching_im_location.x - template_size / 2.f), static_cast<int>(template_matching_im_location.y - template_size / 2.f), static_cast<int>(template_size), static_cast<int>(template_size));
        template_match = clamp_rect(template_match, IMG_W, IMG_H);
        cv::rectangle(frameL_color, template_match, cv::Scalar(255, 0, 255), 1);
    }

    if (_patser->drone.tracker.taking_off()) {
        float pad_im_size = _patser->drone.tracker.pad_im_size();
        cv::Point2f takeoff_im_direction = _patser->drone.tracker.takeoff_direction_predicted();
        cv::Point2f pad_im_location = _patser->drone.tracker.pad_im_location();
        cv::line(frameL_color, pad_im_location, pad_im_location + (pad_im_size / 2.f) *takeoff_im_direction, cv::Scalar(0, 255, 255));
    }

    if (last_drone_detection.predicted_image_item.valid) {
        auto pred =  last_drone_detection.predicted_image_item;
        cv::circle(frameL_color, pred.pt, pred.size / 2, cv::Scalar(0, 255, 0));
    } else if (_patser->drone.tracker.pad_location_valid() && !drn_path.size()) {
        cv::circle(frameL_color, _patser->drone.tracker.pad_im_location(), _patser->drone.tracker.pad_im_size() / 2, cv::Scalar(0, 255, 0));
    }
    if (last_drone_detection.world_item.image_item.valid) {
        auto p =  last_drone_detection.world_item.image_item;
        cv::circle(frameL_color, p.pt(), p.size / 2, cv::Scalar(0, 0, 255));
    }

    putText(frameL_color, "Drone prediction replay", cv::Point(3, frameL_color.rows - 12), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0));
    putText(frameL_color, "Drone detection live", cv::Point(3, frameL_color.rows - 24), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0));
    putText(frameL_color, "Drone detection replay", cv::Point(3, frameL_color.rows - 36), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255));
    putText(frameL_color, "Drone detection template match", cv::Point(3, frameL_color.rows - 48), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255));

    cv::Scalar itrkr_color = cv::Scalar(0, 0, 128);
    if (last_insect_detection.predicted_image_item.valid) {
        std::stringstream ss;
        tracking::WorldItem wti = last_insect_detection.world_item;

        cv::Point2i p_ins_im = last_insect_detection.predicted_image_item.pt;
        float ins_size = last_insect_detection.predicted_image_item.size;
        ss << "i ";
        if (wti.valid) {
            ss << to_string_with_precision(wti.distance, 1);
            itrkr_color = cv::Scalar(0, 0, 255);
            p_ins_im  = wti.image_item.pt();
            ins_size = wti.image_item.size;
        }
        if (ins_size <= 0) //replay insects don't have a proper size
            ins_size = 2;

        putText(frameL_color, ss.str(), p_ins_im, cv::FONT_HERSHEY_SIMPLEX, 0.5, itrkr_color);
        cv::line(frameL_color, p_ins_im, p_ins_im, itrkr_color, 2);
        cv::Point3f p_ins_ground_wrld = wti.pt;
        p_ins_ground_wrld.y = ground_y;
        cv::Point2f p_ins_ground_im = world2im_2d(p_ins_ground_wrld, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        cv::line(frameL_color, p_ins_im, p_ins_ground_im, itrkr_color, 1);


        cv::circle(frameL_color, p_ins_im, ins_size / 2.f, itrkr_color);

    }

    if (_patser->drone.control.ff_interception()) {
        //burn:
        cv::Point2f viz_drone_pos_after_burn_im = world2im_2d(_patser->drone.control.viz_drone_pos_after_burn, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        cv::Point2f viz_target_pos_after_burn_im = world2im_2d(_patser->drone.control.viz_target_pos_after_burn, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());


        if (!_patser->drone.control.viz_trajectory.empty() && ! drn_path.empty()) {
            double raak = norm(_patser->drone.control.viz_trajectory.back() .pos - last_drone_detection.world_item.pt);
            putText(resFrame, "trgt: |"  + to_string_with_precision(raak, 2) + "|", cv::Point(460, 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 255));
        }

        cv::circle(frameL_color, viz_target_pos_after_burn_im, 2, white);
        viz_drone_pos_after_burn_im.x += 15;
        putText(frameL_color, to_string_with_precision(_patser->drone.control.viz_time_after_burn, 2), viz_drone_pos_after_burn_im, cv::FONT_HERSHEY_SIMPLEX, 0.5, pink);

        for (size_t i = 0; i < _patser->drone.control.viz_trajectory.size(); i++) {
            cv::Point2f p = world2im_2d(_patser->drone.control.viz_trajectory.at(i).pos, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
            cv::circle(frameL_color, p, 1, pink);
        }

        for (size_t i = 0; i < drn_path.size(); i++) {
            cv::Point2f p = world2im_2d(drn_path.at(i).world_item.pt, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
            cv::circle(frameL_color, p, 1, blue);
        }
    }

    if (last_drone_detection.world_item.image_item.valid) {
        std::stringstream ss;
        tracking::WorldItem wti = last_drone_detection.world_item;
        ss << "d " << to_string_with_precision(wti.distance, 1);
        cv::Scalar c(0, 0, 255);
        if (wti.distance_bkg > wti.distance)
            c = cv::Scalar(0, 180, 255); //first number cannot be 0 because of super weird qtcreator / gdb bug
        cv::Point2i drone_pos(wti.image_item.x, wti.image_item.y);
        putText(frameL_color, ss.str(), drone_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, c);
        cv::line(frameL_color, drone_pos, drone_pos, c, 2);

        if (_patser->drone.in_flight() && !_patser->drone.control.ff_interception()) { //draw line from drone to target setpoint
            cv::Point2i target = _patser->drone.tracker.image_item().pt();
            cv::Scalar c2;
            if (_patser->drone.nav.drone_hunting() && target.x + target.y > 0) {
                c2 = red;
                cv::Point2i text_pos = drone_pos - (drone_pos - target) / 2;
                putText(frameL_color, to_string_with_precision(_patser->interceptor.time_to_intercept(), 2) + "s", text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, c2);
            } else
                c2 = white;
            cv::line(frameL_color, drone_pos, target, c2, 1);
        }
    }

    if (pos_log_drone_valid) {
        cv::circle(frameL_color, pos_log_drone, 10, blue, 2);
        pos_log_drone_valid = false;
    }

    cv::resize(frameL_color, roi, size);

    cv::Size vizsizeL(size.width / 4, size.height / 4);
    cv::Mat frameL_small;
    cv::resize(frameL, frameL_small, cv::Size(frameL.cols / im_scaler, frameL.rows / im_scaler));
    cv::Mat frameL_small_drone = draw_sub_tracking_viz(frameL_small, vizsizeL, setpoint, drn_path);
    cv::Mat frameL_small_insect = draw_sub_tracking_viz(frameL_small, vizsizeL, setpoint, ins_path);
    frameL_small_drone.copyTo(resFrame(cv::Rect(0, 0, frameL_small_drone.cols, frameL_small_drone.rows)));
    frameL_small_insect.copyTo(resFrame(cv::Rect(resFrame.cols - frameL_small_drone.cols, 0, frameL_small_drone.cols, frameL_small_drone.rows)));
    draw_target_text(resFrame, time, dis, min_dis);

    if (_patser->trackers.diff_viz_buf.cols > 0) {

        cv::Mat ext_res_frame = cv::Mat::zeros(resFrame.rows, resFrame.cols + _patser->trackers.diff_viz_buf.cols, CV_8UC3);
        resFrame.copyTo(ext_res_frame(cv::Rect(0, 0, resFrame.cols, resFrame.rows)));
        _patser->trackers.diff_viz_buf.copyTo(ext_res_frame(cv::Rect(resFrame.cols - 1, frameL_small_drone.rows - 1, _patser->trackers.diff_viz_buf.cols, _patser->trackers.diff_viz_buf.rows)));
        cv::Mat diff = ext_res_frame(cv::Rect(resFrame.cols, frameL_small_drone.rows, _patser->trackers.diff_viz_buf.cols, _patser->trackers.diff_viz_buf.rows));

        putText(diff, "Drone", cv::Point(3, diff.rows - 12), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0));
        putText(diff, "Insect blb->trk", cv::Point(3, diff.rows - 24), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255));
        putText(diff, "Replay/Virtual", cv::Point(3, diff.rows - 36), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 180));
        putText(diff, "Blink", cv::Point(3, diff.rows - 48), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255));
        putText(diff, "Ignored", cv::Point(3, diff.rows - 60), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 128, 0));
        putText(diff, "Untracked", cv::Point(3, diff.rows - 72), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 55));
        putText(diff, "Multitracked", cv::Point(3, diff.rows - 84), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 128, 255));
        putText(diff, "False positive", cv::Point(3, diff.rows - 96), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0));
        putText(diff, "Overexposed noise", cv::Point(3, diff.rows - 108), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 128, 128));

        trackframe = ext_res_frame;
    } else
        trackframe = resFrame;

}

void Visualizer::draw_drone_from_log(logging::LogEntryDrone entry) {
    if (entry.foundL) {
        pos_log_drone = cv::Point(entry.im_x, entry.im_y);
        pos_log_drone_valid = true;
    }
}

void Visualizer::render() {
    new_tracker_viz_data_requested = true;
}
void Visualizer::paint() {
    if (request_trackframe_paint) {
        request_trackframe_paint = false;

        if (!_tracking_viz_initialized) {
            _tracking_viz_initialized = true;
            namedWindow("tracking results", cv::WINDOW_OPENGL | cv::WINDOW_AUTOSIZE);
        }
        cv::imshow("tracking results", trackframe);


        if (_viz_noise_initialized && _visdat->motion_max_noise_mapL.cols) {
            cv::imshow("filtered noise map", _visdat->motion_filtered_noise_mapL * 10);
            cv::imshow("noise map", _visdat->motion_max_noise_mapL * 10);
        }
        if (_viz_exposure_initialized && _visdat->overexposed_mapL.cols)
            cv::imshow("exposure map", _visdat->overexposed_mapL);

        auto drone_diff_viz = _patser->drone.tracker.diff_viz;
        if (drone_diff_viz.cols) {
            static bool drn_diff_viz_initialized = false;
            if (!drn_diff_viz_initialized) {
                drn_diff_viz_initialized = true;
                namedWindow("drn_diff", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
            }
            cv::imshow("drn_diff", drone_diff_viz);
        }
        if (_patser->trackers.viz_trkrs_buf.cols) {
            static bool trkrs_blobs_viz_initialized = false;
            if (!trkrs_blobs_viz_initialized) {
                trkrs_blobs_viz_initialized = true;
                namedWindow("trackers & blobs", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
            }
            cv::imshow("trackers & blobs", _patser->trackers.viz_trkrs_buf);
        }
        if (_patser->trackers.viz_max_points.cols) {
            static bool motion_points_viz_initialized = false;
            if (!motion_points_viz_initialized) {
                motion_points_viz_initialized = true;
                namedWindow("motion blobs", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
            }
            cv::imshow("motion blobs", _patser->trackers.viz_max_points);
        }
        for (auto drone : _patser->trackers.dronetrackers()) {
            if (drone->viz_disp.cols > 0) {
                static bool stereo_viz_initialized = false;
                if (!stereo_viz_initialized) {
                    stereo_viz_initialized = true;
                    namedWindow("stereo drone " + std::to_string(drone->uid()), cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
                }
                imshow("stereo drone " + std::to_string(drone->uid()), drone->viz_disp);
            }
        }
        if (_patser->interceptor.target_insecttracker()) {
            if (_patser->interceptor.target_insecttracker()->viz_disp.cols > 0) {
                static bool stereo_viz_initialized = false;
                if (!stereo_viz_initialized) {
                    stereo_viz_initialized = true;
                    namedWindow("stereo insect", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
                }
                imshow("stereo insect", _patser->interceptor.target_insecttracker()->viz_disp);
            }
        }

        new_tracker_viz_data_requested = true;
    }
    if (request_plotframe_paint && plotframe.rows > 0) {
        request_plotframe_paint = false;
        static bool plot_viz_initialized = false;
        if (!plot_viz_initialized) {
            plot_viz_initialized = true;
            namedWindow("Plots", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        }
        imshow("Plots", plotframe);
    }
}

void Visualizer::workerThread(void) {
    std::cout << "Viz thread started!" << std::endl;
    std::unique_lock<std::mutex> lk(m, std::defer_lock);
    while (!exitVizThread) {
        newdata.wait(lk);
        if (roll_joystick.rows > 0) {
            if (enable_plots) {
                lock_plot_data.lock();
                plot();
                request_plotframe_paint = true;
                lock_plot_data.unlock();
            }
        }
        lock_frame_data.lock();
        if (tracker_viz_base_data.frameL.rows > 0) {
            draw_tracker_viz();
            request_trackframe_paint = true;
        }
        lock_frame_data.unlock();


    }
}
void Visualizer::close() {
    if (initialized) {
        std::cout << "Closing visualizer" << std::endl;
        exitVizThread = true;
        newdata.notify_all();
        lock_plot_data.unlock();
        thread_viz.join();
        initialized = false;
        std::cout << "Closest: " << closest_dist << std::endl;
    }
}
