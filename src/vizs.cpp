#include "vizs.h"
using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define DRAWPLOTS
#define DRAWTRACKING
#endif

cv::Scalar white(255,255,255);
cv::Scalar black(0,0,0);
cv::Scalar green(0,255,0);
cv::Scalar blue(255,0,0);
cv::Scalar red(0,0,255);
cv::Scalar linecolors[] = {green,blue,red,cv::Scalar(0,255,255),cv::Scalar(255,255,0),cv::Scalar(255,0,255)};

//cv::Scalar background_color(255,255,255);
cv::Scalar fore_color(0,0,0);
cv::Scalar background_color(255,255,255);

void Visualizer::addPlotSample(void) {
#ifdef DRAWPLOTS
    lock_plot_data.lock();
    roll_joystick.push_back(static_cast<float>(_dctrl->joyRoll));
    pitch_joystick.push_back(static_cast<float>(_dctrl->joyPitch));
    yaw_joystick.push_back(static_cast<float>(_dctrl->joyPitch));
    throttle_joystick.push_back(static_cast<float>(_dctrl->joyThrottle));

    roll_calculated.push_back(static_cast<float>(_dctrl->autoRoll));
    pitch_calculated.push_back(static_cast<float>(_dctrl->autoPitch));
    //    yaw_calculated.push_back(static_cast<float>(_dctrl->commandedYaw));
    throttle_calculated.push_back(static_cast<float>(_dctrl->autoThrottle));
    throttle_hover.push_back(_dctrl->hoverthrottle);

    trackData data = _dtrkr->get_last_track_data();
    dt.push_back(data.dt);
    dt_target.push_back(1.f/VIDEOFPS);

    posX.push_back(-data.posX);
    posY.push_back(data.posY);
    posZ.push_back(-data.posZ);
    disparity.push_back(_dtrkr->find_result.disparity);
    sdisparity.push_back(_dtrkr->find_result.smoothed_disparity);

    sposX.push_back(-data.sposX);
    sposY.push_back(data.sposY);
    sposZ.push_back(-data.sposZ);

    setposX.push_back(-_dnav->setpoint_world.x);
    setposY.push_back(_dnav->setpoint_world.y);
    setposZ.push_back(-_dnav->setpoint_world.z);

    velX.push_back(-data.velX);
    velY.push_back(data.velY);
    velZ.push_back(-data.velZ);

    svelX.push_back(-data.svelX);
    svelY.push_back(data.svelY);
    svelZ.push_back(-data.svelZ);

    accX.push_back(-data.accX);
    accY.push_back(data.accY);
    accZ.push_back(-data.accZ);

    saccX.push_back(-data.saccX);
    saccY.push_back(data.saccY);
    saccZ.push_back(-data.saccZ);

    autotakeoff_velY_thresh.push_back(static_cast<float>(_dnav->params.auto_takeoff_speed) / 100.f);

    lock_plot_data.unlock();
    newdata.notify_all();
#endif
}

void Visualizer::plot(void) {
    std::vector<cv::Mat> ims_trk;
    ims_trk.push_back(plot_xyd());
    ims_trk.push_back(plot_all_position());
    ims_trk.push_back(plot_all_velocity());
    //    ims_trk.push_back(plot_all_acceleration());
    ims_trk.push_back(plot_all_control());
    //showRowImage(ims_trk, "Tracking",CV_8UC3);
    plotframe = createRowImage(ims_trk,CV_8UC3);
}

cv::Mat Visualizer::plot_xyd(void) {
    std::vector<cv::Mat> ims_xyd;
    ims_xyd.push_back(plot({disparity,sdisparity},"Disparity"));


    cv::Point sp1(-_dnav->setpoint_world.x*1000.f,-_dnav->setpoint_world.z*1000.f);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    ims_xyd.push_back(plotxy(posX,posZ, sp1,"PosXZ",min_xz_range,max_xz_range));

    cv::Point sp2(-_dnav->setpoint_world.x*1000.f,_dnav->setpoint_world.y*1000.f);
    cv::Point min_xy_range,max_xy_range;
    min_xy_range.x =-3000;
    max_xy_range.x = 3000;
    min_xy_range.y =-3000;
    max_xy_range.y = 3000;
    ims_xyd.push_back(plotxy(posX,posY, sp2, "PosXY",min_xy_range,max_xy_range));

    return createColumnImage(ims_xyd, CV_8UC3);
}

cv::Mat Visualizer::plot_all_control(void) {
    std::vector<cv::Mat> ims_joy;
    ims_joy.push_back(plot({roll_joystick,roll_calculated},"Roll"));
    ims_joy.push_back(plot({pitch_joystick,pitch_calculated},"Pitch"));
    ims_joy.push_back(plot({throttle_joystick,throttle_calculated,throttle_hover},"Throttle"));
    return createColumnImage(ims_joy, CV_8UC3);
}

cv::Mat Visualizer::plot_all_velocity(void) {
    std::vector<cv::Mat> ims_vel;
    ims_vel.push_back(plot({velX,svelX}, "VelX"));
    ims_vel.push_back(plot({velY,svelY,autotakeoff_velY_thresh},"VelY"));
    ims_vel.push_back(plot({velZ,svelZ},"VelZ"));
    return createColumnImage(ims_vel,CV_8UC3);
}

cv::Mat Visualizer::plot_all_acceleration(void) {
    std::vector<cv::Mat> ims_acc;
    ims_acc.push_back(plot({accX,saccX}, "AccX"));
    ims_acc.push_back(plot({accY,saccY,autotakeoff_velY_thresh},"AccY"));
    ims_acc.push_back(plot({accZ,saccZ},"AccZ"));
    return createColumnImage(ims_acc,CV_8UC3);
}

cv::Mat Visualizer::plot_all_position(void) {
    std::vector<cv::Mat> ims_pos;
    ims_pos.push_back(plot({posX,sposX,setposX},"PosX"));
    ims_pos.push_back(plot({posY,sposY,setposY},"PosY"));
    ims_pos.push_back(plot({posZ,sposZ,setposZ},"PosZ"));
    return createColumnImage(ims_pos, CV_8UC3);
}

cv::Mat Visualizer::plot(std::vector<cv::Mat> data, const std::string name) {
    cv::Mat frame(fsizey+4*line_width, fsizex+4*line_width, CV_8UC3);
    frame.setTo(background_color);
    plot(data, &frame,name);
    return frame;
}

void Visualizer::plot(std::vector<cv::Mat> data, cv::Mat *frame, std::string name) {
    putText(*frame,name,cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,text_size,fore_color);
    cv::line(*frame,cv::Point(0,frame->rows-1),cv::Point(frame->cols,frame->rows-1),fore_color);
    cv::line(*frame,cv::Point(frame->cols-1,0),cv::Point(frame->cols-1,frame->rows-1),fore_color);

    int current_buf_size = bufsize;
    if (current_buf_size > data.at(0).rows)
        current_buf_size = data.at(0).rows;
    int start = data.at(0).rows -current_buf_size;

    double mind,maxd;
    cv::Mat tmp;
    for (uint i = 0 ; i< data.size();i++) {
        cv::Mat vec = data.at(i);
        cv::Mat vect = cv::Mat(vec,cv::Rect(cv::Point(0,start),cv::Point(1,start+current_buf_size)));
        tmp.push_back(vect);
    }
    cv::minMaxIdx(tmp,&mind,&maxd,NULL,NULL);

    float min,max;
    min = static_cast<float>(mind);
    max = static_cast<float>(maxd);
    std::stringstream ss;
    ss << std::setprecision(4);
    ss << "[" << min << " - " << max << "]";
    putText(*frame,ss.str(),cv::Point(0, 28),cv::FONT_HERSHEY_SIMPLEX,text_size,red);

    float range = max - min;
    float amplify_y = 1;
    amplify_y = fsizey / range;
    min *=amplify_y;
    max *=amplify_y;
    min-=1;
    max+=1;
    const float scaleX = static_cast<float>(fsizex)/(bufsize);
    const float scaleY = static_cast<float>(fsizey)/(max-min);

    for (uint i = 0 ; i< data.size();i++) {
        int prev_y =0;
        int prev_x=0;
        for (int j = start; j < data.at(i).rows-1; j++)  {
            int y = data.at(i).at<float>(j,1)*amplify_y - min;
            int x = (j-start)*scaleX + 2*line_width;
            cv::line(*frame, cv::Point(prev_x, fsizey- prev_y*scaleY +line_width*2) , cv::Point(x, fsizey - y*scaleY +2*line_width), linecolors[i], line_width, CV_NORMAL, 0);
            prev_y = y;
            prev_x = x;
        }
    }
}

cv::Mat Visualizer::plotxy(cv::Mat datax,cv::Mat datay, cv::Point setpoint, std::string name,cv::Point minaxis,cv::Point maxaxis) {
    cv::Mat frame = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame.setTo(background_color);
    std::stringstream ss;
    ss.precision(2);
    ss << name << " " << datax.at<float>(datax.rows-1) << "; " << datay.at<float>(datay.rows-1);

    putText(frame,ss.str() ,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,text_size,fore_color);
    cv::line(frame,cv::Point(0,frame.rows-1),cv::Point(frame.cols,frame.rows-1),fore_color);
    cv::line(frame,cv::Point(frame.cols-1,0),cv::Point(frame.cols-1,frame.rows-1),fore_color);

    double minx,maxx;
    double miny,maxy;

    cv::Mat xS = datax*1000;
    cv::Mat yS = datay*1000;

    cv::Mat tmpx;
    tmpx.push_back(xS);
    cv::minMaxIdx(tmpx,&minx,&maxx,NULL,NULL);

    cv::Mat tmpy;
    tmpy.push_back(yS);
    cv::minMaxIdx(tmpy,&miny,&maxy,NULL,NULL);

    minx=minaxis.x;
    maxx=maxaxis.x;
    miny=minaxis.y;
    maxy=maxaxis.y;

    const float scaleX = (fsizex)/(maxx-minx);
    const float scaleY = (fsizey)/(maxy-miny);

    int prev_x =0;
    int prev_y =0;
    int start = xS.rows -bufsize;
    if (start < 0)
        start = 0;

    float x,y;
    for (int j = start; j < xS.rows-1; j++)  {
        x = xS.at<float>(j,1) - static_cast<float>(minx);
        x =x*scaleX + 2*line_width;
        y = yS.at<float>(j,1) - static_cast<float>(miny);
        y= fsizey - y*scaleY + 2*line_width;
        if (j > start)
            cv::line(frame, cv::Point(prev_x, prev_y) , cv::Point(x, y), green, line_width, CV_AA, 0);
        prev_x = x;
        prev_y = y;
    }

    //draw current position more clearly
    cv::circle(frame,cv::Point(x,y),2,red);

    //draw the setpoint
    x = setpoint.x - minx;
    x = x* scaleX + 2*line_width;
    y = setpoint.y - miny;
    y= fsizey - y*scaleY + 2*line_width;
    cv::line(frame,cv::Point(0,y),cv::Point(frame.cols,y),cv::Scalar(80,80,150));
    cv::line(frame,cv::Point(x,0),cv::Point(x,frame.rows),cv::Scalar(80,80,150));


    float dist_th_x = (_dnav->distance_threshold_mm)* scaleX + 2*line_width;
    float dist_th_y = (_dnav->distance_threshold_mm)* scaleY + 2*line_width;
    cv::rectangle(frame,Point(x-dist_th_x,y-dist_th_y),Point(x+dist_th_x,y+dist_th_y),cv::Scalar(255,0,0));

    return frame;
}

void Visualizer::draw_segment_viz(void){
    std::vector<cv::Mat> ims;
    ims.push_back(cir8);
    ims.push_back(bkg8);
    ims.push_back(dif8);
    ims.push_back(_dtrkr->_approx);
    ims.push_back(_dtrkr->_treshL);
    showColumnImage(ims,"drone_roi",CV_8UC1);
}

void Visualizer::draw_target_text(cv::Mat resFrame, float time, float dis,float min_dis) {
    std::stringstream ss_time,ss_dis,ss_min;

    ss_time << "T: " << (roundf(time*100)/100);
    ss_min << "Closest: " << (roundf(min_dis*100)/100) << " [m]";
    ss_dis << "|" << (roundf(dis*100)/100) << "|";

    putText(resFrame,ss_time.str() ,cv::Point(220*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    if (min_dis<9999){
        putText(resFrame,ss_dis.str() ,cv::Point(300*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
        putText(resFrame,ss_min.str() ,cv::Point(360*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    }

}

cv::Mat Visualizer::draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<ItemTracker::track_item> path,std::vector<ItemTracker::track_item> predicted_path,std::vector<ItemTracker::track_item> exclude_path,cv::Rect roi_offset,int exclude_max_distance, int exclude_min_distance) {

    cv::Mat frameL_small_drone;
    std::vector<ItemTracker::track_item> tmp = predicted_path;
    if (tmp.size()>0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< tmp.size();i++) {
            tmp.at(i).k.size = 24/IMSCALEF;
            keypoints.push_back(tmp.at(i).k);
        }
        drawKeypoints( frameL_small, keypoints, frameL_small_drone, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    } else {
        cvtColor(frameL_small,frameL_small_drone,CV_GRAY2BGR);
    }

    cv::rectangle(frameL_small_drone,roi_offset,cv::Scalar(180,100,240),4/IMSCALEF);

    if (exclude_path.size() > 0) {
        ItemTracker::track_item exclude = exclude_path.at(exclude_path.size()-1);
        float threshold_dis = exclude_min_distance / sqrtf(exclude.tracking_certainty)+exclude.k.size;
        if (threshold_dis > exclude_max_distance)
            threshold_dis = exclude_max_distance;
        cv::circle(frameL_small_drone,exclude.k.pt,threshold_dis,cv::Scalar(255,0,0),4/IMSCALEF);
    }


    if (path.size() > 0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< path.size();i++) {
            path.at(i).k.size = 12/IMSCALEF;
            keypoints.push_back(path.at(i).k);
        }
        drawKeypoints( frameL_small_drone, keypoints, frameL_small_drone, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(frameL_small_drone,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(frameL_small_drone,frameL_small_drone,vizsizeL);

    return frameL_small_drone;
}


void Visualizer::update_tracker_data(cv::Mat frameL, cv::Point3d setpoint, float time, DroneTracker * dtrk, InsectTracker * itrk) {
    if (new_tracker_viz_data_requested) {
        lock_frame_data.lock();

        static float min_dis = 9999;
        float dis = 0;
        if (dtrk->n_frames_tracking>0 && itrk->n_frames_tracking>0) {
            dis = powf(dtrk->get_last_track_data().posX-itrk->get_last_track_data().posX,2) +
                    powf(dtrk->get_last_track_data().posY-itrk->get_last_track_data().posY,2) +
                    powf(dtrk->get_last_track_data().posZ-itrk->get_last_track_data().posZ,2);
            dis = sqrtf(dis);

            if (dis < min_dis)
                min_dis = dis;
        }


        tracker_viz_base_data.frameL = frameL;
        tracker_viz_base_data.dis = dis;
        tracker_viz_base_data.min_dis = min_dis;
        tracker_viz_base_data.setpoint = setpoint;
        tracker_viz_base_data.time = time;
        tracker_viz_base_data.drn_path = dtrk->pathL;
        tracker_viz_base_data.drn_predicted_path = dtrk->predicted_pathL;
        tracker_viz_base_data.ins_path = itrk->pathL;
        tracker_viz_base_data.ins_predicted_path = itrk->predicted_pathL;
        tracker_viz_base_data.drn_roi_offset = dtrk->find_result.roi_offset;
        tracker_viz_base_data.ins_roi_offset = itrk->find_result.roi_offset;
        tracker_viz_base_data.exclude_min_distance = dtrk->settings.exclude_min_distance;
        tracker_viz_base_data.exclude_max_distance = dtrk->settings.exclude_max_distance;

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
    float time = tracker_viz_base_data.time;

    std::vector<ItemTracker::track_item> drn_path = tracker_viz_base_data.drn_path;
    std::vector<ItemTracker::track_item> drn_predicted_path = tracker_viz_base_data.drn_predicted_path;
    std::vector<ItemTracker::track_item> ins_path = tracker_viz_base_data.ins_path;
    std::vector<ItemTracker::track_item> ins_predicted_path = tracker_viz_base_data.ins_predicted_path;
    cv::Rect drn_roi_offset = tracker_viz_base_data.drn_roi_offset;
    cv::Rect ins_roi_offset = tracker_viz_base_data.ins_roi_offset;
    int exclude_min_distance = tracker_viz_base_data.exclude_min_distance;
    int exclude_max_distance = tracker_viz_base_data.exclude_max_distance;

    cv::Mat resFrame = cv::Mat::zeros(frameL.rows*_res_mult+frameL.rows*_res_mult/4,frameL.cols*_res_mult,CV_8UC3);
    cv::Mat frameL_color;
    cvtColor(frameL,frameL_color,CV_GRAY2BGR);
    cv::Rect rect(0,frameL.rows*_res_mult/4,frameL.cols*_res_mult,frameL.rows*_res_mult);
    cv::Mat roi = resFrame(rect);
    cv::Size size (frameL.cols*_res_mult,frameL.rows*_res_mult);
    cv::resize(frameL_color,roi,size);

    //        cir8 = _dtrkr->_cir*255;
    //        bkg8 = _dtrkr->_bkg*255;
    //        dif8 = _dtrkr->_dif*10;
    //        cir8.convertTo(cir8, CV_8UC1);
    //        bkg8.convertTo(bkg8, CV_8UC1);
    //        dif8.convertTo(dif8, CV_8UC1);
    //        draw_segment_viz(); //warning NOT THREAD SAFE, do not uncomment here anymore!

    cv::Size vizsizeL(size.width/4,size.height/4);
    cv::Mat frameL_small;
    cv::resize(frameL,frameL_small,cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF));
    cv::Mat frameL_small_drone = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,drn_path,drn_predicted_path,ins_predicted_path,drn_roi_offset,exclude_max_distance, exclude_min_distance);
    cv::Mat frameL_small_insect = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,ins_path,ins_predicted_path,drn_predicted_path,ins_roi_offset,exclude_max_distance, exclude_min_distance);
    frameL_small_drone.copyTo(resFrame(cv::Rect(0,0,frameL_small_drone.cols, frameL_small_drone.rows)));
    frameL_small_insect.copyTo(resFrame(cv::Rect(resFrame.cols-frameL_small_drone.cols,0,frameL_small_drone.cols, frameL_small_drone.rows)));
    draw_target_text(resFrame,time,dis,min_dis);

    trackframe = resFrame;

}

void Visualizer::paint() {
    if (request_trackframe_paint) {
        request_trackframe_paint = false;
        cv::imshow("tracking results", trackframe);
        new_tracker_viz_data_requested = true;
    }
    if (request_plotframe_paint) {
        request_plotframe_paint = false;
        imshow("Plots",plotframe);
    }
}

void Visualizer::workerThread(void) {
    std::cout << "Viz thread started!" << std::endl;
    std::unique_lock<std::mutex> lk(m,std::defer_lock);
    while (!exitVizThread) {
        newdata.wait(lk);
        if (roll_joystick.rows > 0) {
#ifdef DRAWPLOTS
            lock_plot_data.lock();
            plot();
            request_plotframe_paint = true;
            lock_plot_data.unlock();
#endif
        }
#ifdef DRAWTRACKING
        lock_frame_data.lock();
        draw_tracker_viz();
        request_trackframe_paint=true;
        lock_frame_data.unlock();
#endif

    }
}
void Visualizer::close() {
    if (initialised){
        exitVizThread = true;
        newdata.notify_all();
        lock_plot_data.unlock();
        thread_viz.join();
    }
}
