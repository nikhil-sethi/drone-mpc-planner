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

void Visualizer::init(VisionData *visdat, ItemManager *imngr, DroneController *dctrl, DroneNavigation *dnav, MultiModule *rc, bool fromfile, DronePredictor *dprdct){
    _visdat = visdat;
    _dctrl = dctrl;
    _trackers = imngr;
    _dtrkr = _trackers->dronetracker();
    _itrkr = _trackers->insecttracker();
    _dnav = dnav;
    _rc = rc;
    _dprdct = dprdct;

    _fromfile = fromfile;
    if (fromfile) {
        _res_mult = 1.5f;
    } else {
        _res_mult = 1;
    }

    thread_viz = std::thread(&Visualizer::workerThread,this);
    initialized = true;
}

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

    track_data data = _dtrkr->Last_track_data();
    dt.push_back(data.dt);
    dt_target.push_back(1.f/VIDEOFPS);

    posX.push_back(-data.posX);
    posY.push_back(data.posY);
    posZ.push_back(-data.posZ);
    disparity.push_back(_dtrkr->find_result.disparity);
    sdisparity.push_back(_dtrkr->find_result.disparity);

    sposX.push_back(-data.sposX);
    sposY.push_back(data.sposY);
    sposZ.push_back(-data.sposZ);

    setposX.push_back(-_dnav->setpoint_pos_world.x);
    setposY.push_back(_dnav->setpoint_pos_world.y);
    setposZ.push_back(-_dnav->setpoint_pos_world.z);

    svelX.push_back(-data.svelX);
    svelY.push_back(data.svelY);
    svelZ.push_back(-data.svelZ);

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
    plotframe = createRowImage(ims_trk,CV_8UC3);
}

cv::Mat Visualizer::plot_xyd(void) {
    std::vector<cv::Mat> ims_xyd;
    ims_xyd.push_back(plot({disparity,sdisparity},"Disparity"));


    cv::Point sp1(-_dnav->setpoint_pos_world.x*1000.f,-_dnav->setpoint_pos_world.z*1000.f);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    ims_xyd.push_back(plotxy(posX,posZ, sp1,"PosXZ",min_xz_range,max_xz_range));

    cv::Point sp2(-_dnav->setpoint_pos_world.x*1000.f,_dnav->setpoint_pos_world.y*1000.f);
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
    ims_vel.push_back(plot({svelX}, "VelX"));
    ims_vel.push_back(plot({svelY,autotakeoff_velY_thresh},"VelY"));
    ims_vel.push_back(plot({svelZ},"VelZ"));
    return createColumnImage(ims_vel,CV_8UC3);
}

cv::Mat Visualizer::plot_all_acceleration(void) {
    std::vector<cv::Mat> ims_acc;
    ims_acc.push_back(plot({saccX}, "AccX"));
    ims_acc.push_back(plot({saccY,autotakeoff_velY_thresh},"AccY"));
    ims_acc.push_back(plot({saccZ},"AccZ"));
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


    float dist_th_x = (_dnav->distance_threshold_mm())* scaleX + 2*line_width;
    float dist_th_y = (_dnav->distance_threshold_mm())* scaleY + 2*line_width;
    cv::rectangle(frame,Point(x-dist_th_x,y-dist_th_y),Point(x+dist_th_x,y+dist_th_y),cv::Scalar(255,0,0));

    return frame;
}

void Visualizer::draw_target_text(cv::Mat resFrame, double time, float dis,float min_dis) {
    std::stringstream ss_time,ss_dis,ss_min;

    ss_time << "T: " << (round(time*100)/100);
    ss_min << "Closest: " << (roundf(min_dis*100)/100) << " [m]";
    ss_dis << "|" << (roundf(dis*100)/100) << "|";

    putText(resFrame,ss_time.str() ,cv::Point(220*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    if (min_dis<9999){
        putText(resFrame,ss_dis.str() ,cv::Point(300*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
        putText(resFrame,ss_min.str() ,cv::Point(360*_res_mult,12*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    }

    putText(resFrame,_dctrl->flight_mode() ,cv::Point(220*_res_mult,70*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,_dnav->navigation_status() ,cv::Point(220*_res_mult,82*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame, _rc->Armed() ,cv::Point(450*_res_mult,82*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame, _dctrl->Joy_State_str() ,cv::Point(525*_res_mult,82*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,_trackers->mode_str() ,cv::Point(220*_res_mult,96*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));

    putText(resFrame,_dtrkr->drone_tracking_state() ,cv::Point(450*_res_mult,96*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,_dnav->get_Interceptor().Interceptor_State(),cv::Point(450*_res_mult,70*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));

    if (_fromfile) {
        static int popcorn_cnt = 0;
        popcorn_cnt++;
        if (popcorn_cnt < 20)
            putText(resFrame,"POPCORN TIME!" ,cv::Point(400*_res_mult,24*_res_mult),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
        if (popcorn_cnt > 35)
            popcorn_cnt  = 0;
    }

}

cv::Mat Visualizer::draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<ItemTracker::WorldItem> path,std::vector<ItemTracker::ImagePredictItem> predicted_path) {

    cv::Mat frameL_small_drone;
    std::vector<ItemTracker::ImagePredictItem> tmp = predicted_path;
    if (tmp.size()>0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< tmp.size();i++) {
            cv::KeyPoint k(tmp.at(i).x,tmp.at(i).y,24/IMSCALEF);
            keypoints.push_back(k);
        }
        drawKeypoints( frameL_small, keypoints, frameL_small_drone, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    } else {
        cvtColor(frameL_small,frameL_small_drone,CV_GRAY2BGR);
    }

    if (path.size() > 0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< path.size();i++) {
            cv::KeyPoint k(path.at(i).iti.x,path.at(i).iti.y,12/IMSCALEF);
            keypoints.push_back(k);
        }
        drawKeypoints( frameL_small_drone, keypoints, frameL_small_drone, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(frameL_small_drone,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(frameL_small_drone,frameL_small_drone,vizsizeL);

    return frameL_small_drone;
}


void Visualizer::update_tracker_data(cv::Mat frameL, cv::Point3d setpoint, double time) {
    if (new_tracker_viz_data_requested) {
        lock_frame_data.lock();

        static float min_dis = 9999;
        float dis = 0;
        if (_dtrkr->n_frames_tracking>0 && _itrkr->n_frames_tracking>0) {
            dis = powf(_dtrkr->Last_track_data().posX-_itrkr->Last_track_data().posX,2) +
                    powf(_dtrkr->Last_track_data().posY-_itrkr->Last_track_data().posY,2) +
                    powf(_dtrkr->Last_track_data().posZ-_itrkr->Last_track_data().posZ,2);
            dis = sqrtf(dis);

            if (dis < min_dis)
                min_dis = dis;
        }

        tracker_viz_base_data.frameL = frameL;
        tracker_viz_base_data.dis = dis;
        tracker_viz_base_data.min_dis = min_dis;
        tracker_viz_base_data.setpoint = setpoint;
        tracker_viz_base_data.time = time;
        tracker_viz_base_data.drn_path = _dtrkr->path;
        tracker_viz_base_data.drn_predicted_path = _dtrkr->predicted_image_path;
        tracker_viz_base_data.ins_path = _itrkr->path;
        tracker_viz_base_data.ins_predicted_path = _itrkr->predicted_image_path;

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

    std::vector<ItemTracker::WorldItem> drn_path = tracker_viz_base_data.drn_path;
    std::vector<ItemTracker::ImagePredictItem> drn_predicted_path = tracker_viz_base_data.drn_predicted_path;
    std::vector<ItemTracker::WorldItem> ins_path = tracker_viz_base_data.ins_path;
    std::vector<ItemTracker::ImagePredictItem> ins_predicted_path = tracker_viz_base_data.ins_predicted_path;

    cv::Mat resFrame = cv::Mat::zeros(viz_frame_size(),CV_8UC3);
    cv::Mat frameL_color;
    cvtColor(frameL,frameL_color,CV_GRAY2BGR);
    cv::Rect rect(0,frameL.rows*_res_mult/4,frameL.cols*_res_mult,frameL.rows*_res_mult);
    cv::Mat roi = resFrame(rect);
    cv::Size size (frameL.cols*_res_mult,frameL.rows*_res_mult);

    if ( drn_predicted_path.size()>0 ) {
        auto pred = drn_predicted_path.back();
        cv::circle(frameL_color,pred.pt()*IMSCALEF,pred.size*IMSCALEF,cv::Scalar(0,255,0));
    }

    if (ins_path.size()>0){
        std::stringstream ss;
        ItemTracker::WorldItem wti = ins_path.back();
        ss << "i " << to_string_with_precision(wti.distance_background,1);
        cv::Scalar c(0,0,255);
        if (wti.distance_background >wti.distance )
            c = cv::Scalar(180,180,255);
        cv::Point2i p (wti.iti.x*IMSCALEF,wti.iti.y*IMSCALEF);
        putText(frameL_color,ss.str(),p,cv::FONT_HERSHEY_SIMPLEX,0.5,c);
        cv::line(frameL_color,p,p,c,2);
    }
    if (drn_path.size()>0){
        std::stringstream ss;
        ItemTracker::WorldItem wti = drn_path.back();
        ss << "d " << to_string_with_precision(wti.distance_background,1);
        cv::Scalar c(0,0,255);
        if (wti.distance_background >wti.distance )
            c = cv::Scalar(180,180,255);
        cv::Point2i p (wti.iti.x*IMSCALEF,wti.iti.y*IMSCALEF);
        putText(frameL_color,ss.str(),p,cv::FONT_HERSHEY_SIMPLEX,0.5,c);
        cv::line(frameL_color,p,p,c,2);

        //draw line to drone target setpoint
        if (_dnav->drone_is_flying()){
            cv::Point2i t = _dnav->drone_setpoint_im();
            cv::Scalar c2;
            if (_dnav->drone_is_hunting() && t.x+t.y>0 ) {
                c2 = cv::Scalar(0,0,255);
                cv::Point2i t2 = p - (p - t)/2;
                putText(frameL_color,to_string_with_precision(_dnav->get_Interceptor().time_to_intercept(),2),t2,cv::FONT_HERSHEY_SIMPLEX,0.5,c2);
            } else
                c2 = cv::Scalar(255,255,255);
            cv::line(frameL_color,p,t,c2,1);

            //draw speed vector:
            cv::Point2i tv = _dnav->drone_v_setpoint_im();
            cv::line(frameL_color,p,tv,cv::Scalar(0,255,0),1);
        }
    }
    cv::resize(frameL_color,roi,size);

    cv::Size vizsizeL(size.width/4,size.height/4);
    cv::Mat frameL_small;
    cv::resize(frameL,frameL_small,cv::Size(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF));
    cv::Mat frameL_small_drone = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,drn_path,drn_predicted_path);
    cv::Mat frameL_small_insect = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,ins_path,ins_predicted_path);
    frameL_small_drone.copyTo(resFrame(cv::Rect(0,0,frameL_small_drone.cols, frameL_small_drone.rows)));
    frameL_small_insect.copyTo(resFrame(cv::Rect(resFrame.cols-frameL_small_drone.cols,0,frameL_small_drone.cols, frameL_small_drone.rows)));
    draw_target_text(resFrame,time,dis,min_dis);

    trackframe = resFrame;
}

void Visualizer::paint() {
    if (request_trackframe_paint) {
        request_trackframe_paint = false;
        cv::imshow("tracking results", trackframe);
        if (_visdat->viz_frame.cols>0)
            cv::imshow("diff",_visdat->viz_frame);
        if (_dtrkr->diff_viz.cols > 0)
            cv::imshow("drn_diff", _dtrkr->diff_viz);
        if (_trackers->viz_max_points.cols > 0)
            cv::imshow("motion points", _trackers->viz_max_points);
        if (_trackers->diff_viz.cols > 0)
            cv::imshow("diff", _trackers->diff_viz);
        new_tracker_viz_data_requested = true;
    }
    if (request_plotframe_paint && plotframe.rows > 0) {
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
        if (tracker_viz_base_data.frameL.rows>0) {
            draw_tracker_viz();
            request_trackframe_paint=true;
        }
        lock_frame_data.unlock();
#endif

    }
}
void Visualizer::close() {
    if (initialized){
        std::cout << "Closing visualizer" << std::endl;
        exitVizThread = true;
        newdata.notify_all();
        lock_plot_data.unlock();
        thread_viz.join();
        initialized = false;
    }
}
