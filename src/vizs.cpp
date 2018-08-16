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

    g_lockData.lock();


    static int div = 0;
    if (paint && div++ % 4 == 1) {
        imshow("Plots",plotframe);
        paint = false;

        //imshow("dt", plot({dt,dt_target},"dt"));
    }


    roll_joystick.push_back((float)_dctrl->joyRoll);
    pitch_joystick.push_back((float)_dctrl->joyPitch);
    yaw_joystick.push_back((float)_dctrl->joyPitch);
    throttle_joystick.push_back((float)_dctrl->joyThrottle);

    roll_calculated.push_back((float)_dctrl->autoRoll);
    pitch_calculated.push_back((float)_dctrl->autoPitch);
    //    yaw_calculated.push_back((float)_dctrl->commandedYaw);
    throttle_calculated.push_back((float)_dctrl->autoThrottle);
    throttle_hover.push_back(_dctrl->hoverthrottle);

    trackData data = _dtrk->get_last_track_data();
    dt.push_back(data.dt);
    dt_target.push_back(1.f/VIDEOFPS);

    posX.push_back(-data.posX);
    posY.push_back(data.posY);
    posZ.push_back(-data.posZ);
    disparity.push_back(data.disparity);
    sdisparity.push_back(data.sdisparity);

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

    autotakeoff_velY_thresh.push_back((float)(_dctrl->params.auto_takeoff_speed) / 100.f);

    g_lockData.unlock();
#endif
}

void Visualizer::plot(void) {
    std::vector<cv::Mat> ims_trk;
    ims_trk.push_back(plot_xyd());
    ims_trk.push_back(plot_all_position());
    ims_trk.push_back(plot_all_velocity());
    ims_trk.push_back(plot_all_control());
    //showRowImage(ims_trk, "Tracking",CV_8UC3);
    plotframe = createRowImage(ims_trk,CV_8UC3);
    paint=true;
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
    min = (float)mind;
    max = (float)maxd;
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
    const float scaleX = (float)((fsizex))/(bufsize);
    const float scaleY = ((float)fsizey)/(max-min);

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
        x = xS.at<float>(j,1) - (float)minx;
        x =x*scaleX + 2*line_width;
        y = yS.at<float>(j,1) - (float)miny;
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

void Visualizer::draw_segment_viz(){
    std::vector<cv::Mat> ims;
    ims.push_back(cir8);
    ims.push_back(bkg8);
    ims.push_back(dif8);
    ims.push_back(_dtrkr->_approx);
    ims.push_back(_dtrkr->_treshL);
    showColumnImage(ims,"drone_roi",CV_8UC1);
}

void Visualizer::draw_target_text(cv::Mat resFrame) {
    std::stringstream ss1,ss2,ss3;
    ss1.precision(2);
    ss2.precision(2);
    ss3.precision(2);

    trackData data = _dtrkr->get_last_track_data();
    ss1 << "[" << data.posX << ", " << data.posY << ", " << data.posZ << "] " ;
    ss2 << "[" << data.posErrX << ", " << data.posErrY << ", " << data.posErrZ << "] " ;
    ss3 << "Delta: " << sqrtf(data.posErrX*data.posErrX+data.posErrY*data.posErrY+data.posErrZ*data.posErrZ);

    putText(resFrame,ss1.str() ,cv::Point(220,20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,ss2.str() ,cv::Point(220,40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,ss3.str() ,cv::Point(220,60),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));

}

cv::Mat Visualizer::draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, ItemTracker *trkr) {
    cv::Mat frameL_small_drone;
    std::vector<ItemTracker::track_item> tmp = trkr->predicted_pathL;
    if (tmp.size()>0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< tmp.size();i++) {
            keypoints.push_back(tmp.at(i).k);
        }
        drawKeypoints( frameL_small, keypoints, frameL_small_drone, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
    } else {
        cvtColor(frameL_small,frameL_small_drone,CV_GRAY2BGR);
    }

    cv::rectangle(frameL_small_drone,trkr->find_result.roi_offset,cv::Scalar(180,100,240),4/IMSCALEF);

    if (trkr->find_result.excludes.size() > 0) {
        ItemTracker::track_item exclude = trkr->find_result.excludes.at(trkr->find_result.excludes.size()-1);
        float threshold_dis = trkr->settings.exclude_min_distance / sqrtf(exclude.tracking_certainty);
        if (threshold_dis > trkr->settings.exclude_max_distance)
            threshold_dis = trkr->settings.exclude_max_distance;
        cv::circle(frameL_small_drone,exclude.k.pt,threshold_dis,cv::Scalar(255,0,0),4/IMSCALEF);
    }


    if (trkr->pathL.size() > 0) {
        std::vector<cv::KeyPoint> keypoints;
        for (uint i = 0; i< trkr->pathL.size();i++) {
            keypoints.push_back(trkr->pathL.at(i).k);
        }
        drawKeypoints( frameL_small_drone, keypoints, frameL_small_drone, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(frameL_small_drone,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(frameL_small_drone,frameL_small_drone,vizsizeL);

    return frameL_small_drone;
}


void Visualizer::draw_tracker_viz(cv::Mat frameL,cv::Mat frameL_small, cv::Point3d setpoint) {
#ifdef DRAWTRACKING
    static int div = 0;
    if (div++ % 4 == 1) {

        cv::Mat resFrame;
        cvtColor(frameL,resFrame,CV_GRAY2BGR);

        cir8 = _dtrkr->_cir*255;
        bkg8 = _dtrkr->_bkg*255;
        dif8 = _dtrkr->_dif*10;
        cir8.convertTo(cir8, CV_8UC1);
        bkg8.convertTo(bkg8, CV_8UC1);
        dif8.convertTo(dif8, CV_8UC1);

//        draw_segment_viz();

        cv::Size vizsizeL(resFrame.cols/4,resFrame.rows/4);
        cv::Mat frameL_small_drone = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,_dtrkr);
        cv::Mat frameL_small_insect = draw_sub_tracking_viz(frameL_small,vizsizeL,setpoint,_itrkr);
        frameL_small_drone.copyTo(resFrame(cv::Rect(0,0,frameL_small_drone.cols, frameL_small_drone.rows)));
        frameL_small_insect.copyTo(resFrame(cv::Rect(resFrame.cols-frameL_small_drone.cols,0,frameL_small_drone.cols, frameL_small_drone.rows)));
        draw_target_text(resFrame);


        cv::imshow("tracking results", resFrame);
        trackframe   = resFrame;
    }
#endif
}


void Visualizer::workerThread(void) {
    std::cout << "Viz thread started!" << std::endl;
    while (!exitVizThread) {
        if (roll_joystick.rows > 0) {
            g_lockData.lock();
#ifdef DRAWPLOTS
            plot();
#endif
            g_lockData.unlock();
        }
        usleep(2000);
    }
}
void Visualizer::close() {
    exitVizThread = true;
    g_lockData.unlock();
    thread_viz.join();
}
