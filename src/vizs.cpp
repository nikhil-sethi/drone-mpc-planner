#include "vizs.h"
using namespace cv;
using namespace std;

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


    g_lockData.lock();


    static int div = 0;
    if (paint && div++ % 4 == 1) {
        imshow("Plots",resframe);
        paint = false;

        //imshow("dt", plot({dt,dt_target},"dt"));
    }


    roll_joystick.push_back((float)dctrl->joyRoll);
    pitch_joystick.push_back((float)dctrl->joyPitch);
    yaw_joystick.push_back((float)dctrl->joyPitch);
    throttle_joystick.push_back((float)dctrl->joyThrottle);

    roll_calculated.push_back((float)dctrl->autoRoll);
    pitch_calculated.push_back((float)dctrl->autoPitch);
    //    yaw_calculated.push_back((float)dctrl->commandedYaw);
    throttle_calculated.push_back((float)dctrl->autoThrottle);
    throttle_hover.push_back((float)dctrl->hoverthrottle);

    dt.push_back((float)dtrkr->data.dt);
    dt_target.push_back((float)1.f/VIDEOFPS);

    posX.push_back(-(float)dtrkr->data.posX);
    posY.push_back((float)dtrkr->data.posY);
    posZ.push_back(-(float)dtrkr->data.posZ);
    disparity.push_back((float)dtrkr->data.disparity);
    sdisparity.push_back((float)dtrkr->data.sdisparity);

    sposX.push_back(-(float)dtrkr->data.sposX);
    sposY.push_back((float)dtrkr->data.sposY);
    sposZ.push_back(-(float)dtrkr->data.sposZ);

    setposX.push_back(-(float)dnav->setpoint_world.x);
    setposY.push_back((float)dnav->setpoint_world.y);
    setposZ.push_back(-(float)dnav->setpoint_world.z);

    velX.push_back(-(float)dtrkr->data.velX);
    velY.push_back((float)dtrkr->data.velY);
    velZ.push_back(-(float)dtrkr->data.velZ);

    svelX.push_back(-(float)dtrkr->data.svelX);
    svelY.push_back((float)dtrkr->data.svelY);
    svelZ.push_back(-(float)dtrkr->data.svelZ);

    autotakeoff_velY_thresh.push_back((float)(dctrl->params.auto_takeoff_speed) / 100.f);

    g_lockData.unlock();

}

void Visualizer::plot(void) {
    std::vector<cv::Mat> ims_trk;
    ims_trk.push_back(plot_xyd());
    ims_trk.push_back(plot_all_position());
    ims_trk.push_back(plot_all_velocity());
    ims_trk.push_back(plot_all_control());
    //showRowImage(ims_trk, "Tracking",CV_8UC3);
    resframe = createRowImage(ims_trk,CV_8UC3);
    paint=true;
}

cv::Mat Visualizer::plot_xyd(void) {
    std::vector<cv::Mat> ims_xyd;
    ims_xyd.push_back(plot({disparity,sdisparity},"Disparity"));


    cv::Point sp1(dnav->setpoint_world.x,-dnav->setpoint_world.z);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    ims_xyd.push_back(plotxy(posX,posZ, sp1,"PosXZ",min_xz_range,max_xz_range));

    cv::Point sp2(dnav->setpoint_world.x,dnav->setpoint_world.y);
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

    double min,max;
    cv::Mat tmp;
    for (int i = 0 ; i< data.size();i++) {
        cv::Mat vec = data.at(i);
        cv::Mat vect = cv::Mat(vec,cv::Rect(cv::Point(0,start),cv::Point(1,start+current_buf_size)));
        tmp.push_back(vect);
    }
    cv::minMaxIdx(tmp,&min,&max,NULL,NULL);

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

    for (int i = 0 ; i< data.size();i++) {
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
        x = xS.at<float>(j,1) - minx;
        x =x*scaleX + 2*line_width;
        y = yS.at<float>(j,1) - miny;
        y= fsizey - y*scaleY + 2*line_width;
        if (j > start)
            cv::line(frame, cv::Point(prev_x, prev_y) , cv::Point(x, y), green, line_width, CV_AA, 0);
        prev_x = x;
        prev_y = y;
    }

    //draw current position more clearly
    cv::circle(frame,cv::Point(x,y),2,red);

    //draw the setpoint
    x = setpoint.x*1000 - minx;
    x = x* scaleX + 2*line_width;
    y = setpoint.y*1000 - miny;
    y= fsizey - y*scaleY + 2*line_width;
    cv::line(frame,cv::Point(0,y),cv::Point(frame.cols,y),cv::Scalar(80,80,150));
    cv::line(frame,cv::Point(x,0),cv::Point(x,frame.rows),cv::Scalar(80,80,150));

    return frame;
}

void Visualizer::draw_segment_viz(){



    std::vector<cv::Mat> ims;
    ims.push_back(cir8);
    ims.push_back(bkg8);
    ims.push_back(dif8);
    ims.push_back(dtrkr->approx);
    ims.push_back(dtrkr->treshL);
    showColumnImage(ims,"drone_roi",CV_8UC1);
}

void Visualizer::draw_target_text(cv::Mat resFrame) {
    std::stringstream ss1,ss2,ss3;
    ss1.precision(2);
    ss2.precision(2);
    ss3.precision(2);

    ss1 << "[" << dtrkr->data.posX << ", " << dtrkr->data.posY << ", " << dtrkr->data.posZ << "] " ;
    ss2 << "[" << dtrkr->data.posErrX << ", " << dtrkr->data.posErrY << ", " << dtrkr->data.posErrZ << "] " ;
    ss3 << "Delta: " << sqrtf(dtrkr->data.posErrX*dtrkr->data.posErrX+dtrkr->data.posErrY*dtrkr->data.posErrY+dtrkr->data.posErrZ*dtrkr->data.posErrZ);

    putText(resFrame,ss1.str() ,cv::Point(220,20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,ss2.str() ,cv::Point(220,40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrame,ss3.str() ,cv::Point(220,60),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));

}

cv::Mat Visualizer::draw_sub_tracking_drone_viz(cv::Mat frameL_small,cv::Size vizsizeL,cv::Point3d setpoint) {
    cv::Mat frameL_small_drone;
    if (dtrkr->predicted_drone_pathL.size()>0) {
        drawKeypoints( frameL_small, dtrkr->predicted_drone_pathL, frameL_small_drone, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
    } else {
        cvtColor(frameL_small,frameL_small_drone,CV_GRAY2BGR);
    }

    cv::rectangle(frameL_small_drone,dtrkr->find_drone_result.roi_offset,cv::Scalar(180,100,240),4/IMSCALEF);

    if (dtrkr->drone_pathL.size() > 0) {
        drawKeypoints( frameL_small_drone, dtrkr->drone_pathL, frameL_small_drone, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(frameL_small_drone,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(frameL_small_drone,frameL_small_drone,vizsizeL);

    return frameL_small_drone;
}
cv::Mat Visualizer::draw_sub_tracking_insect_viz(cv::Mat frameL_small,cv::Size vizsizeL,cv::Point3d setpoint) { // TODO: make a generic tracker class
    cv::Mat frameL_small_drone;
    if (itrkr->predicted_insect_pathL.size()>0) {
        drawKeypoints( frameL_small, itrkr->predicted_insect_pathL, frameL_small_drone, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
    } else {
        cvtColor(frameL_small,frameL_small_drone,CV_GRAY2BGR);
    }

    cv::rectangle(frameL_small_drone,itrkr->find_insect_result.roi_offset,cv::Scalar(180,100,240),4/IMSCALEF);

    if (itrkr->insect_pathL.size() > 0) {
        drawKeypoints( frameL_small_drone, itrkr->insect_pathL, frameL_small_drone, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(frameL_small_drone,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(frameL_small_drone,frameL_small_drone,vizsizeL);

    return frameL_small_drone;
}


void Visualizer::draw_tracker_viz(cv::Mat frameL,cv::Mat frameL_small, cv::Point3d setpoint) {

    static int div = 0;
    if (div++ % 4 == 1) {

        cv::Mat resFrame;
        cvtColor(frameL,resFrame,CV_GRAY2BGR);

        cir8 = dtrkr->cir*255;
        bkg8 = dtrkr->bkg*255;
        dif8 = dtrkr->dif*10;
        cir8.convertTo(cir8, CV_8UC1);
        bkg8.convertTo(bkg8, CV_8UC1);
        dif8.convertTo(dif8, CV_8UC1);

        draw_segment_viz();

        cv::Size vizsizeL(resFrame.cols/4,resFrame.rows/4);
        cv::Mat frameL_small_drone = draw_sub_tracking_drone_viz(frameL_small,vizsizeL,setpoint);
        cv::Mat frameL_small_insect = draw_sub_tracking_insect_viz(frameL_small,vizsizeL,setpoint);
        frameL_small_drone.copyTo(resFrame(cv::Rect(0,0,frameL_small_drone.cols, frameL_small_drone.rows)));
        frameL_small_insect.copyTo(resFrame(cv::Rect(resFrame.cols-frameL_small_drone.cols,0,frameL_small_drone.cols, frameL_small_drone.rows)));
        draw_target_text(resFrame);


        cv::imshow("tracking results", resFrame);
    }
}


void Visualizer::workerThread(void) {
    std::cout << "Viz thread started!" << std::endl;
    while (!exitVizThread) {
        if (roll_joystick.rows > 0) {
            g_lockData.lock();
            plot();
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
