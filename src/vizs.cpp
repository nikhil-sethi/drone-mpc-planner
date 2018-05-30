#include "vizs.h"

cv::Scalar white(255,255,255);
cv::Scalar black(0,0,0);
cv::Scalar green(0,255,0);
cv::Scalar blue(255,0,0);
cv::Scalar red(0,0,255);
cv::Scalar linecolors[] = {green,blue,red,cv::Scalar(0,255,255),cv::Scalar(255,255,0),cv::Scalar(255,0,255)};

//cv::Scalar background_color(255,255,255);
cv::Scalar background_color(0,0,0);
cv::Scalar fore_color(255,255,255);

void Visualizer::addSample(void) {


    g_lockData.lock();


    static int div = 0;
    if (paint && div++ % 4 == 1) {
        imshow("Tracking",resframe);
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

    setposX.push_back(-(float)dtrkr->setpointw.x);
    setposY.push_back((float)dtrkr->setpointw.y);
    setposZ.push_back(-(float)dtrkr->setpointw.z);

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


    cv::Point sp1(dtrkr->setpointw.x,-dtrkr->setpointw.z);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    ims_xyd.push_back(plotxy(posX,posZ, sp1,"PosXZ",min_xz_range,max_xz_range));

    cv::Point sp2(dtrkr->setpointw.x,dtrkr->setpointw.y);
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
