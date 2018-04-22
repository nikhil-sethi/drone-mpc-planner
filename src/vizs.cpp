#include "vizs.h"

cv::Scalar white(255,255,255);
cv::Scalar black(0,0,0);
cv::Scalar green(0,255,0);
cv::Scalar blue(255,0,0);
cv::Scalar red(0,0,255);


void Visualizer::addSample(void) {

    roll_joystick.push_back((float)dctrl->joyRoll);
    pitch_joystick.push_back((float)dctrl->joyPitch);
    yaw_joystick.push_back((float)dctrl->joyPitch);
    throttle_joystick.push_back((float)dctrl->joyThrottle);

    roll_calculated.push_back((float)dctrl->autoRoll);
    pitch_calculated.push_back((float)dctrl->autoPitch);
//    yaw_calculated.push_back((float)dctrl->commandedYaw);
    throttle_calculated.push_back((float)dctrl->autoThrottle);

    posX.push_back(-(float)dtrkr->data.csposX);
    posY.push_back((float)dtrkr->data.csposY);
    posZ.push_back(-(float)dtrkr->data.csposZ);

    velX.push_back(-(float)dtrkr->data.velX*1000);
    velY.push_back((float)dtrkr->data.velY*1000);
    velZ.push_back(-(float)dtrkr->data.velZ*1000);

    svelX.push_back(-(float)dtrkr->data.svelX*1000);
    svelY.push_back((float)dtrkr->data.svelY*1000);
    svelZ.push_back(-(float)dtrkr->data.svelZ*1000);

}
const int fsizex = 500;
const int fsizey = 300;
const int line_width = 1;

//cv::Scalar background_color(255,255,255);
cv::Scalar background_color(0,0,0);

void Visualizer::plot(void) {
    addSample();

    cv::Mat frame_xz = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_xz.setTo(background_color);
    cv::Point sp1(dtrkr->setpointw.x,-dtrkr->setpointw.z);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    plotxy(posX,posZ, &frame_xz,sp1,"PosXZ",min_xz_range,max_xz_range);

    cv::Mat frame_xy = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_xy.setTo(background_color);
    cv::Point sp2(dtrkr->setpointw.x,dtrkr->setpointw.y);
    cv::Point min_xy_range,max_xy_range;
    min_xy_range.x =-3000;
    max_xy_range.x = 3000;
    min_xy_range.y =-3000;
    max_xy_range.y = 3000;
    plotxy(posX,posY,&frame_xy, sp2, "PosXY",min_xy_range,max_xy_range);

    std::vector<cv::Mat> ims_detect;
    ims_detect.push_back(frame_xz);
    ims_detect.push_back(frame_xy);
    showColumnImage(ims_detect, "Detection",CV_8UC3);

    cv::Mat frame_throttle = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_throttle.setTo(background_color);
    plot(throttle_joystick,throttle_calculated, &frame_throttle,"Throttle");

    cv::Mat frame_pitch = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_pitch.setTo(background_color);
    plot(pitch_joystick,pitch_calculated, &frame_pitch,"Pitch");

    cv::Mat frame_roll = cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_roll.setTo(background_color);
    plot(roll_joystick,roll_calculated, &frame_roll,"Roll");

    std::vector<cv::Mat> ims_joy;
    ims_joy.push_back(frame_roll);
    ims_joy.push_back(frame_pitch);
    ims_joy.push_back(frame_throttle);
    showColumnImage(ims_joy, "Joystick",CV_8UC3);



    cv::Mat frame_velX= cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_velX.setTo(background_color);
    plot(velX,svelX, &frame_velX,"VelX");

    cv::Mat frame_velY= cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_velY.setTo(background_color);
    plot(velY,svelY, &frame_velY,"VelY");

    cv::Mat frame_velZ= cv::Mat::zeros((fsizey+4*line_width), fsizex+4*line_width, CV_8UC3);
    frame_velZ.setTo(background_color);
    plot(velZ,svelZ, &frame_velZ,"VelZ");

    std::vector<cv::Mat> ims_vel;
    ims_vel.push_back(frame_velX);
    ims_vel.push_back(frame_velY);
    ims_vel.push_back(frame_velZ);
    showColumnImage(ims_vel, "Velocities",CV_8UC3);

}

void Visualizer::plot(cv::Mat data1,cv::Mat data2, const std::string name) {
    cv::Mat frame = cv::Mat::zeros(fsizey+4*line_width, fsizex+4*line_width, CV_8UC3);
    plot(data1,data2, &frame,name);
    imshow(name,frame);
}

void Visualizer::plot(cv::Mat data1,cv::Mat data2, cv::Mat *frame, std::string name) {
    putText(*frame,name,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,0.5,black);
    cv::line(*frame,cv::Point(0,frame->rows-1),cv::Point(frame->cols,frame->rows-1),black);

    double min,max;
    cv::Mat tmp;
    tmp.push_back(data1);
    tmp.push_back(data2);
    cv::minMaxIdx(tmp,&min,&max,NULL,NULL);

    putText(*frame,"[" + std::to_string((int)min) + " - " + std::to_string((int)max) + "]",cv::Point(frame->cols-130, 12),cv::FONT_HERSHEY_SIMPLEX,0.5,red);


    min-=1;
    max+=1;

    const float scaleX = (float)((fsizex))/(bufsize);
    const float scaleY = (fsizey)/(max-min);

    int start = data1.rows -bufsize;
    if (start < 0)
        start = 0;

    int prev_y1 =0;
    int prev_y2 =0;
    int prev_x=0;
    for (int j = start; j < data1.rows-1; j++)  {
        int y1 = data1.at<float>(j,1) - min;
        int y2 = data2.at<float>(j,1) - min;
        int x = (j-start)*scaleX + 2*line_width;
        cv::line(*frame, cv::Point(prev_x, fsizey- prev_y1*scaleY +line_width*2) , cv::Point(x, fsizey - y1*scaleY +2*line_width), green, line_width, CV_AA, 0);
        cv::line(*frame, cv::Point(prev_x, fsizey- prev_y2*scaleY +line_width*2) , cv::Point(x, fsizey - y2*scaleY +2*line_width), blue, line_width, CV_AA, 0);
        prev_y1 = y1;
        prev_y2 = y2;
        prev_x = x;
    }
}

void Visualizer::plotxy(cv::Mat datax,cv::Mat datay, cv::Mat *frame, cv::Point setpoint, std::string name,cv::Point minaxis,cv::Point maxaxis) {
    std::stringstream ss;
    ss.precision(2);
    ss << name << " " << datax.at<float>(datax.rows-1) << "; " << datay.at<float>(datay.rows-1);


    putText(*frame,ss.str() ,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,0.5,black);
    cv::line(*frame,cv::Point(0,frame->rows-1),cv::Point(frame->cols,frame->rows-1),black);

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

//    minx=-3000;
//    maxx=3000;
//    miny=-3000;
//    maxy=3000;

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
            cv::line(*frame, cv::Point(prev_x, prev_y) , cv::Point(x, y), green, line_width, CV_AA, 0);
        prev_x = x;
        prev_y = y;
    }

    //draw current position more clearly
    cv::circle(*frame,cv::Point(x,y),2,red);

    //draw the setpoint
    x = setpoint.x*1000 - minx;
    x = x* scaleX + 2*line_width;
    y = setpoint.y*1000 - miny;
    y= fsizey - y*scaleY + 2*line_width;
    cv::line(*frame,cv::Point(0,y),cv::Point(frame->cols,y),cv::Scalar(80,80,150));
    cv::line(*frame,cv::Point(x,0),cv::Point(x,frame->rows),cv::Scalar(80,80,150));

}
