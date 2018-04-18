#include "vizs.h"

cv::Scalar white(255,255,255);
cv::Scalar black(0,0,0);
cv::Scalar green(0,255,0);
cv::Scalar blue(255,0,0);
cv::Scalar red(0,0,255);


void Visualizer::addSample(void) {

    roll_joystick.push_back((float)dctrl->roll);
    pitch_joystick.push_back((float)dctrl->pitch);
    yaw_joystick.push_back((float)dctrl->yaw);
    throttle_joystick.push_back((float)dctrl->joyThrottle);

    roll_calculated.push_back((float)dctrl->autoRoll);
    pitch_calculated.push_back((float)dctrl->autoPitch);
    //yaw_calculated.push_back((float)dctrl->commandedYaw);
    throttle_calculated.push_back((float)dctrl->autoThrottle);

    posX.push_back(-(float)dtrkr->data.csposX);
    posY.push_back((float)dtrkr->data.csposY);
    posZ.push_back(-(float)dtrkr->data.csposZ);


}
const int fsizex = 500;
const int fsizey = 300;
const int line_width = 1;

void Visualizer::plot(void) {
    addSample();

    cv::Mat frame = cv::Mat::zeros((fsizey+4*line_width)*3, fsizex+4*line_width, CV_8UC3);
    frame.setTo(cv::Scalar(255,255,255));

    //cv::Mat frameRoll = cv::Mat(frame, cv::Rect(cv::Point(0, 0),cv::Point(frame.cols, frame.rows/3)));
    cv::Mat framePosXZ = cv::Mat(frame, cv::Rect(cv::Point(0, 0),cv::Point(frame.cols, frame.rows/3)));

    //cv::Mat framePitch = cv::Mat(frame, cv::Rect(cv::Point(0, frame.rows/3),cv::Point(frame.cols, frame.rows/3*2)));
    cv::Mat framePosXY = cv::Mat(frame, cv::Rect(cv::Point(0, frame.rows/3),cv::Point(frame.cols, frame.rows/3*2)));

    cv::Mat frameThrottle = cv::Mat(frame, cv::Rect(cv::Point(0, frame.rows/3*2),cv::Point(frame.cols, frame.rows)));


    //plot(roll_joystick,roll_calculated, &frameRoll,"Roll");
    cv::Point sp1(dtrkr->setpointw.x,-dtrkr->setpointw.z);
    cv::Point min_xz_range,max_xz_range;
    min_xz_range.x =-3000;
    max_xz_range.x = 3000;
    min_xz_range.y = 0; // z
    max_xz_range.y = 5000; // z
    plotxy(posX,posZ, &framePosXZ,sp1,"PosXZ",min_xz_range,max_xz_range);

    //plot(pitch_joystick,pitch_calculated, &framePitch,"Pitch");

    cv::Point sp2(dtrkr->setpointw.x,dtrkr->setpointw.y);
    cv::Point min_xy_range,max_xy_range;
    min_xy_range.x =-3000;
    max_xy_range.x = 3000;
    min_xy_range.y =-3000;
    max_xy_range.y = 3000;
    plotxy(posX,posY,&framePosXY, sp2, "PosXY",min_xy_range,max_xy_range);

    //plot(throttle_joystick,throttle_calculated, &frameThrottle,"Throttle");
    //plot(roll_joystick,roll_calculated, &frameThrottle,"Roll");
    plot(throttle_joystick,throttle_calculated, &frameThrottle,"Throttle");

    imshow("Shizzle",frame);
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
    min-=1;
    max+=1;

    const float scaleX = (float)((fsizex))/(data1.rows);
    const float scaleY = (fsizey)/(max-min);

    int start = data1.rows -bufsize;
    if (start < 0)
        start = 0;

    int prev_y1 =0;
    int prev_y2 =0;
    for (int j = start; j < data1.rows-1; j++)  {
        int y1 = data1.at<float>(j,1) - min;
        int y2 = data2.at<float>(j,1) - min;
        if (j > start) {
            j-=start;
            cv::line(*frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y1*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey - y1*scaleY +2*line_width), green, line_width, CV_AA, 0);
            cv::line(*frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y2*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey - y2*scaleY +2*line_width), blue, line_width, CV_AA, 0);
            j+=start;
        }
        prev_y1 = y1;
        prev_y2 = y2;
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
