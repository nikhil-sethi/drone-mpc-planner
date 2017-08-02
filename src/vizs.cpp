#include "vizs.h"

void Visualizer::addSample(void) {

    roll_joystick.push_back(cam->getCurrentRoll());
    roll_calculated.push_back((float)dctrl->commandedRoll);

    pitch_joystick.push_back(cam->getCurrentPitch());
    pitch_calculated.push_back((float)dctrl->commandedPitch);

    yaw_joystick.push_back(cam->getCurrentYaw());
    yaw_calculated.push_back((float)dctrl->commandedYaw);

    throttle_joystick.push_back(cam->getCurrentThrust());
    throttle_calculated.push_back((float)dctrl->commandedThrottle);

}
const int fsizex = 500;
const int fsizey = 300;
const int line_width = 1;

void Visualizer::plot(void) {
    addSample();

    cv::Mat frame = cv::Mat::zeros((fsizey+4*line_width)*3, fsizex+4*line_width, CV_8UC3);

    cv::Mat frameRoll = cv::Mat(frame, cv::Rect(cv::Point(0, 0),cv::Point(frame.cols, frame.rows/3)));
    cv::Mat framePitch = cv::Mat(frame, cv::Rect(cv::Point(0, frame.rows/3),cv::Point(frame.cols, frame.rows/3*2)));
    cv::Mat frameThrottle = cv::Mat(frame, cv::Rect(cv::Point(0, frame.rows/3*2),cv::Point(frame.cols, frame.rows)));

    //plot(roll_joystick,roll_calculated, &frameRoll,"Roll");
    //plot(pitch_joystick,pitch_calculated, &framePitch,"Pitch");
    plot(throttle_joystick,throttle_calculated, &frameThrottle,"Throttle");

    imshow("Shizzle",frame);
}

void Visualizer::plot(cv::Mat data1,cv::Mat data2, const std::string name) {
    cv::Mat frame = cv::Mat::zeros(fsizey+4*line_width, fsizex+4*line_width, CV_8UC3);
    plot(data1,data2, &frame,name);
    imshow(name,frame);
}

void Visualizer::plot(cv::Mat data1,cv::Mat data2, cv::Mat *frame, std::string name) {
    putText(*frame,name,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));

    double min,max;

    cv::Mat tmp;
    tmp.push_back(data1);
    tmp.push_back(data2);
    cv::minMaxIdx(tmp,&min,&max,NULL,NULL);

    min-=1;
    max+=1;

    const float scaleX = (float)((fsizex))/(data1.rows);
    const float scaleY = (fsizey)/(max-min);

    int prev_y1 =0;
    int prev_y2 =0;
    for (int j = 0; j < data1.rows-1; j++)  {
        int y1 = data1.at<float>(j,1) - min;
        int y2 = data2.at<float>(j,1) - min;
        cv::line(*frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y1*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey -  y1*scaleY +2*line_width), cv::Scalar(0,255,0), line_width, CV_AA, 0);
        cv::line(*frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y2*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey -  y2*scaleY +2*line_width), cv::Scalar(255,0,0), line_width, CV_AA, 0);
        prev_y1 = y1;
        prev_y2 = y2;
    }
}
