#include "stereoalg.h"

#include <stdio.h>
#include <fstream>
#include <iostream>


//opencv
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace cv;

bool stereoAlg::init (std::string calib_folder) {

    frameD = cv::Mat::zeros(96,96, CV_8UC3);

    FileStorage fs(calib_folder + "intrinsics.yml", CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        std::cout << "Error did not find intrinsics" << std::endl;
        return -1;
    }
    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(calib_folder + "extrinsics.yml", CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        std::cout << "Error did not find extrinsics" << std::endl;
        return -1;
    }

    Size img_size = cv::Size(ROIsize,ROIsize);
    Mat R1, R2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;

    //Mat R, T;
    //fs["R"] >> R;
    //fs["T"] >> T;
    //fisheye::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, pt1, pt2, Qf, CALIB_ZERO_DISPARITY); // gives wrong P matrices!

    float fx0 = M1.at<double>(0,0) / IMSCALEF;
    float fy0 = M1.at<double>(1,1) / IMSCALEF;
    float cx =  ROIsizeH;
    float cy =  ROIsizeH;



    cv::Matx33f P1(fx0, 0. , cx,
                   0. , fy0, cy,
                   0. , 0. , 1.);
    float fx1 = fx0;
    float fy1 = fy0;
    int maxdisp = 0;
    cv::Matx33f P2(fx1, 0. , cx + maxdisp,
                   0. , fy1, cy,
                   0. , 0. , 1.);

        std::cout << "P1: " << P1 << std::endl;
        std::cout << "P2: " << P2 << std::endl;

    M1.at<double>(0,2) /=IMSCALEF; // cx
    M1.at<double>(1,2) /=IMSCALEF; // cy
    M1.at<double>(0,2) +=  ROIsizeH;
    M1.at<double>(1,2) +=  ROIsizeH;

    M2.at<double>(0,2) /=IMSCALEF;
    M2.at<double>(1,2) /=IMSCALEF;
    M2.at<double>(0,2) +=  ROIsizeH;
    M2.at<double>(1,2) +=  ROIsizeH;


    M1.at<double>(0,0) /=IMSCALEF; //fx
    M1.at<double>(1,1) /=IMSCALEF; //fy
    M2.at<double>(0,0) /=IMSCALEF;
    M2.at<double>(1,1) /=IMSCALEF;

    //the D matrices are scaleless

    Mat R, T;
    fs["R"] >> R;
    fs["T"] >> T;
    Mat R1fake, R2fake, P1fake, P2fake;    
    fisheye::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1fake, R2fake, P1fake, P2fake, Qf, CALIB_ZERO_DISPARITY);
    Qf.at<double>(3,2) = 5.0; // baseline is approx 0.2 m, so 1/baseline = 5

    fisheye::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    fisheye::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);    

    //SGBM = cv::StereoSGBM.create (5, 256, 5) ; // ,50,500,10,100,10,0,0,false);
    int nDisparity = 64;
    sgbm = cv::StereoSGBM(0, nDisparity, 1,
                          50,5000,10,
                          100,10,
                          000,000,
                          false);
    dispScale = 4096/(256/nDisparity);

    return 0;

}

void stereoAlg::rectify(cv::Mat frameL,cv::Mat frameR) {
    cv::Point pr2(frameL.cols/2-ROIsizeH, frameL.rows/2-ROIsizeH);
    cv::Point pr1(frameL.cols/2+ROIsizeH, frameL.rows/2+ROIsizeH);
   	cv::Mat roiL = cv::Mat(frameL, cv::Rect(pr1, pr2));
   	cv::Mat roiR = cv::Mat(frameR, cv::Rect(pr1, pr2));
    remap(roiL, frameLrect, map11, map12, INTER_LINEAR);
    remap(roiR, frameRrect, map21, map22, INTER_LINEAR);
}

void stereoAlg::calcDisparityMap() {

   Mat imgLr, imgRr;
   
   //combineImage(imgLrf,imgRrf);
   //imshow("FrameLR rectified fisheye", frameC);

   cv::Point size(96,96);
   cv::resize(frameLrect,imgLr,size);
   cv::resize(frameRrect,imgRr,size);

   //cv::imshow("t1" , imgLr);
   //cv::imshow("t2" , imgRr);

    frameD = cv::Mat::zeros(96,96, cv::DataType<uint16_t>::type);
    sgbm(imgLr, imgRr, frameD);

    //calc depth in mm from disparity
    // z = (b*F) / (d*s)
    // z = depth in mm
    // b = baseline in mm
    // F = focal length in mm
    // d = depth in pixel
    // s = sensor size in mm/pixel.
    //or
    // z = b*F / d
    // mm = mm * pixel / pixel
#ifdef blabla
    cv::Mat tmpD;
    frameD.convertTo(tmpD,CV_8UC1, 256.0/dispScale, 0.0); // expand range to 0..255.
    cv::Point size2(ROIsize,ROIsize);
    cv::resize(tmpD,tmpD,size2);
    cv::applyColorMap(tmpD, tmpD, cv::COLORMAP_JET);
    cv::imshow("Disparity Opencv" , tmpD);
#endif
}


