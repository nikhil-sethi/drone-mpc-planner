
#include "kalamoscam.h"
#ifndef _PC
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <kalamos_context.hpp>

using namespace kalamos;
using namespace std::placeholders;

bool KalamosCam::init (void) {
	frameL = cv::Mat::zeros(960,1280,CV_8UC3);
	frameR = cv::Mat::zeros(960,1280,CV_8UC3);
	frameD_mat = cv::Mat::zeros(96,96,CV_32F);		
}

void KalamosCam::CBdepth(DepthmapData const& data) {
	data.depth.copyTo(frameD_mat);
}

void KalamosCam::CBstereo(StereoYuvData const& data) {

	//prepare left frame:
	std::vector<cv::Mat> channelsLuv,channelsLyuv;
	channelsLuv.push_back(*(data.leftYuv[2]));
	channelsLuv.push_back(*(data.leftYuv[1]));
	cv::Mat frameLuv,frameLyuv;
	cv::merge(channelsLuv,frameLuv);
	cv::resize(frameLuv,frameLuv,frameL.size(),0,0);
	cv::Mat tmpLuv[2];
	cv::split(frameLuv,tmpLuv);
	channelsLyuv.push_back(*(data.leftYuv[0]));
	channelsLyuv.push_back(tmpLuv[0]);
	channelsLyuv.push_back(tmpLuv[1]);
	cv::merge(channelsLyuv,frameLyuv);

	//prepare right frame
	std::vector<cv::Mat> channelsRuv,channelsRyuv;
	channelsRuv.push_back(*(data.rightYuv[2]));
	channelsRuv.push_back(*(data.rightYuv[1]));
	cv::Mat frameRuv,frameRyuv;
	cv::merge(channelsRuv,frameRuv);
	cv::resize(frameRuv,frameRuv,frameR.size(),0,0);
	cv::Mat tmpRuv[2];
	cv::split(frameRuv,tmpRuv);
	channelsRyuv.push_back(*(data.rightYuv[0]));
	channelsRyuv.push_back(tmpRuv[0]);
	channelsRyuv.push_back(tmpRuv[1]);
	cv::merge(channelsRyuv,frameRyuv);

	//synchronously copy buffers:
	g_lockWaitForImage1.lock();
	cv::cvtColor(frameLyuv,frameL,CV_YUV2BGR);
	cv::cvtColor(frameRyuv,frameR,CV_YUV2BGR);
	g_lockWaitForImage2.unlock();
	
}


void KalamosCam::workerThread(void) {
	cbs.stereoYuvCallback = std::bind(&KalamosCam::CBstereo, this,_1);
	cbs.depthmapCallback = std::bind(&KalamosCam::CBdepth, this,_1);

	if (std::unique_ptr<kalamos::Context> kalamosContext = kalamos::init(cbs)) {	
	    std::unique_ptr<kalamos::ServiceHandle> captureHandle = kalamosContext->startService(ServiceType::CAPTURE);	
	    std::cout << "Kalamos init succes!" << std::endl;
	    kalamosContext->run();
  	}
}

cv::Mat KalamosCam::get_raw_frame() {
    return frameL;
}

cv::Mat KalamosCam::get_disp_frame() {
    cv::Mat frameDt = cv::Mat::zeros(frameD_mat.rows,frameD_mat.cols, CV_8UC1);
    frameD_mat.convertTo(frameDt,CV_8UC1, 2);
    return frameDt;
}

void KalamosCam::close (void) {
    Cam::close();
}


#endif
