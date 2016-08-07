#include "kalamoscam.h"
#if CAMMODE == CAM_KALAMOSCAM

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <kalamos_context.hpp>

using namespace kalamos;
using namespace std::placeholders;

bool KalamosCam::init (void) {
	frameL_mat = cv::Mat::zeros(960,1280,CV_8UC3);
	frameD_mat = cv::Mat::zeros(96,96,CV_32F);		
}


void KalamosCam::CBdepth(DepthmapData const& data) {
	std::cout << "New depth!!!"  << std::endl;
	data.depth.copyTo(frameD_mat);

}

void KalamosCam::CBstereo(StereoYuvData const& data) {
	std::cout << "New im!!!"  << std::endl;
	g_lockWaitForImage1.lock();

	cv::Mat tmpf1;
	cv::resize(*(data.leftYuv[1]),tmpf1,frameL_mat.size(),0,0);

	cv::Mat tmpf2;	
	cv::resize(*(data.leftYuv[2]),tmpf2,frameL_mat.size(),0,0);

	std::vector<cv::Mat> channels;
	channels.push_back(*(data.leftYuv[0]));
	channels.push_back(tmpf2);
	channels.push_back(tmpf1);
	cv::Mat frameL_yuv;
	cv::merge(channels,frameL_yuv);
	cv::cvtColor(frameL_yuv,frameL_mat,CV_YUV2BGR);
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
    return frameL_mat;
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
