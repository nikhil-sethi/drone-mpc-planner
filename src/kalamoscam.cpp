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
	frameL_mat = cv::Mat::zeros(480,640,CV_8UC3);
	frameR_mat = cv::Mat::zeros(480,640,CV_8UC3);
	frameD_mat = cv::Mat::zeros(96,96,CV_32F);		
}


void KalamosCam::CBdepth(DepthmapData const& data) {
	std::cout << "New depth!!!"  << std::endl;
	data.depth.copyTo(frameD_mat);

}

void KalamosCam::CBstereo(StereoYuvData const& data) {
	std::cout << "New im!!!"  << std::endl;
	g_lockWaitForImage1.lock();


	cv::Mat tmpLy;
	cv::resize(*(data.leftYuv[0]),tmpLy,frameL_mat.size(),0,0);

	cv::Mat tmpLv;
	cv::resize(*(data.leftYuv[1]),tmpLv,frameL_mat.size(),0,0);

	cv::Mat tmpLu;	
	cv::resize(*(data.leftYuv[2]),tmpLu,frameL_mat.size(),0,0);

	std::vector<cv::Mat> channelsL;
	channelsL.push_back(tmpLy);
	channelsL.push_back(tmpLu);
	channelsL.push_back(tmpLv);
	cv::Mat frameL_yuv;
	cv::merge(channelsL,frameL_yuv);
	cv::cvtColor(frameL_yuv,frameL_mat,CV_YUV2BGR);

	cv::Mat tmpRy;
	cv::resize(*(data.rightYuv[0]),tmpRy,frameR_mat.size(),0,0);

	cv::Mat tmpRv;
	cv::resize(*(data.rightYuv[1]),tmpRv,frameR_mat.size(),0,0);

	cv::Mat tmpRu;	
	cv::resize(*(data.rightYuv[2]),tmpRu,frameR_mat.size(),0,0);

	std::vector<cv::Mat> channelsR;
	channelsR.push_back(tmpRy);
	channelsR.push_back(tmpRu);
	channelsR.push_back(tmpRv);
	cv::Mat frameR_yuv;
	cv::merge(channelsR,frameR_yuv);
	cv::cvtColor(frameR_yuv,frameR_mat,CV_YUV2BGR);


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
