#include "kalamoscam.h"
#if CAMMODE == CAM_KALAMOSCAM

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <kalamos_context.hpp>

using namespace kalamos;
using namespace std::placeholders;

	static cv::Mat frameL_rgb;
	static cv::Mat frameR_rgb;
	static cv::Mat frameD;
	static float acc[3];	
	static float gyro[3];
	static float mag[3];
	static float baro;
	static float sonar;
	
	static bool newImL = true;
	static bool newImD = true;
	
	static bool newImu = true;
	static bool newSonar = true;
	static bool newBaro = true;
	static bool newMag = true;

	static float lastUtraSoundDistance;
	static float lastBarometerPressure;

    static std::mutex g_lockWaitForImageL;
    static std::mutex g_lockWaitForImageR;
    static std::mutex g_lockWaitForImageD;


void leftFrame_callback(RgbData const& data, void* userData) {
	if (newImL) {
		data.rgb.copyTo(frameL_rgb);
		newImL = false;
	}
	g_lockWaitForImageL.unlock();
}

void depthMap_callback(DepthmapData const& data, void* userData) {
	if (newImD) {
		data.depth.copyTo(frameD);
		newImD = false;
	}
	g_lockWaitForImageD.unlock();
}

void ultraSound_callback(UltrasoundData const& data, void* userData) {
	if(newSonar) {
		sonar = data.distance;
		newSonar = false;
	}
	//std::cout << "US distance: " << data.distance << std::endl;
}
void barometer_callback(BarometerData const& data, void* userData) {	
	if(newBaro) {
		baro = data.pressure;
		newBaro = false;
	}
	//std::cout << "Baro pressure: " << data.pressure << std::endl;
}

void imu_callback(ImuData const& data, void* userData) {
	if(newImu) {
		acc[0]  = data.accel[0];
		acc[1]  = data.accel[1];
		acc[2]  = data.accel[2];
		gyro[0]  = data.gyro[0];
		gyro[1]  = data.gyro[1];
		gyro[2]  = data.gyro[2];
		newImu = false;
	}
//	std::cout << "Imu acc: " << data.accel[0] << "  " << data.accel[1] << " " << data.accel[2] << std::endl;
//	std::cout << "Imu gyro: " << data.gyro[0] << "  " << data.gyro[1] << " " << data.gyro[2] << std::endl;
}

void mag_callback(MagnetometerData const& data, void* userData) {
		if(newMag) {
		mag[0]  = data.field[0];
		mag[1]  = data.field[1];
		mag[2]  = data.field[2];
		newMag = false;
	}
	//std::cout << "Imu mag: " << data.field[0] << "  " << data.field[1] << " " << data.field[2] << std::endl;
}


bool KalamosCam::init (void) {

	thread_workaround = std::thread(&KalamosCam::workaroundThread,this);

    fps = VIDEOFPS;
}

void KalamosCam::workaroundThread(void) {
  	cbs.leftRgbCallback = leftFrame_callback;
  	cbs.depthmapCallback = depthMap_callback;
	cbs.ultrasoundCallback = ultraSound_callback;
	cbs.barometerCallback = barometer_callback;
	cbs.imuCallback = imu_callback;
	cbs.magnetometerCallback = mag_callback;
	//cbs.period = 1000;
  	//cbs.periodicCallback = [](void *) { std::cout << "periodic" << std::endl; };

	if (std::unique_ptr<kalamos::Context> kalamosContext = kalamos::init(cbs)) {
		std::unique_ptr<kalamos::ServiceHandle> captureHandle = kalamosContext->startService(ServiceType::CAPTURE);	
	    kalamosContext->run();
  	}
    std::cout << "Kalamos init succes!" << std::endl;

}

void KalamosCam::workerThread(void) {
		g_lockWaitForImageL.lock();
		g_lockWaitForImageD.lock();
	while (camRunning) {
		g_lockWaitForImageL.lock();
		//g_lockWaitForImageR.lock();
		g_lockWaitForImageD.lock();

		g_lockWaitForImage1.lock();
		currentFrame++;
		cv::cvtColor(frameL_rgb,frameL_mat,CV_RGB2BGR);
		
		im_height = frameL_mat.rows;
		im_width = frameL_mat.cols;	
		frameD.copyTo(frameD_mat);

		lastsonar = sonar;		
		lastbaro = baro;		
		lastacc[0]  = acc[0];
		lastacc[1]  = acc[1];
		lastacc[2]  = acc[2];
		lastgyro[0]  = gyro[0];
		lastgyro[1]  = gyro[1];
		lastgyro[2]  = gyro[2];		
		lastmag[0]  = mag[0];
		lastmag[1]  = mag[1];
		lastmag[2]  = mag[2];
		
		newImL = true;
		newImD = true;
		newImu = true;
		newBaro = true;
		newSonar = true;
		newMag = true;
		
		g_lockWaitForImage2.unlock();
	}

}

cv::Mat KalamosCam::get_raw_frame() {
    return frameL_mat;
}

cv::Mat KalamosCam::get_disp_frame() {
    cv::Mat frameDt = cv::Mat::zeros(frameD_mat.rows,frameD_mat.cols, CV_8UC1);
    frameD.convertTo(frameDt,CV_8UC1, 4);
    return frameDt;
}

void KalamosCam::close (void) {
    Cam::close();
	//thread_workaround.join();
}


#endif
