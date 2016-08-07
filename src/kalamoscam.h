


#ifndef KALAMOSCAM_H
#define KALAMOSCAM_H
#include "defines.h"
#if CAMMODE == CAM_KALAMOSCAM
#include "cam.h"
//#include <kalamos/onboard-sdk/StereoDepthGrabber.hpp>
#include <kalamos_context.hpp>

class KalamosCam : public Cam{


private:
    kalamos::Callbacks cbs;

    std::thread thread_workaround;
	void workaroundThread(void);

	void CBstereo(kalamos::StereoYuvData const& data);
	void CBdepth(kalamos::DepthmapData const& data);

public:
    cv::Mat frameD_mat;

    bool init (void);
    void close (void);

    void workerThread(void);
    cv::Mat get_raw_frame(void);
    cv::Mat get_disp_frame(void);

    KalamosCam() {
        isStereo = true;
    }



};
#endif
#endif //KALAMOSCAM_H
