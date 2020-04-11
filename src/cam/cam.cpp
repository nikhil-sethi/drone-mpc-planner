#include "cam.h"
#include <librealsense2/rsutil.h>

void Cam::set_file_paths(std::string replay_dir) {
    //make sure the origirnam files are not overwritten when playing bags:
    calib_wfn = data_output_dir + calib_rfn;
    depth_map_wfn = data_output_dir + depth_map_rfn;
    depth_unfiltered_map_wfn = data_output_dir + depth_unfiltered_map_rfn;
    disparity_map_wfn = data_output_dir + disparity_map_rfn;
    brightness_map_wfn = data_output_dir + brightness_map_rfn;

    calib_rfn = replay_dir + '/' + calib_rfn;
    depth_map_rfn = replay_dir + '/' + depth_map_rfn;
    depth_unfiltered_map_rfn = replay_dir + '/' + depth_unfiltered_map_rfn;
    disparity_map_rfn = replay_dir + '/' + disparity_map_rfn;
    brightness_map_rfn = replay_dir + '/' + brightness_map_rfn;
}


//Converting the raw depths of depth_background distances to
//world coordinates in the IR camera frame
//There's probably a way to do this more efficient...
void Cam::convert_depth_background_to_world() {
    depth_background_3mm = cv::Mat::zeros(depth_background.rows,depth_background.cols,CV_32FC3);
    depth_background_3mm_world = cv::Mat::zeros(depth_background.rows,depth_background.cols,CV_32FC3);
    depth_background_mm = cv::Mat::zeros(depth_background.rows,depth_background.cols,CV_32FC1);
    for (int i = 0; i < depth_background.cols; i++)
        for (int j = 0; j < depth_background.rows; j++) {
            uint16_t back = depth_background.at<uint16_t>(j,i);
            float backf = static_cast<float>(back) * camparams.depth_scale;
            float pixel[2];
            pixel[0] = i;
            pixel[1] = j;
            float p[3];
            rs2_deproject_pixel_to_point(p, intr, pixel, backf);
            cv::Vec3f pixelColor(p[0],p[1],p[2]);
            //            std::cout << "depth_background_3mm( " << i << " , " << j << " ): p[0]: " << p[0] << " ,p[1]: " << p[1] << " ,p[2]: " << p[2] <<std::endl;
            depth_background_3mm.at<cv::Vec3f>(j,i) = pixelColor;
            cv::Vec3f pixelWorldPos(p[0],
                                    p[1]*cosf(camparams.camera_angle_y*deg2rad) + p[2]*-sinf(-camparams.camera_angle_y*deg2rad),
                                    p[1]*sinf(-camparams.camera_angle_y*deg2rad) + p[2]*cosf(camparams.camera_angle_y*deg2rad));
            depth_background_3mm_world.at<cv::Vec3f>(j,i) = pixelWorldPos;
            depth_background_mm.at<float>(j,i) = sqrtf(powf(p[0],2)+powf(p[1],2)+powf(p[2],2));
        }
}

CameraView Cam::def_volume () {

    float b_depth, b_ground;

    cv::Point3f point_left_top, point_right_top, point_left_bottom, point_right_bottom;

    // Some planes are defined with the slopes of the corner pixels:
    // In the past this was more precice then the difintion with background pixels.
    point_left_top = get_SlopesOfPixel(0,0); // ..get_SlopesOfPixel(width, height)
    point_right_top = get_SlopesOfPixel(847, 0);
    point_left_bottom = get_SlopesOfPixel(0, 479);
    point_right_bottom = get_SlopesOfPixel(847, 479);

    // For the groundplane the offset in y direction is directly calculated;
    float y_sum = 0;
    uint n=0;
    for(uint row=300; row<480; row+=5) {
        for(uint col=212; col<636; col+=5) {

            if(depth_background_3mm_world.at<cv::Vec3f>(row,col)[1]!=0) {
                y_sum += depth_background_3mm_world.at<cv::Vec3f>(row,col)[1];
                n+=1;
            }
        }
    }
    b_ground = -y_sum/n;

    float z_sum = 0;
    n=0;
    for(uint row=0; row<10; row+=1) {
        for(uint col=0; col<848; col+=5) {

            if(depth_background_3mm_world.at<cv::Vec3f>(row, col)[2]!=0) {
                z_sum += depth_background_3mm_world.at<cv::Vec3f>(row, col)[2];
                n+=1;
            }
        }
    }
    b_depth = -z_sum/n;

    CameraView camview;
    camview.init(point_left_top, point_right_top, point_left_bottom, point_right_bottom, b_depth, b_ground, camparams.camera_angle_y);

    return camview;
}

cv::Point3f Cam::get_SlopesOfPixel(uint x, uint y) {
    std::vector<cv::Point3d> camera_coordinates, world_coordinates;
    camera_coordinates.push_back(cv::Point3d(x,y,-1));
    cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

    cv::Point3f w;
    w.x = world_coordinates[0].x;
    w.y = world_coordinates[0].y;
    w.z = world_coordinates[0].z;
    //compensate camera rotation:
    float theta = camparams.camera_angle_y * deg2rad;
    float temp_y = w.y * cosf(theta) + w.z * sinf(theta);
    w.z = -w.y * sinf(theta) + w.z * cosf(theta);
    w.y = temp_y;

    return w;
}
