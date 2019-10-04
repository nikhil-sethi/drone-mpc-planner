#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>

void CameraVolume::init(float a_top, float a_front, float a_left, float a_right,
                        float b_depth, float b_height){
    slope_top = a_top;
    slope_front = a_front;
    slope_left = a_left;
    slope_right = a_right;

    z_limit = b_depth;
    y_limit = b_height;
}


bool CameraVolume::in_view(cv::Point3f p, volume_check_mode c){
    if (c == relaxed)
        return in_view(p,0.3);
    else
        return in_view(p,0);
}
bool CameraVolume::in_view(cv::Point3f p,float hysteresis_margin){
    // Attention check the negative case!
    if( //p.y>slope_top*p.z - hysteresis_margin ||
         p.y<slope_front*p.z + hysteresis_margin
        || p.x>slope_left*p.z - hysteresis_margin
        || p.x<slope_right*p.z + hysteresis_margin
        || p.z<z_limit + hysteresis_margin
        || p.y<y_limit + hysteresis_margin)
        return false;
    else
        return true;
}

float CameraVolume::calc_distance_to_borders(track_data data_drone){
    std::vector<cv::Point3f> p{data_drone.pos (), data_drone.vel ()};
    return calc_distance_to_borders (p);
}

float CameraVolume::calc_distance_to_borders(std::vector<cv::Point3f> p){
    cv::Mat pMat = (cv::Mat_<float_t>(3,2) << p[0].x, p[1].x, p[0].y, p[1].y, p[0].z, p[1].z);

    float min_dist = std::numeric_limits<float>::max();
    float dist;

    // Check the back:
    cv::Mat plane = (cv::Mat_<float>(3,2) << 0.f, 0.f, 0.f, 0.f, z_limit, -1.f);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the ground:
    plane = (cv::Mat_<float>(3,2) << 0.f, 0.f, y_limit, 1, 0.f, 0.f);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the front:
    plane = (cv::Mat_<float>(3,2) << 0.f, 0.f, 0.f, -1, 0.f, slope_front);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the top:
    plane = (cv::Mat_<float>(3,2) << 0.f, 0.f, 0.f, -1, 0.f, slope_top);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the left:
    plane = (cv::Mat_<float>(3,2) << 0.f, -1.f, 0.f, 0.f, 0.f, slope_left);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the right:
    plane = (cv::Mat_<float>(3,2) << 0.f, -1.f, 0.f, 0.f, 0.f, slope_right);
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    return min_dist;
}

float CameraVolume::calc_distance_to_plane(cv::Mat vec, cv::Mat plane){
    cv::Mat b(3, 1, CV_32F);
    b = vec.col(0) - plane.col(0);

    cv::Mat n(3, 1, CV_32F);
    n = plane.col(1)/norm(plane.col(1));
    cv::Mat e1(3, 1, CV_32F);
    e1 = get_orthogonal_vector(n);
    cv::Mat e2(3, 1, CV_32F);
    e2 = n.cross(e1);

    cv::Mat A(3, 3, CV_32F);
    e1.copyTo (A.col(0));
    e2.copyTo (A.col(1));
    cv::Mat tmp = (cv::Mat_<float>(3,1) << -vec.at<float>(0,1), -vec.at<float>(1,1), -vec.at<float>(2,1));
    tmp.copyTo (A.col(2));

    if(abs(cv::determinant(A))<0.05){
        // the plane and the vector are (almost) parallel to each other.
        // distance is infinity
        return std::numeric_limits<float>::max();
    }

    cv::Mat params(3, 1, CV_32F);
    params = A.inv()*b;
    float sgn_param3 = 1 - 2*(params.at<float>(2,0)<0);

    return sgn_param3 * static_cast<float>(norm( params.at<float>(2,0)*vec.col(1) ));
}

cv::Mat CameraVolume::get_orthogonal_vector(cv::Mat vec){
    cv::Mat rt;
    if(vec.at<float>(2,0)!=0){
        rt = (cv::Mat_<float>(3,1) << 1,1,(-vec.at<float>(0,0) -vec.at<float>(1,0))/vec.at<float>(2,0) );
    }else if(vec.at<float>(1,0)!=0){
        rt = (cv::Mat_<float>(3,1) << 1,(-vec.at<float>(0,0) -vec.at<float>(2,0))/vec.at<float>(1,0),1 );
    }else if(vec.at<float>(0,0)!=0){
        rt = (cv::Mat_<float>(3,1) << (-vec.at<float>(1,0) -vec.at<float>(2,0))/vec.at<float>(0,0),1,1 );
    }

    return rt;
}

//strips disparity from world2im_3d
cv::Point2f world2im_2d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle){
    cv::Point3f p_im = world2im_3d(p_world,Qfi,camera_angle);
    return cv::Point2f(p_im.x,p_im.y);
}

//returns image coordinates from a world coordinate: x,y,disparity
cv::Point3f world2im_3d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle){
    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    //derotate camera and convert to double:
    cv::Point3d tmpd (p_world.x,p_world.y,p_world.z);
    float theta = -camera_angle * deg2rad;
    float temp_y = p_world.y * cosf(theta) + p_world.z * sinf(theta);
    tmpd.z = -p_world.y * sinf(theta) + p_world.z * cosf(theta);
    tmpd.y = temp_y;
    tmpd.x = p_world.x;

    world_coordinates.push_back(tmpd);
    cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);

    camera_coordinates.at(0).z = -camera_coordinates.at(0).z; // negate disparity
    return camera_coordinates.at(0);
}

//returns image coordinates from a world coordinate: x,y,disparity
cv::Point3f im2world(cv::Point2f p_im,float disparity, cv::Mat Qf, float camera_angle){
    std::vector<cv::Point3d> camera_coordinates, world_coordinates;
    camera_coordinates.push_back(cv::Point3d(p_im.x,p_im.y,-disparity));
    cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

    cv::Point3f w;
    w.x = world_coordinates[0].x;
    w.y = world_coordinates[0].y;
    w.z = world_coordinates[0].z;
    //compensate camera rotation:
    float theta = camera_angle * deg2rad;
    float temp_y = w.y * cosf(theta) + w.z * sinf(theta);
    w.z = -w.y * sinf(theta) + w.z * cosf(theta);
    w.y = temp_y;

    return w;
}

bool file_exist (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

//combines a sperate left and right image into one combined concenated image
void combine_image(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

    *res = cv::Mat(iml.rows,iml.cols + imr.cols,CV_8UC3);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols+imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

//combines a sperate left and right image into one combined concenated image
void combine_gray_image(cv::Mat iml,cv::Mat imr,cv::Mat *res) {

    *res = cv::Mat(iml.rows,iml.cols + imr.cols,CV_8UC1);
    cv::Point pl1(0, 0);
    cv::Point pl2(iml.cols, iml.rows);
    cv::Mat roil = cv::Mat(*res, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(iml.cols, 0);
    cv::Point pr2(iml.cols+imr.cols, imr.rows);
    cv::Mat roir = cv::Mat(*res, cv::Rect(pr1, pr2));
    imr.copyTo(roir);
}

cv::Mat create_row_image(std::vector<cv::Mat> ims, int type,float resizef)
{
    //find max height and total width:
    int width = 0;
    int height = -1;
    for (uint i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).rows > height)
        {
            height = ims.at(i).rows;
        }
        width += ims.at(i).cols;
    }

    cv::Mat res = cv::Mat(height*resizef, width*resizef, type);

    cv::Point p1(0, 0);

    for (uint i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(p1.x + im.cols*resizef, im.rows*resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));
        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, CV_BGR2GRAY);
            else
                cv::cvtColor(im, im, CV_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(roi.cols-1,0),cv::Point(roi.cols-1,roi.rows-1),cv::Scalar(128,128,128));

        p1.x += im.cols*resizef;
    }
    return res;
}

cv::Mat create_column_image(std::vector<cv::Mat> ims, int type,float resizef)
{
    //find max width and total height:
    int width = -1;
    int height = 0;
    for (uint i = 0; i < ims.size(); i++)
    {
        if (ims.at(i).cols > width)
        {
            width = ims.at(i).cols;
        }
        height += ims.at(i).rows;
    }

    cv::Mat res = cv::Mat::zeros(height*resizef, width*resizef, type);

    cv::Point p1(0, 0);

    for (uint i = 0; i < ims.size(); i++)
    {
        cv::Mat im = ims.at(i);
        cv::Point p2(im.cols*resizef, p1.y + im.rows*resizef);
        cv::Mat roi = cv::Mat(res, cv::Rect(p1, p2));

        if (im.type() != type)
        {
            if (type == CV_8UC1)
                cv::cvtColor(im, im, CV_BGR2GRAY);
            else
                cv::cvtColor(im, im, CV_GRAY2BGR);
        }

        cv::resize(im,roi,cv::Size(im.cols*resizef,im.rows*resizef));
        if (i<ims.size()-1)
            cv::line(roi,cv::Point(0,roi.rows-1),cv::Point(roi.cols,roi.rows-1),cv::Scalar(128,128,128));
        p1.y += im.rows*resizef;
    }
    return res;
}

/* combines a bunch of images into one column, and shows it */
void show_column_image(std::vector<cv::Mat> ims, std::string window_name, int type, float resizef) {
    cv::Mat res = create_column_image(ims,type,resizef);
    cv::imshow(window_name, res);
}

/* combines a bunch of images into one row, and shows it */
void show_row_image(std::vector<cv::Mat> ims, std::string window_name, int type,float resizef) {
    cv::Mat res = create_row_image(ims,type,resizef);
    cv::imshow(window_name, res);
}

std::string to_string_with_precision(float f, const int n)
{
    std::ostringstream out;

    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

int seconds_since_file_creation(std::string file_path)
{
    if (!file_exist(file_path))
        return std::numeric_limits<int>::max();
    struct stat attrib;
    stat(file_path.c_str(), &attrib);
    //    auto t1 = localtime(&(attrib.st_ctime));
    //    std::cout << std::asctime( localtime(&(attrib.st_ctime)));
    auto curtime=  std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(curtime);
    //    std::cout << std::asctime(std::localtime(&in_time_t)) << std::endl;
    //    auto now = std::localtime(&in_time_t);
    double diff = difftime(in_time_t,attrib.st_ctime);
    return static_cast<int>(diff);
}




