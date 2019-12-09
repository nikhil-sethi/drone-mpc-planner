#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>


template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void CameraVolume::init(cv::Point3f point_left_top, cv::Point3f point_right_top, cv::Point3f point_left_bottom, cv::Point3f point_right_bottom,
                        float b_depth, float b_height){

    _n_front = get_plane_normal_vector (point_left_bottom, point_right_bottom);
    _p0_front = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_top = get_plane_normal_vector (point_left_top, point_right_top);
    _n_top *= -1; // let normal vector look inside the volume
    _p0_top = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_left = get_plane_normal_vector (point_left_bottom, point_left_top);
    _n_left *= -1; // let normal vector look inside the volume
    _p0_left = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_right = get_plane_normal_vector (point_right_bottom, point_right_top);
    _p0_right = (cv::Mat_<float>(3,1) << 0, 0, 0);

    _n_bottom = (cv::Mat_<float>(3,1) << 0, 1, 0);
    _p0_bottom = (cv::Mat_<float>(3,1) << 0, b_height, 0);

    _n_back = (cv::Mat_<float>(3,1) << 0, 0, 1);
    _p0_back = (cv::Mat_<float>(3,1) << 0, 0, b_depth);

#if false
    std::cout << "front> p0_front:" << p0_front.t() << " n_front: " << n_front.t() << std::endl;
    std::cout << "top> p0_top:" << p0_top.t() << " n_top: " << n_top.t() << std::endl;
    std::cout << "left> p0_left:" << p0_left.t() << " n_left: " << n_left.t() << std::endl;
    std::cout << "right> p0_right:" << p0_right.t() << " n_right: " << n_right.t() << std::endl;
    std::cout << "bottom> p0_bottom:" << p0_bottom.t() << " n_bottom: " << n_bottom.t() << std::endl;
    std::cout << "back> p0_back:" << p0_back.t() << " n_back: " << n_back.t() << std::endl;
#endif

    double margin_top = 0.2;
    double margin_bottom = 0.2;
    double margin_front = 1.5;
    double margin_back = 0.2;
    double margin_left = 7.0;
    double margin_right = 7.0;

    _n_bottom_hunt = _n_bottom;
    _n_top_hunt = _n_top;
    _n_front_hunt = (cv::Mat_<float>(3,1) << 0, 0, -1);
    _n_back_hunt = _n_back;

    point_left_bottom.x -= static_cast<float>(margin_left);
    point_left_top.x -= static_cast<float>(margin_left);
    point_right_bottom.x += static_cast<float>(margin_right);
    point_right_top.x += static_cast<float>(margin_right);

    _n_left_hunt = get_plane_normal_vector (point_left_bottom, point_left_top);
    _n_left_hunt *= -1;
    _n_right_hunt = get_plane_normal_vector (point_right_bottom, point_right_top);

    _p0_top_hunt = _p0_top + margin_top * _n_top_hunt;
    _p0_bottom_hunt = _p0_bottom + margin_bottom * _n_bottom_hunt;
    _p0_front_hunt =  margin_front * _n_front_hunt;
    _p0_back_hunt = _p0_back + margin_back * _n_back_hunt;
    _p0_left_hunt = _p0_left;
    _p0_right_hunt = _p0_right;

    calc_corner_points_hunt(_p0_front_hunt, _n_front_hunt, _p0_back_hunt, _n_back_hunt, _p0_top_hunt, _n_top_hunt, _p0_bottom_hunt, _n_bottom_hunt, _p0_left_hunt, _n_left_hunt, _p0_right_hunt, _n_right_hunt);
    calc_corner_points(_p0_front, _n_front, _p0_back, _n_back, _p0_top, _n_top, _p0_bottom, _n_bottom, _p0_left, _n_left, _p0_right, _n_right);
}


bool CameraVolume::in_view(cv::Point3f p, view_volume_check_mode c){
    if (c == relaxed)
        return in_view(p,0.3f);
    else
        return in_view(p,0.6f);
}

void CameraVolume::calc_corner_points(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                      cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                      cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right)
{
    _bottom_left_back = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    _bottom_right_back = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    _top_left_back = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    _top_right_back = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    _bottom_left_front = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    _bottom_right_front = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    _top_left_front = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    _top_right_front = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

void CameraVolume::calc_corner_points_hunt(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                           cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                           cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right)
{
    _bottom_left_back_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_back, n_back);
    _bottom_right_back_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_back, n_back);
    _top_left_back_hunt = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_back, n_back);
    _top_right_back_hunt = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_back, n_back);

    _bottom_left_front_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_left, n_left, p0_front, n_front);
    _bottom_right_front_hunt = intersection_of_3_planes(p0_bottom, n_bottom, p0_right, n_right, p0_front, n_front);
    _top_left_front_hunt = intersection_of_3_planes(p0_top, n_top, p0_left, n_left, p0_front, n_front);
    _top_right_front_hunt = intersection_of_3_planes(p0_top, n_top, p0_right, n_right, p0_front, n_front);
}

cv::Mat CameraVolume::intersection_of_3_planes(cv::Mat p0_1, cv::Mat n_1, cv::Mat p0_2, cv::Mat n_2, cv::Mat p0_3, cv::Mat n_3)
{
    cv::Mat n01, n02, n03;
    float d1, d2, d3;

    std::tie(d1, n01) = hesse_normal_form(p0_1, n_1);
    std::tie(d2, n02) = hesse_normal_form(p0_2, n_2);
    std::tie(d3, n03) = hesse_normal_form(p0_3, n_3);

    cv::Mat b, A;

    b = (cv::Mat_<float>(3,1) << d1, d2, d3);

    cv::hconcat(n01, n02, A);
    cv::hconcat(A, n03, A);

    return  A.t().inv() * (b.reshape(1,3));
}

std::tuple<float, cv::Mat> CameraVolume::hesse_normal_form(cv::Mat p0, cv::Mat n)
{
    cv::Mat n0;

    n0 = n.mul(cv::norm(n));
    double d = p0.dot(n0);

    return std::make_tuple(d, n0);
}

bool CameraVolume::in_view(cv::Point3f p,float hysteresis_margin){
    bool front_check = on_normal_side (_p0_front+hysteresis_margin*_n_front, _n_front, cv::Mat(p));
    bool left_check = on_normal_side (_p0_left-hysteresis_margin*_n_left, _n_left, cv::Mat(p));
    bool right_check = on_normal_side (_p0_right+hysteresis_margin*_n_right, _n_right, cv::Mat(p));
    bool bottom_check = on_normal_side (_p0_bottom+hysteresis_margin*_n_bottom, _n_bottom, cv::Mat(p));
    bool back_check = on_normal_side (_p0_back+hysteresis_margin*_n_back, _n_back, cv::Mat(p));

    // Attention check the negative case!
    if( // !top_check ||
            !front_check
            || !left_check
            || !right_check
            || !bottom_check
            || !back_check)
        return false;
    else
        return true;
}

CameraVolume::hunt_check_result CameraVolume::in_hunt_area(cv::Point3f d[[maybe_unused]], cv::Point3f m){
    bool front_check = on_normal_side (_p0_front_hunt, _n_front_hunt, cv::Mat(m));
    bool left_check = on_normal_side (_p0_left_hunt, _n_left_hunt, cv::Mat(m));
    bool right_check = on_normal_side (_p0_right_hunt, _n_right_hunt, cv::Mat(m));
    bool bottom_check = on_normal_side (_p0_bottom_hunt, _n_bottom_hunt, cv::Mat(m));
    bool back_check = on_normal_side (_p0_back_hunt, _n_back_hunt, cv::Mat(m));
    bool top_check = on_normal_side (_p0_top_hunt, _n_top_hunt, cv::Mat(m));

    if( !top_check)
        return HuntVolume_To_High;
    if( !bottom_check)
        return HuntVolume_To_Low;
    if( !front_check)
        return HuntVolume_Outside_Cone;
    if( !back_check)
        return HuntVolume_Outside_Cone;
    if( !left_check)
        return HuntVolume_Outside_Cone;
    if( !right_check)
        return HuntVolume_Outside_Cone;

    return HuntVolume_OK;
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
    cv::Mat plane = cv::Mat::zeros(cv::Size(2,3), CV_32F);
    _p0_back.copyTo (plane.col(0));
    _n_back.copyTo (plane.col(1));
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the bottom:
    _p0_bottom.copyTo (plane.col(0));
    _n_bottom.copyTo (plane.col(1));
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the front:
    _p0_front.copyTo (plane.col(0));
    _n_front.copyTo (plane.col(1));
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the top:
    _p0_top.copyTo (plane.col(0));
    _n_top.copyTo (plane.col(1));
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the left:
    _p0_left.copyTo (plane.col(0));
    _n_left.copyTo (plane.col(1));
    dist = calc_distance_to_plane (pMat, plane);

    if(dist>0 && dist<min_dist)
        min_dist = dist;

    // Check the right:
    _p0_right.copyTo (plane.col(0));
    _n_right.copyTo (plane.col(1));
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

cv::Mat CameraVolume::get_plane_normal_vector(cv::Point3f x1, cv::Point3f x2){
    cv::Point3f n = x1.cross (x2);
    return cv::Mat(n)/norm(n);
}

bool CameraVolume::on_normal_side(cv::Mat p0, cv::Mat n, cv::Mat p){
    cv::Mat v = cv::Mat::zeros(cv::Size(1,3), CV_32F);
    v = p-p0;
    if(v.dot (n)>=0)
        return true;
    else
        return false;
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

std::string to_string_with_precision(double f, const int n)
{
    std::ostringstream out;

    out << std::fixed << std::setprecision(n) << f;
    return out.str();
}

float normf(cv::Point3f m) { return static_cast<float>(cv::norm(m));}

cv::Point3f mult(cv::Point3f  p1, cv::Point3f p2){
    cv::Point3f p;
    p.x = p1.x * p2.x;
    p.y = p1.y * p2.y;
    p.z = p1.z * p2.z;
    return p;
}

cv::Point3f deadzone(cv::Point3f p,float lo, float hi){
    return cv::Point3f(deadzone(p.x,lo,hi),deadzone(p.y,lo,hi),deadzone(p.z,lo,hi));
}
float deadzone( float v, float lo, float hi ) {
    if (v < 0 && v > lo )
        v = 0;
    else if (v > 0 && v < hi )
        v = 0;
    return v;
}
