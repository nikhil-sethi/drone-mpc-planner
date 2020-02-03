#include "common.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>


cv::Mat intersection_of_3_planes(cv::Mat p0_1, cv::Mat n_1, cv::Mat p0_2, cv::Mat n_2, cv::Mat p0_3, cv::Mat n_3) {
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

std::tuple<float, cv::Mat> hesse_normal_form(cv::Mat p0, cv::Mat n)
{
    cv::Mat n0;

    n0 = n.mul(cv::norm(n));
    double d = p0.dot(n0);

    return std::make_tuple(d, n0);
}

float calc_distance_to_plane(cv::Mat vec, cv::Mat plane) {
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

    if(abs(cv::determinant(A))<0.05) {
        // the plane and the vector are (almost) parallel to each other.
        // distance is infinity
        return std::numeric_limits<float>::max();
    }

    cv::Mat params(3, 1, CV_32F);
    params = A.inv()*b;
    float sgn_param3 = 1 - 2*(params.at<float>(2,0)<0);

    return sgn_param3 * static_cast<float>(norm( params.at<float>(2,0)*vec.col(1) ));
}

cv::Mat get_orthogonal_vector(cv::Mat vec) {
    cv::Mat rt;
    if(vec.at<float>(2,0)!=0) {
        rt = (cv::Mat_<float>(3,1) << 1,1,(-vec.at<float>(0,0) -vec.at<float>(1,0))/vec.at<float>(2,0) );
    } else if(vec.at<float>(1,0)!=0) {
        rt = (cv::Mat_<float>(3,1) << 1,(-vec.at<float>(0,0) -vec.at<float>(2,0))/vec.at<float>(1,0),1 );
    } else if(vec.at<float>(0,0)!=0) {
        rt = (cv::Mat_<float>(3,1) << (-vec.at<float>(1,0) -vec.at<float>(2,0))/vec.at<float>(0,0),1,1 );
    }

    return rt;
}

std::tuple<cv::Mat, cv::Mat> get_orthogonal_vectors(cv::Mat vec) {
    cv::Mat orth1 = get_orthogonal_vector(vec);
    cv::Mat orth2 = vec.cross(orth1);

    return std::tuple(orth1, orth2);
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> split_vector_to_basis_vectors(cv::Mat vec, cv::Mat b1, cv::Mat b2, cv::Mat b3) {
    cv::Mat A(3, 3, CV_32F);
    b1.copyTo(A.col(0));
    b2.copyTo(A.col(1));
    b3.copyTo(A.col(2));

    cv::Mat params(3, 1, CV_32F);
    params = A.inv()*vec;

    return std::tuple(params.at<float>(0,0)*b1,
                      params.at<float>(1,0)*b2,
                      params.at<float>(2,0)*b3);
}

cv::Mat get_plane_normal_vector(cv::Point3f x1, cv::Point3f x2) {
    cv::Point3f n = x1.cross (x2);
    return cv::Mat(n)/norm(n);
}

bool on_normal_side(cv::Mat p0, cv::Mat n, cv::Mat p) {
    cv::Mat v = cv::Mat::zeros(cv::Size(1,3), CV_32F);
    v = p-p0;
    if(v.dot (n)>=0)
        return true;
    else
        return false;
}



//strips disparity from world2im_3d
cv::Point2f world2im_2d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle) {
    cv::Point3f p_im = world2im_3d(p_world,Qfi,camera_angle);
    return cv::Point2f(p_im.x,p_im.y);
}

//returns image coordinates from a world coordinate: x,y,disparity
cv::Point3f world2im_3d(cv::Point3f p_world, cv::Mat Qfi, float camera_angle) {
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
cv::Point3f im2world(cv::Point2f p_im,float disparity, cv::Mat Qf, float camera_angle) {
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

float normf(cv::Point2f m) { return static_cast<float>(cv::norm(m));}
float normf(cv::Point3f m) { return static_cast<float>(cv::norm(m));}

cv::Point3f multf(cv::Point3f  p1, cv::Point3f p2) {
    cv::Point3f p;
    p.x = p1.x * p2.x;
    p.y = p1.y * p2.y;
    p.z = p1.z * p2.z;
    return p;
}

cv::Point3f deadzone(cv::Point3f p,float lo, float hi) {
    return cv::Point3f(deadzone(p.x,lo,hi),deadzone(p.y,lo,hi),deadzone(p.z,lo,hi));
}
float deadzone( float v, float lo, float hi ) {
    if (v < 0 && v > lo )
        v = 0;
    else if (v > 0 && v < hi )
        v = 0;
    return v;
}

float angle_to_horizontal(cv::Point3f direction) {
    //https://onlinemschool.com/math/library/analytic_geometry/plane_line/
    float A = 0;
    float B = 1;
    float C = 0;

    return asinf(abs(A*direction.x + B*direction.y + C*direction.z) /normf({A, B, C}) /normf(direction));
}

cv::Point3f lowest_direction_to_horizontal(cv::Point3f direction, float min_angle) {
    if(angle_to_horizontal(direction)<min_angle){
        direction.y = tan(min_angle) / normf({direction.x, direction.z});
    }
    direction /= norm(direction);
    return direction;
}