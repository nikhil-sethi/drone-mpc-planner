#include "linalg.h"

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

float distance_to_plane(cv::Mat p0, cv::Mat n, cv::Mat p) {
    cv::Mat v = cv::Mat::zeros(cv::Size(1,3), CV_32F);
    v = p-p0;
    //WARNING: Only works if norm(n)==1!
    return v.dot (n);
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