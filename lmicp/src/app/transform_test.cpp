#include <memory>
#include "ros/ros.h"

//#include "common/cxy_config.h"
#include "common/cxy_common.h"
#include "utility/cxy_transform.h"
//#include "kinematic/cxy_icp_kinematic.h"

void rotateXYZ(const float& x_r, const float& y_r, const float& z_r, Eigen::Matrix<float, 3, 3>& matrix);


int main(int argc, char const *argv[])
{
	//cxy::config("common/config");
    //std::shared_ptr<cxy::cxy_config> config_ptr = std::make_shared<cxy::cxy_config>("/home/xiongyi/cxy_workspace/src/cxyros/lmicp/include/common/config");
    //config_ptr->unserialize();

    //cxy::cxy_lmicp_lib::cxy_icp_kinematic kin(config_ptr);

    Eigen::Matrix<float, 3, 1> p1(1.0, 1.0, 0.0);
    Eigen::Matrix<float, 3, 1> p2(0.0, 0.0, 0.0);
    cxy::cxy_transform::Pose<float> pose;
    Eigen::Matrix<float, 3, 1> offset (0,2,0);
    pose.rotateByAxis(cxy::cxy_transform::Axis::Z_axis_rotation, Deg2Rad(90));
    //pose.rotateByAxis(cxy::cxy_transform::Axis::Y_axis_rotation, Deg2Rad(90));
    pose.t()(0) = 1;
    pose.t()(1) = 1;
    ROS_INFO_STREAM("p1 = "<<p1);
    ROS_INFO_STREAM("pose = "<<pose.q().w()<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z());
    pose.composePoint(p1, p2);
    ROS_INFO_STREAM("p2 = "<<p2(0)<<" "<<p2(1)<<" "<<p2(2));


    ROS_INFO_STREAM(" 90 x ");
    ROS_INFO_STREAM(" 90 x ");
    Eigen::Matrix<float, 3, 3> matrix;
    rotateXYZ(90, 90, 0, matrix);
    //matrix = matrix.inverse();

    p2 = matrix * p1;
    ROS_INFO_STREAM(matrix);

    ROS_INFO_STREAM("p2 = "<<p2(0)<<" "<<p2(1)<<" "<<p2(2));

    Eigen::Matrix<float, 3, 3> matrix2;
    ROS_INFO_STREAM(" 90 y ");
    ROS_INFO_STREAM(" 90 y ");
    rotateXYZ(0, 90, 0, matrix2);
    p2 = matrix2 * p1;
    ROS_INFO_STREAM(matrix2);

    ROS_INFO_STREAM("p2 = "<<p2(0)<<" "<<p2(1)<<" "<<p2(2));

    ROS_INFO_STREAM(" 90 x*y ");
    ROS_INFO_STREAM(" 90 x*y ");
    p2 = matrix*matrix2 * p1;
    ROS_INFO_STREAM("p2 = "<<p2(0)<<" "<<p2(1)<<" "<<p2(2));
    return 0;
}


void rotateXYZ(const float& x_r, const float& y_r, const float& z_r, Eigen::Matrix<float, 3, 3>& matrix)
{
    Eigen::Matrix<float, 3, 3> x_mat;
    x_mat<<1,   0,                      0,
            0, std::cos(Deg2Rad(x_r)), -std::sin(Deg2Rad(x_r)),
            0, std::sin(Deg2Rad(x_r)), std::cos(Deg2Rad(x_r));

    Eigen::Matrix<float, 3, 3> y_mat;
    y_mat<<std::cos(Deg2Rad(y_r)),  0, std::sin(Deg2Rad(y_r)),
            0,                      1, 0,
            -std::sin(Deg2Rad(y_r)), 0, std::cos(Deg2Rad(y_r));

    Eigen::Matrix<float, 3, 3> z_mat;
    z_mat<<std::cos(Deg2Rad(z_r)), -std::sin(Deg2Rad(z_r)), 0,
            std::sin(Deg2Rad(z_r)), std::cos(Deg2Rad(z_r)), 0,
            0,                      0,                      1;

    matrix =  x_mat * y_mat * z_mat;
    return;

}