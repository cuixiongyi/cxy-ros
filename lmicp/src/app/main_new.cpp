#include <memory>
#include "ros/ros.h"

//#include "common/cxy_config.h"
#include "common/cxy_common.h"
#include "utility/cxy_transform.h"
//#include "kinematic/cxy_icp_kinematic.h"


int main(int argc, char const *argv[])
{
	//cxy::config("common/config");
    //std::shared_ptr<cxy::cxy_config> config_ptr = std::make_shared<cxy::cxy_config>("/home/xiongyi/cxy_workspace/src/cxyros/lmicp/include/common/config");
    //config_ptr->unserialize();

    //cxy::cxy_lmicp_lib::cxy_icp_kinematic kin(config_ptr);

    Eigen::Matrix<float, 3, 1> p1(1.0, 1.0, 1.0);
    Eigen::Matrix<float, 3, 1> p2(0.0, 0.0, 0.0);
    cxy::cxy_transform::Pose<float> pose;
    pose.rotateByAxis(cxy::cxy_transform::Axis::X_axis_rotation, Deg2Rad(90));

    ROS_INFO_STREAM("p1 = "<<p1);
    pose.composePoint(p1, p2);
    ROS_INFO_STREAM("p2 = "<<p2(0)<<" "<<p2(1)<<" "<<p2(2));

	return 0;
}