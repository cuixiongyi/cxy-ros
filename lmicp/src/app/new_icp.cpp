#include <memory>
#include "ros/ros.h"

//#include "common/cxy_config.h"
#include "common/cxy_common.h"
#include "utility/cxy_transform.h"
#include "kinematic/cxy_icp_kinematic.h"



int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "my_node_name");
    ros::NodeHandle nh;

	//cxy::config("common/config");
    std::shared_ptr<const cxy::cxy_config> config_ptr = cxy::cxy_config::getConfig();
    cxy::cxy_config::unserialize();
    cxy::cxy_lmicp_lib::cxy_icp_kinematic<float> kin(config_ptr.get());
    cxy::cxy_lmicp_lib::cxy_icp_kinematic<float> kin_data(config_ptr.get());
    Eigen::Matrix< float, Eigen::Dynamic, 1> x;
    x.resize(config_ptr->joint_number_);
    kin_data.updateJointModel(x);

    return 0;
}

