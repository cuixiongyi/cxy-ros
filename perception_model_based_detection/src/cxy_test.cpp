#include "visualization_msgs/Marker.h"
#include "perception_model_based_detection/mbt_callback.h"
#include "perception_model_based_detection/mbt_config.h"
#include "perception_model_based_detection/mbt_action.h"
#include "perception_model_based_detection/mbt.h"
#include <ros/callback_queue.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cxy_test_mbd");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    cxy_mbt::mbt_Config::getInstance()->setNodeHandle(nh, pnh);
   
    //pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloud;
    //mbt_Action action;
    /// loop at 15 Hz
    //ros::Rate rate((double)cxy_mbt::mbt_Config::getInstance()->main_loop_rate_);
    //Eigen::Matrix4d lastTransform = Eigen::Matrix4d::Identity() ;
    cxy_mbt::mbt mbt1;
    mbt1.start();
    return 0;
}

