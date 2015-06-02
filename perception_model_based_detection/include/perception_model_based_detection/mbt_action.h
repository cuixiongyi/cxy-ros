#pragma once
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "perception_msgs/visual_servo_LRAction.h"
#include <actionlib/server/simple_action_server.h>
#include "limits"
#include "perception_model_based_detection/mbt_config.h"

#include <string>

namespace cxy_mbt
{
class mbt_Action 
{
protected:

    perception_msgs::visual_servo_LRFeedback feedback_;
    perception_msgs::visual_servo_LRResult result_;
    ros::NodeHandle nh_;
    std::string action_name_;
public:

    mbt_Action(std::string);
    void executeCB(const perception_msgs::visual_servo_LRGoalConstPtr &goal);
    tf::TransformListener listener;
    
    actionlib::SimpleActionServer<perception_msgs::visual_servo_LRAction> as_;


};

}
