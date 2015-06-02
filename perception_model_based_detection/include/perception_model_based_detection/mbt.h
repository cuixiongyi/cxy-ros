#pragma once
#include "perception_model_based_detection/mbt_config.h"
#include "perception_model_based_detection/test_mbd.h"
#include "visualization_msgs/Marker.h"
#include "perception_model_based_detection/mbt_callback.h"
#include "perception_model_based_detection/model_based_detection.h"
#include "perception_model_based_detection/mbt_config.h"
#include "perception_model_based_detection/mbt_action.h"
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

#include "limits"
#include "boost/lexical_cast.hpp"
#include "visualization_msgs/MarkerArray.h"

#include <string>

namespace cxy_mbt
{
class mbt
{
    public:
                
        Eigen::Matrix4d detect(pcl::PointCloud<pcl::PointXYZ>::Ptr rawInput);
        void start();
        bool checkPointCloudUpdate();
        bool updatePoseWithTransform(geometry_msgs::PoseStamped &pose, Eigen::Matrix4d transform);

        bool parseGoal(const int goal_grab);
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

    mbt();

    private:
    VisualServo_Callback callback_;
    mbt_Action action_;
    
    boost::shared_ptr<perception_model_based_detection::ModelBasedDetection> model_based_detector_;
    perception_model_based_detection::Model model_;
    ros::Publisher                          model_desired_pose_publisher_;
    ros::Publisher                          pub_stl_marker_;
    ros::Subscriber                         sub_visualServoState_;
    ros::Subscriber sub_cam_;
  	ros::Subscriber sub_guesspose_;
    tf::Transform                                  tftransform_;
    tf::TransformListener                   listener_;



};

}





