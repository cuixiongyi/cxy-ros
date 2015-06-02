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

#include "limits"
#include "boost/lexical_cast.hpp"
#include "visualization_msgs/MarkerArray.h"

#include <string>
#include "perception_model_based_detection/mbt_config.h"
namespace cxy_mbt 
{
typedef pcl::PointXYZ  PointT;

class VisualServo_Callback
{
private:
public:
    VisualServo_Callback();
    ~VisualServo_Callback();
    void callback_GuessPose(const geometry_msgs::PoseStamped::ConstPtr pose);
    void callback_DepthCamera(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);
    void callback_VisualServoState(const std_msgs::Int16::ConstPtr& state);
    void publishTFFrame(std::string name);
    bool convertPointCloudTypes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in);

    tf::TransformListener                   listener;
    tf::TransformBroadcaster                br_;

    ros::Publisher pub_depth;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    const int camerInitFrameCount_Max;
};

}
