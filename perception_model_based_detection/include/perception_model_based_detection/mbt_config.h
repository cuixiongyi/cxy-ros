#pragma once
#include <string>
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
#include <cstddef>
#include "perception_msgs/visual_servo_LRAction.h"
namespace cxy_mbt 
{
class mbt_Config
{
private:


    mbt_Config(); 

    mbt_Config(mbt_Config const&)  = delete;
    void operator=(mbt_Config&)  = delete;
    
    static mbt_Config* m_pInstance;
    
public:
    ~mbt_Config();
    static mbt_Config* getInstance();
    static bool poseToMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d &matrix);
    static bool matrixToPose(std::string frame, const Eigen::Matrix4d& transform, geometry_msgs::PoseStamped &pose);
    static bool isSynchronized(ros::Time last, double tin = -10);
    static void setNodeHandle(ros::NodeHandle nh, ros::NodeHandle pnh); 
    

    ros::Time newtime_guesspose_;
    ros::Time newtime_depth_;
    ros::Time newtime_visualservostate_;

    bool isnew_guesspose_;
    bool isnew_depth_;
    bool isnew_visualservostate_;
    
    geometry_msgs::PoseStamped newpose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newdepth_;
    int newvisualservostate_;
    int cam_init_count_;

    ros::NodeHandle                     nh_, pnh_;


    perception_msgs::visual_servo_LRGoal action_goal_;

    std::string action_name_;
    std::string model_type_;
    std::string mbt_frame_id_;
    double main_loop_rate_;

    Eigen::Matrix4d transform_;
    ros::Time newtime_transform_;




};
}
