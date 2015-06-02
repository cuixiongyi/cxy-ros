#pragma once
#include "perception_model_based_detection/mbt_config.h"
namespace cxy_mbt 
{
    mbt_Config* mbt_Config::m_pInstance = nullptr;

    mbt_Config::~mbt_Config()
    {
        if (m_pInstance != nullptr)
            delete m_pInstance;
        return;
    }

mbt_Config::mbt_Config()  : 
            isnew_guesspose_(false)
            , cam_init_count_(0)
{
    mbt_frame_id_ = "tf_sentis_tof";
    
    newtime_guesspose_ = ros::Time::now();
    newtime_depth_ = ros::Time::now();
    newtime_visualservostate_ = ros::Time::now();

    bool isnew_guesspose_ = false;
    bool isnew_depth_ = false;
    bool isnew_visualservostate_ = false;
    
    //newdepth_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    int newvisualservostate_ = 0;

    std::string model_type_;
    float main_loop_rate_ = 1.0;
    transform_ = Eigen::Matrix4d::Identity();   


}
    void mbt_Config::setNodeHandle(ros::NodeHandle nh, ros::NodeHandle pnh)
    {
        
        if (m_pInstance == nullptr)
            m_pInstance = new mbt_Config;
        getInstance()->nh_ = nh;
        getInstance()->pnh_ = pnh;
        std::string model_type, action_name;
        pnh.param<std::string>("model_type", getInstance()->model_type_, "Valve");
        pnh.param<double>("main_loop_rate", getInstance()->main_loop_rate_, 15.0);
        pnh.param<std::string>("action_name", getInstance()->action_name_, "mbt_valve");
        
        return;
    }

   mbt_Config* mbt_Config::getInstance()
   {
        if (m_pInstance == nullptr)
            m_pInstance = new mbt_Config;
        return m_pInstance;
   }



bool mbt_Config::poseToMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d &matrix)
{
    Eigen::Quaternion<double> q(pose.pose.orientation.w, pose.pose.orientation.x,
                                pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Matrix3d m = q.toRotationMatrix();

    matrix << m(0,0), m(0,1), m(0,2), pose.pose.position.x,
              m(1,0), m(1,1), m(1,2), pose.pose.position.y,
              m(2,0), m(2,1), m(2,2), pose.pose.position.z,
              0,      0,      0,      1;

  return true;
}

bool mbt_Config::matrixToPose(std::string frame, const Eigen::Matrix4d& transform, geometry_msgs::PoseStamped &pose)
{
    Eigen::Matrix3d m;
    m << transform(0,0), transform(0,1), transform(0,2),
         transform(1,0), transform(1,1), transform(1,2),
         transform(2,0), transform(2,1), transform(2,2);

    // check orthonormal
    double det = (m(0,0)*m(1,1)*m(2,2)) + (m(0,1)*m(1,2)*m(2,0)) + (m(0,2)*m(1,0)*m(2,1)) -
                 (m(0,2)*m(1,1)*m(2,0)) - (m(0,1)*m(1,0)*m(2,2)) - (m(0,0)*m(1,2)*m(2,1));

    if ((det >= 1.25) || (det <= -1.25))
        return false;
    else
    {
        Eigen::Quaternion<double> q (m);
        //geometry_msgs::PoseStamped guess_pose;
        pose.header.frame_id = frame;
        pose.pose.position.x = transform(0,3);
        pose.pose.position.y = transform(1,3);
        pose.pose.position.z = transform(2,3);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        return true;
    }
}
bool mbt_Config::isSynchronized(ros::Time last, double tin)
{
    
    static const double update_rate(1/(mbt_Config::getInstance()->main_loop_rate_-7));
    double tolerance(tin);
    if (tolerance < 0)
        tolerance = update_rate;

    double dnow = ros::Time::now().toSec();
    double dlast = last.toSec();
    if (dnow -dlast > tolerance)
        return false;

    return true;
}





}
