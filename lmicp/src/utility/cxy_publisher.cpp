#include "utility/cxy_publisher.h"

std::atomic<cxy::cxy_publisher*> cxy::cxy_publisher::m_instance;
std::mutex cxy::cxy_publisher::m_mutex;



namespace cxy
{
    cxy_publisher::cxy_publisher()
    {
        nh_ = ros::NodeHandle();
        model_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( "model_point_pub", 0 );
        data_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( "data_point_pub", 0 );
    }

    cxy_publisher *cxy_publisher::getInstance()
    {
        cxy_publisher *tmp = m_instance.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (tmp == nullptr)
        {
            std::lock_guard <std::mutex> lock(m_mutex);
            tmp = m_instance.load(std::memory_order_relaxed);
            if (tmp == nullptr)
            {
                tmp = new cxy_publisher;
                std::atomic_thread_fence(std::memory_order_release);
                m_instance.store(tmp, std::memory_order_relaxed);
            }
        }
        return tmp;
    }
    void cxy_publisher::publishModelPoint(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        sensor_msgs::PointCloud2 rosCloud;
        publishPrepare(cloud, rosCloud);
        getInstance()->model_point_pub_.publish(rosCloud);
        return;
    }

    void cxy_publisher::publishDataPoint(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        sensor_msgs::PointCloud2 rosCloud;
        publishPrepare(cloud, rosCloud);
        getInstance()->data_point_pub_.publish(rosCloud);
        return;
    }

    void cxy_publisher::publishPrepare(const pcl::PointCloud<PointT>::Ptr& cloud, sensor_msgs::PointCloud2& rosCloud)
    {
        pcl::toROSMsg(*cloud, rosCloud);
        rosCloud.header.frame_id = cxy_config::rviz_frame_name_;
        rosCloud.header.stamp = ros::Time::now();
        return;
    }
}