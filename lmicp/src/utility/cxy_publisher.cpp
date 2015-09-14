#include "utility/cxy_publisher.h"

std::atomic<cxy::cxy_publisher*> cxy::cxy_publisher::m_instance;
std::mutex cxy::cxy_publisher::m_mutex;



namespace cxy
{
    cxy_publisher::cxy_publisher()
    {
        //nh_ = ros::NodeHandle();
        model_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( "model_point_pub", 0 );
        data_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( "data_point_pub", 0 );
        arrayMarker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("meshModel_pub",0);
        marker_pub_vec_.resize(cxy_config::joint_number_);
        for (int ii = 0; ii < marker_pub_vec_.size(); ++ii)
        {
            std::string pub_name = "oneModel_pub" + std::to_string(ii);
            marker_pub_vec_[ii] = nh_.advertise<visualization_msgs::Marker>(pub_name, 0);

        }

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
        if ( ! publishPrepare(cloud, rosCloud))
            return;
        getInstance()->model_point_pub_.publish(rosCloud);
        return;
    }

    void cxy_publisher::publishDataPoint(const pcl::PointCloud<PointT>::Ptr& cloud)
    {

        sensor_msgs::PointCloud2 rosCloud;
        if ( ! publishPrepare(cloud, rosCloud))
            return;

        getInstance()->data_point_pub_.publish(rosCloud);
        return;
    }

    bool cxy_publisher::publishPrepare(const pcl::PointCloud<PointT>::Ptr& cloud, sensor_msgs::PointCloud2& rosCloud)
    {
        if (cloud == nullptr)
            return false;
        if (cloud->size() == 0)
            return false;
        pcl::toROSMsg(*cloud, rosCloud);
        rosCloud.header.frame_id = cxy_config::rviz_frame_name_;
        rosCloud.header.stamp = ros::Time::now();
        return true;
    }

    void cxy_publisher::publishMeshModel(visualization_msgs::MarkerArray const& markerArray)
    {

        /*
         * TODO this is a hack, it seems that MarkerArray only display
         * the last mesh model
         * So I can only publish mesh model individually for now
         */
        std::vector<ros::Publisher> const& pub_vec = getInstance()->marker_pub_vec_;
        for (int ii = 0; ii < pub_vec.size(); ++ii)
        {
            pub_vec[ii].publish(markerArray.markers[ii]);

        }
        return;

    }
}