#pragma once

#include <atomic>
#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

#include "common/cxy_common.h"
#include "common/cxy_config.h"

namespace cxy
{
    class cxy_publisher
    {
        public:
            static cxy_publisher* getInstance();


            static void publishModelPoint(
                    const pcl::PointCloud<PointT>::Ptr& cloud);

            static void publishDataPoint(
                    const pcl::PointCloud<PointT>::Ptr& cloud);

            static bool publishPrepare(
                        const pcl::PointCloud<PointT>::Ptr& cloud
                        , sensor_msgs::PointCloud2& rosCloud);

        private:
            cxy_publisher();
            static std::atomic<cxy::cxy_publisher*> m_instance;
            static std::mutex m_mutex;

            ros::NodeHandle             nh_;
            ros::Publisher data_point_pub_;
            ros::Publisher model_point_pub_;

    };

}