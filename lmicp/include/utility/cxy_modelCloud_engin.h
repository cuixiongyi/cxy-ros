#pragma once

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

#include "common/cxy_common.h"
#include "common/cxy_config.h"
#include "utility/cxy_CAD_helper.h"
namespace cxy
{

    class cxy_modelCloud_engin
    {
    protected:
        static ros::NodeHandle             nh_;
        static ros::Publisher data_point_pub_;
        static ros::Publisher model_point_pub_;


        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_normal_;

        cxy_CAD_helper cad_helper_;

    public:
        cxy_modelCloud_engin(const std::string& filename );
        ~cxy_modelCloud_engin();

        const pcl::PointCloud<pcl::PointXYZ>::Ptr& getModelCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getVisibleCloud(const cxy_transform::Pose<float>& pose);



        static void publishModelPoint(const pcl::PointCloud<PointT>::Ptr& cloud);
        static void publishDataPoint(const pcl::PointCloud<PointT>::Ptr& cloud);
        static void publishPrepare(const pcl::PointCloud<PointT>::Ptr& cloud, sensor_msgs::PointCloud2& rosCloud);


    };
}