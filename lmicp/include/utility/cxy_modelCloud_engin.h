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



        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_normal_;

        cxy_CAD_helper cad_helper_;

    public:
        cxy_modelCloud_engin(const std::string& filename );
        ~cxy_modelCloud_engin();

        const pcl::PointCloud<pcl::PointXYZ>::Ptr& getModelCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getVisibleCloud(const cxy_transform::Pose<float>& pose);


    };
}