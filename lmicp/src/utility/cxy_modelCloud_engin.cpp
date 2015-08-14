#include "utility/cxy_modelCloud_engin.h"



namespace cxy
{
    ros::NodeHandle cxy_modelCloud_engin::nh_;

    ros::Publisher cxy_modelCloud_engin::model_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2_>( "model_point_pub", 0 );
    ros::Publisher cxy_modelCloud_engin::data_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2_>( "data_point_pub_", 0 );

    cxy_modelCloud_engin::cxy_modelCloud_engin(const std::string& filename )
    : modelCloud_(new pcl::PointCloud<PointT>)
     , modelCloud_normal_(new pcl::PointCloud<PointT>)
    {
        SamplingParams para(500, 0.005, SampleType::RANDOM);
        if (! cad_helper_.meshToPointCloud(filename, *modelCloud_, *modelCloud_normal_, para))
            throw std::runtime_error("CAD model import failed");


    }

    cxy_modelCloud_engin::~cxy_modelCloud_engin()
    {

    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr&
                        cxy_modelCloud_engin::getModelCloud()
    {
        return modelCloud_;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cxy_modelCloud_engin::getVisibleCloud(const cxy_transform::Pose<float>& pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);
        pose.composePoint(modelCloud_, transCloud);
        pose.composeDirectionVector(modelCloud_normal_, transCloud_normal);

        cad_helper_.filterOccludedPoints(transCloud, retCloud, transCloud_normal, retCloud_normal, Eigen::Vector3d(1.0, 0.0, 0.0));

        return retCloud;
    }




    void cxy_modelCloud_engin::publishModelPoint(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        sensor_msgs::PointCloud2 rosCloud;
        publishPrepare(cloud, rosCloud);
        model_point_pub_.publish(rosCloud);
        return;
    }
    void cxy_modelCloud_engin::publishDataPoint(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        sensor_msgs::PointCloud2 rosCloud;
        publishPrepare(cloud, rosCloud);
        data_point_pub_.publish(rosCloud);
        return;
    }
    void cxy_modelCloud_engin::publishPrepare(const pcl::PointCloud<PointT>::Ptr& cloud, sensor_msgs::PointCloud2& rosCloud)
    {
        pcl::toROSMsg(*cloud, rosCloud);
        rosCloud.header.frame_id = cxy_config::rviz_frame_name_;
        rosCloud.header.stamp = ros::Time::now();
        return;
    }

}