//
// Created by xiongyi on 6/7/15.
//

#include "cxy_icp.h"
#include "../include/cxy_icp.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        template<typename _Scalar>
        cxy_icp<_Scalar>::cxy_icp() :  hasSetModelCloud_(false)
                            , pnh_("~")
        {
            /*
            pnh_.param<int>("max_iterations", max_iterations_, 1000);
            pnh_.param<double>("transformation_epsilon", transformation_epsilon_, 1e-8);
            pnh_.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.1);
            pnh_.param<double>("max_correspondence_dist", max_correspondence_dist_, 0.07); // 0.2
            max_correspondence_dist_square_ = max_correspondence_dist_*max_correspondence_dist_;
            */
            pub_model_ = nh_.advertise<visualization_msgs::MarkerArray>("model", 5);
            pub_data_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("data_pointcloud", 5);
            pub_model_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("model_pointcloud", 5);
            pub_result_ = nh_.advertise<sensor_msgs::PointCloud2>("data_result", 5);

        }
        template<typename _Scalar>
        cxy_icp<_Scalar>::~cxy_icp()
        {
        }

        template<typename _Scalar>
        bool cxy_icp<_Scalar>::setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model)
        {
            if (nullptr == model || model->size() < 10)
            {
                return false;
            }
            hasSetModelCloud_ = true;
            //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*model);
            //translateToCenter(tmp);
            //modelCloud_ = tmp;
            modelCloud_ = model;

            kdtreeptr_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
            kdtreeptr_->setInputCloud(modelCloud_);
            return true;

        }
        template<typename _Scalar>
        _Scalar cxy_icp<_Scalar>::icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose)
        {
            if ( ! hasSetModelCloud_)
            {
                return -1.0;
            }
            icp_prepare_cost_function();
            icp_minimization();

            publish(dataCloud_, pub_data_pointcloud_);
            publish(modelCloud_, pub_model_pointcloud_);
        }
        template<typename _Scalar>
        void cxy_icp<_Scalar>::publish(const PointCloudConstPtr& data, const ros::Publisher& pub)
        {
            sensor_msgs::PointCloud2    cloud_msg;
            //ROS_INFO_STREAM(data->size());
            pcl::toROSMsg(*data, cloud_msg);
            cloud_msg.header.frame_id = "icp";
            cloud_msg.header.stamp = ros::Time::now();

            //pub_depth = nhTmpe.advertise<sensor_msgs::PointCloud2>("depth_point_cloud_visualservo", 2);
            pub.publish(cloud_msg);
            return;
        }

    }
}