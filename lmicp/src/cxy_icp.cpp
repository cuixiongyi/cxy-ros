//
// Created by xiongyi on 6/7/15.
//

#include "cxy_icp.h"
#include "../include/cxy_icp.h"

namespace cxy {
    namespace cxy_lmicp_lib {

    cxy_icp::cxy_icp() :  hasSetModelCloud_(false)
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
    cxy_icp::~cxy_icp()
    {
    }


        bool cxy_icp::setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model)
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
    }
}