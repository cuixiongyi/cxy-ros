//
// Created by xiongyi on 6/7/15.
//

#ifndef PROJECT_LMICP_CXY_ICP_H
#define PROJECT_LMICP_CXY_ICP_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "ros/ros.h"
#include "cxy_transform.h"

//#include <vnl/vnl_vector.h>
//#include <vnl/vnl_matrix.h>

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        namespace ctrs = cxy_transform;
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT>    PointCloud;
        typedef pcl::PointCloud<PointT>::Ptr    PointCloudPtr;
        typedef pcl::PointCloud<PointT>::ConstPtr    PointCloudConstPtr;
        typedef Eigen::Matrix< float, 3, 4> Matrix34f;
        typedef Eigen::Matrix< float, 3, 7> Matrix37f;
        typedef Eigen::Matrix< float, E::Dynamic, 7> MatrixX7f;
        typedef Eigen::Matrix< float, 7, 7> Matrix7f;
        typedef Eigen::Matrix< float, 7, 1> Vector7f;
        typedef Eigen::Matrix< float, 6, 6> Matrix6f;
        typedef Eigen::Matrix< float, 6, 1> Vector6f;

        typedef E::Matrix< float, 4, 4> Matrix44f;

        class cxy_icp {


        public:
            cxy_icp();

            virtual float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, cxy_transform::Pose &outPose) = 0;

            virtual float setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) = 0;


        private:
            bool hasSetModelCloud_;
            ros::NodeHandle nh_, pnh_;
            ros::Publisher pub_model_, pub_model_pointcloud_, pub_data_pointcloud_, pub_result_;

        protected:
            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;

        };
    }
}
#endif //PROJECT_LMICP_CXY_ICP_H
