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
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_ros/transforms.h"

#include "geometric_shapes/shape_operations.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

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
        typedef Eigen::Matrix< float, Eigen::Dynamic, 7> MatrixX7f;
        typedef Eigen::Matrix< float, 7, 7> Matrix7f;
        typedef Eigen::Matrix< float, 7, 1> Vector7f;
        typedef Eigen::Matrix< float, 6, 6> Matrix6f;
        typedef Eigen::Matrix< float, 6, 1> Vector6f;

        typedef Eigen::Matrix< float, 4, 4> Matrix44f;

        class cxy_icp {


        public:
            cxy_icp();

            virtual float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, cxy_transform::Pose &outPose);

            bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

            //bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

        private:
            bool hasSetModelCloud_;
            ros::NodeHandle nh_, pnh_;
            ros::Publisher pub_model_, pub_model_pointcloud_, pub_data_pointcloud_, pub_result_;
            pcl::KdTreeFLANN<PointT>::Ptr kdtreeptr_;

        protected:
            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;

        };
    }
}
#endif //PROJECT_LMICP_CXY_ICP_H
