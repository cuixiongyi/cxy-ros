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

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        class cxy_icp {


        public:
            cxy_icp();

            virtual float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, cxy_transform::Pose &outPose) = 0;

            virtual float setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) = 0;

        private:
            bool hasSetModelCloud;
        protected:
            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
        };
    }
}
#endif //PROJECT_LMICP_CXY_ICP_H
