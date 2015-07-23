#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic_joint.h"
#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        class cxy_icp_kinematic_point
        {
        public:
            cxy_icp_kinematic_point(const cxy_config* const , const int& );

            ~cxy_icp_kinematic_point();

            cxy_icp_kinematic_point(const cxy_icp_kinematic_point&) = delete;
            cxy_icp_kinematic_point& operator=(const cxy_icp_kinematic_point&) = delete;

            void init();

            void computePointResidual();
            void computePointJacobian();

            static _Scalar matchPointCloud(const PointT& model
                                        , const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr&
                                        , const PointT&
                                        , Eigen::Matrix< _Scalar, 3, 1>& res);
        //private:

            const int joint_idx_ = {0};

            const cxy_config* const config_;
            Eigen::Matrix<_Scalar, 3, 1> modelPoint_local_;
            PointT modelPoint_global_;
            PointT dataPoint_;

            _Scalar point_resdual1_
            Eigen::Matrix< _Scalar, 3, 1> point_resdual3_;
            Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> jacobian_;

        };
    }
}