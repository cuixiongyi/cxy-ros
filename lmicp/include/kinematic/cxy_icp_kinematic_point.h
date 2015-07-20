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
            cxy_icp_kinematic_point(const std::shared_ptr<const cxy_config>&);
            ~cxy_icp_kinematic_point();

            void computePointResidual();
            void computePointJacobian();

        private:

            const std::shared_ptr<const cxy_config>& config_;
            Eigen::Matrix<_Scalar, 3, 1> modelPoint_local_;
            Eigen::Matrix<_Scalar, 3, 1> dataPoint_;
            const int& joint_idx = {0};
            Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> point_resdual;
            Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> jacobian;

        };
    }
}