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
        class cxy_icp_kinematic_point
        {
        public:
            cxy_icp_kinematic_point();
            ~cxy_icp_kinematic_point();

        private:

            std::shared_ptr<cxy_config> config_;
            Eigen::Matrix<float, 3, 1> modelPoint_local_;
            Eigen::Matrix<float, 3, 1> dataPoint_;

        };
    }
}