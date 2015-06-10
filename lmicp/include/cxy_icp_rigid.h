#pragma once

#include "cxy_icp.h"
#include "cxy_transform.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_icp_rigid : cxy_icp
        {

            cxy_icp_rigid();

            virtual float matchPointCloud();

            virtual bool matchPointCloud();
            virtual const float matchPointCloud(const pcl::PointXYZ& data
                    , Eigen::Vector3f& res);

        };
    }
}