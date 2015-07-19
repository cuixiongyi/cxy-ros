#pragma once

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"

#include <Eigen/Core>

namespace cxy
{
    struct cxy_joint_info
    {
        int joint_idx = {-1};
        int joint_parent = {-1};
        int DoF = {1};

        cxy_transform::Axis jointType;
        std::string model_filename;
        float t[3];
        float r[3];

        cxy_transform::Pose<float> pose_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
    };

}
