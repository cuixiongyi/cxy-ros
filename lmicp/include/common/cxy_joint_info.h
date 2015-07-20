#pragma once

#include <Eigen/Core>
#include <vector>

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"


namespace cxy
{
    struct cxy_joint_info
    {
        int joint_idx = {-1};
        int joint_parent = {-1};
        int DoF = {1};

        cxy_transform::Axis jointType;
        std::string model_filename;
        std::vector<double> t = {0.0, 0.0, 0.0};
        std::vector<double> r = {0.0, 0.0, 0.0};

    };

}
