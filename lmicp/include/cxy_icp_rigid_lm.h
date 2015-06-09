#pragma once

#include "cxy_icp_rigid.h"
#include <dlib/optimization.h>

namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_icp_rigid_lm : cxy_icp_rigid
        {

            cxy_icp_rigid_lm();

            float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose);

        };
    }
}