#pragma once

#include "cxy_lmicp.h"
#include "cxy_icp_func_rigid.h"
#include <dlib/optimization.h>

namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_lmicp : cxy_icp
        {

            cxy_lmicp();

            float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose);

        };
    }
}