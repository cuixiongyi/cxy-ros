#include "../include/cxy_icp_rigid_lm.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        cxy_icp_rigid_lm::cxy_icp_rigid_lm() : cxy_icp_rigid()
        {

        }



        float cxy_lmicp::icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose)
        {
            if ( ! hasSetModelCloud_)
            {
                return -1.0;
            }
            dataCloud_ = data;

            ctrs::Pose guessTmp(0);
            PointCloudPtr transDataCloud;

    ctrs::Pose pose_inc(0);
    ctrs::Pose pose_tmp(0);




        }

    }
}