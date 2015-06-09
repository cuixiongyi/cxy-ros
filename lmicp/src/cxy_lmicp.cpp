#include "../include/cxy_lmicp.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        cxy_lmicp::cxy_lmicp() : cxy_icp()
        {
            func_ = new cxy_icp_func_rigid;

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
    std::vector<int> modelMatchIdx;
    std::vector<float> matchDistance;
    std::vector<int> dataMatchIdx;
    std::vector<E::Vector3f> residual;



        }

    }
}