#pragma once

#include "cxy_icp_rigid.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_icp_rigid_lm : public cxy_icp_rigid
        {

        protected:



        public:

            cxy_icp_rigid_lm();

            float icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose);
            void publish(const PointCloudConstPtr& data, const ros::Publisher& pub);

        };
    }
}