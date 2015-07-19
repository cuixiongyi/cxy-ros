#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_joint_info.h"
#include "common/cxy_config.h"
#include "common/hack.hpp"
namespace cxy
{
    namespace cxy_lmicp_lib
    {
		template<typename _Scalar>
        class cxy_icp_kinematic_joint 
        {

		public:

            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;

			
			public:
				cxy_icp_kinematic_joint(const std::shared_ptr<cxy_config>&, const int&);
                void init();
					
				//cxy_transform::Pose& getPose() {return pose_;}
				//const cxy_transform::Pose& getPose() const {return pose_;}
		private:
			std::shared_ptr<cxy_config> config_;
            cxy_joint_info joint_info_;
            float theta_;


        /// inline function
        public:
            inline const int&                   getParent() {return joint_info_.joint_parent;}
            inline cxy_transform::Axis&         getJointType()  {return joint_info_.jointType;}
            inline std::string&                 getModelFileName() {return joint_info_.model_filename;}
            inline const int&                   numDoF() {return joint_info_.DoF;}
            inline const cxy_transform::Pose<float>&  getPose() { return joint_info_.pose_;}
            inline const pcl::PointCloud<pcl::PointXYZ>::Ptr&  getModelCloud() { return joint_info_.modelCloud_;}
        };
		

	}
}