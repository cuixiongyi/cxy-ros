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
				cxy_icp_kinematic_joint();
				
					
				//cxy_transform::Pose& getPose() {return pose_;}
				//const cxy_transform::Pose& getPose() const {return pose_;}


		};
		

	}
}