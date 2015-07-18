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

namespace cxy
{
    namespace cxy_lmicp_lib
    {
		template<typename _Scalar>
        class cxy_icp_kinematic_joint 
        {

		public:
				cxy_transform::Pose<_Scalar> pose_;
				_Scalar theta_;
				pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
				cxy_transform::Axis rotateAxis_;
				
				
			
			public:
				cxy_icp_kinematic_joint();
				
					
				//cxy_transform::Pose& getPose() {return pose_;}
				//const cxy_transform::Pose& getPose() const {return pose_;}

                void setRotateAxis(cxy_transform::Axis axis);

                void setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

		};
		

	}
}