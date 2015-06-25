#pragma once
#include "cxy_transform.h"
#include "cxy_debug.h"
#include "common/cxy_common.h"

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
		template<typename _Scalar>
        class cxy_icp_kinematic_node 
        {

		public:
				cxy_transform::Pose<_Scalar> pose_;
				_Scalar theta_;
				pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
				cxy_transform::Axis rotateAxis_;
				
				
			
			public:
				cxy_icp_kinematic_node();
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr getTransCloud();
					
				cxy_transform::Pose& getPose() {return pose_;}
				const cxy_transform::Pose& getPose() const {return pose_;}

                void setRotateAxis(cxy_transform::Axis axis) {rotateAxis_ = axis;}

                void setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model) {modelCloud_ = model;}
			
		};
		

	}
}