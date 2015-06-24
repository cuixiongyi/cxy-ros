#pragma once
#include "optimization/cxy_cost_func_abstract.h"
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
#include "pcl_ros/transforms.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
		template<typename _Scalar>
        class cxy_icp_kinematic_node 
        {
			
			private:
				cxy_transform::Pose pose_;
				_Scalar theta_;
				pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
				
				
				
			
			public:
				cxy_icp_kinematic_node();
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr getTransCloud();
					
				
				
			
		}
		

	}
}