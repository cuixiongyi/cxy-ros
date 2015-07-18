#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic_joint.h"
#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{
		class cxy_icp_kinematic
		{
		public:
			cxy_icp_kinematic(std::shared_ptr<cxy_config>);
			~cxy_icp_kinematic();
			std::shared_ptr<cxy_config> config_;

		private:
            std::shared_ptr<cxy_icp_kinematic_chain> kc_;
            std::shared_ptr<std::vector<cxy_icp_kinematic_point>> points_;

            Eigen::Matrix< float, Eigen::Dynamic, 1> x_;


		};
	}
}