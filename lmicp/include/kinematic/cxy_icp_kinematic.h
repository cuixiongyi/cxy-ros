#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>
#include <mutex>
#include <boost/pool/poolfwd.hpp>

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
        template<typename _Scalar>
        class cxy_icp_kinematic
		{
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixXX;
		public:
			cxy_icp_kinematic( cxy_config const*  );
			~cxy_icp_kinematic();
            const cxy_config* const config_;

            void computeResidual(const MatrixX1&, MatrixX1&);

            void computeJacobian(const MatrixX1&, MatrixXX&);


            void setDataCloud(pcl::PointCloud<PointT>::Ptr data);

            /**
             * This function update joint model with joint angle(1 or 6 DoF) of x_
             */
            void updateJointModel(const MatrixX1&);




        private:
            std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc_;
			std::vector<std::shared_ptr<cxy_icp_kinematic_point<_Scalar>>> points_;

            Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> joint_para;
            static std::once_flag joint_Parent_init;


        };
	}
}