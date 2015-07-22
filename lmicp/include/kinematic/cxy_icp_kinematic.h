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
        template<typename _Scalar>
        class cxy_icp_kinematic
		{
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixXX;
		public:
			cxy_icp_kinematic(const std::shared_ptr<const cxy_config>&);
			~cxy_icp_kinematic();
			const std::shared_ptr<const cxy_config> config_;

            void computeResidual(const MatrixX1&, MatrixX1&);

            void computeJacobian(const MatrixX1&, MatrixXX&);


            void setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data);

            void updateJointModel(const MatrixX1&);


        private:
            std::shared_ptr<cxy_icp_kinematic_chain> kc_;
			std::vector<std::shared_ptr<cxy_icp_kinematic_point>> points_;

            Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> joint_para;

            int matrix_rows_ = {0};
            int matrix_rows_Jac_ = {0};
            int matrix_cols_Jac_ = {0};

            void setMatrixSize(const int&);

        };
	}
}