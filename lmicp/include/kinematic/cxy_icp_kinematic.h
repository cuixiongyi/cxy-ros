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
	namespace cxy_kinematic
	{
        template<typename _Scalar>
        class cxy_icp_kinematic
		{
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

		public:
			cxy_icp_kinematic(const cxy_config *const  );
			~cxy_icp_kinematic();
            const cxy_config* const config_;

            void computeResidual(const MatrixX1&, MatrixX1&);

            void computeJacobian(const MatrixX1&, MatrixXX&);



            /**
             * This function update joint model with joint angle(1 or 6 DoF) of x_
             */
            void updateJoints( MatrixX1 const&);

            void constructModelPoints( MatrixX1 const&);


            inline void setModelPointSize( std::size_t const& num)  { model_Point_num_ = num;}
            inline  std::size_t const& getModelPointSize() const { return model_Point_num_;}


            inline pcl::PointCloud<pcl::PointXYZ>::Ptr const& getDataCloud() const
            {
                return dataCloud_;
            }

            inline void setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& dataCloud)
            {
                cxy_icp_kinematic::dataCloud_ = dataCloud;
                kc_->setDataCloud(dataCloud_);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr const& getVisibleModelCloud(MatrixX1 const&);
            pcl::PointCloud<pcl::PointXYZ>::Ptr const& getVisibleModelCloud(
                    MatrixX1 const& x
                    , pcl::PointCloud<pcl::PointXYZ>::Ptr & fullCloud );
        private:
            std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc_;

            mutable MatrixX1 x_;
            static std::once_flag joint_Parent_init;

            std::size_t model_Point_num_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr visibleModelCloud_;

        };
	}
}