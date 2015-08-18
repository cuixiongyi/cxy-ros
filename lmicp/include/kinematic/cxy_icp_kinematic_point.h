#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <memory>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_config.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        class cxy_icp_kinematic_joint;
        template<typename _Scalar>
        class cxy_icp_kinematic_chain;

        template<typename _Scalar>
        class cxy_icp_kinematic_point
        {
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;


        public:
            cxy_icp_kinematic_point(const cxy_config* const
                                    , const pcl::KdTreeFLANN<PointT>::Ptr&
                                    , const pcl::PointCloud<PointT>::Ptr&
                                    , const cxy_icp_kinematic_joint<_Scalar>* const
                                    , const cxy_icp_kinematic_chain<_Scalar>* kc_ptr_);

            ~cxy_icp_kinematic_point() {};

            cxy_icp_kinematic_point(const cxy_icp_kinematic_point&) = delete;
            cxy_icp_kinematic_point& operator=(const cxy_icp_kinematic_point&) = delete;

            void init();

            void computePointResidual(Eigen::Ref<MatrixX1>);

            void computePointJacobian(Eigen::Ref<MatrixXX> jac);

            void compute_icp_jacobian(Eigen::Ref<MatrixXX> jac);
            void compute_collision_jacobian(Eigen::Ref<MatrixXX> jac);
            void compute_push_jacobian(Eigen::Ref<MatrixXX> jac);
            void compute_silhouette_jacobian(Eigen::Ref<MatrixXX> jac);

            const _Scalar& matchPointCloud(const PointT&
                                        , PointT&
                                        , Eigen::Matrix< _Scalar, 3, 1>& res);
        //private:

            const int joint_idx_ = {0};

            const cxy_config* const config_;
            const cxy_icp_kinematic_joint<_Scalar>* const joint_;
            const cxy_icp_kinematic_chain<_Scalar>* kc_ptr_;


            Eigen::Matrix<_Scalar, 3, 1> modelPoint_local_;
            PointT modelPoint_global_;
            PointT dataPoint_;

            _Scalar point_resdual1_;
            Eigen::Matrix< _Scalar, 3, 1> point_resdual3_;
            Eigen::Matrix< _Scalar, CXY_JACO_TYPE_COUNT_FINAL, 1> point_resdual_n;

            Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> jacobian_;

            const pcl::KdTreeFLANN<PointT>::Ptr& kdtree_ptr_;
            const pcl::PointCloud<PointT>::Ptr& dataCloud_;

        };
    }
}