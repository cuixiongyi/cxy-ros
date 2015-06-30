#pragma once
#include "optimization/cxy_cost_func_abstract.h"
#include "cxy_icp_kinematic_chain.h"
#include "cxy_transform.h"
#include "cxy_debug.h"
#include "common/cxy_common.h"
#include <cstdlib>
#include <fstream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_ros/transforms.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {


        template<typename _Scalar>
        class cxy_icp_arti_func : public cxy_optimization::Cxy_Cost_Func_Abstract< _Scalar, Eigen::Dynamic, Eigen::Dynamic>
        {

            typedef pcl::PointXYZ PointT;
            typedef pcl::PointCloud<PointT>    PointCloud;
            typedef pcl::PointCloud<PointT>::Ptr    PointCloudPtr;
            typedef pcl::PointCloud<PointT>::ConstPtr    PointCloudConstPtr;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
            typedef Eigen::Matrix< _Scalar, 4, 4> Matrix44;

            public:
                enum {
                    ParaAtCompileTime = Eigen::Dynamic,
                    DataAtCompileTime = Eigen::Dynamic
                };

                //: all of this is what you need in the derived function

                typedef Eigen::Matrix<_Scalar,ParaAtCompileTime,1> ParaType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,1> ResidualType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,ParaAtCompileTime> JacobianType;

            cxy_icp_arti_func(int nPara
                    , std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc
                    , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                    , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr
                    , int joint
                    , Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> x_full
                    
            );

                _Scalar operator()(ParaType & x, ResidualType& fvec) const;


                _Scalar df(ParaType & x, JacobianType& fjac) const;


                const _Scalar matchPointCloud(const PointT& data
                                               , Eigen::Matrix< _Scalar, 3, 1>& res) const;

                const Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> calculateJacobianKernel(
                                                const Eigen::Matrix< _Scalar, Eigen::Dynamic, 1>& x
                                            , const pcl::PointXYZ& a
                                            , const cxy_transform::Pose<_Scalar> &para_pose
                                            , const cxy_transform::Pose<_Scalar> &para_pose_parent
                                            ) const;

                void manifold() const;

            private:
                pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
                pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;
                std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc_;
                int joint_;
                mutable Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> x_full_;


        };
    }

}