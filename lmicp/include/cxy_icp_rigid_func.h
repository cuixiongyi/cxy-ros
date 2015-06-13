#pragma once
#include "optimization/cxy_cost_func_abstract.h"
#include "cxy_transform.h"

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


        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        class cxy_icp_rigid_func : public cxy_optimization::Cxy_Cost_Func_Abstract< _Scalar, NX, NY>
        {

            typedef pcl::PointXYZ PointT;
            typedef pcl::PointCloud<PointT>    PointCloud;
            typedef pcl::PointCloud<PointT>::Ptr    PointCloudPtr;
            typedef pcl::PointCloud<PointT>::ConstPtr    PointCloudConstPtr;
            typedef Eigen::Matrix< _Scalar, 3, 4> Matrix34f;
            typedef Eigen::Matrix< _Scalar, 3, 7> Matrix37f;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 7> MatrixX7f;
            typedef Eigen::Matrix< _Scalar, 7, 7> Matrix7f;
            typedef Eigen::Matrix< _Scalar, 7, 1> Vector7f;
            typedef Eigen::Matrix< _Scalar, 6, 6> Matrix6f;
            typedef Eigen::Matrix< _Scalar, 6, 1> Vector6f;
            typedef Eigen::Matrix< _Scalar, 3, 1> Vector3;
            typedef Eigen::Matrix< _Scalar, 4, 4> Matrix44f;

            public:
                enum {
                    ParaAtCompileTime = NX,
                    DataAtCompileTime = NY
                };

                //: all of this is what you need in the derived function

                typedef Eigen::Matrix<_Scalar,ParaAtCompileTime,1> ParaType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,1> ResidualType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,ParaAtCompileTime> JacobianType;

                cxy_icp_rigid_func(pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud
                                                   , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                                                   , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr);

                _Scalar operator()(ParaType & x, ResidualType& fvec) const;
                _Scalar df(ParaType & x, JacobianType& fjac) const;
                const Eigen::Matrix< _Scalar, 3, 4> calculateJacobianKernel(const std::vector<_Scalar> &para
                                                        , const pcl::PointXYZ& a) const;

                const _Scalar matchPointCloud(const PointT& data
                                                   , Eigen::Matrix< _Scalar, 3, 1>& res) const;

            private:
                pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
                pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;

        };
    }

}