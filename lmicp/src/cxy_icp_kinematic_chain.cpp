#pragma once
#include "cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        cxy_icp_kinematic_chain<_Scalar>::cxy_icp_kinematic_chain()
        {

        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getFullModelCloud(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x)
        {
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            unsigned int pointCloudSize = 0;
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pointCloudSize += (*kc_nodes_)[ii].modelCloud_->size();

            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            transCloud->reserve(pointCloudSize);


        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x, const int joint)
        {
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            CXY_ASSERT(x.rows() >= joint);

            unsigned int pointCloudSize = 0;
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pointCloudSize += (*kc_nodes_)[ii].modelCloud_->size();

            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            transCloud->reserve(pointCloudSize);


        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getPose2Root(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x, const int joint, cxy_transform::Pose& pose)
        {
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            CXY_ASSERT(x.rows() >= joint);

            if ( -1 == kc_root_list_[joint])
            {
                pose = cxy_transform::Pose();
                pose.
                return;
            }
            else
            {
                getPose2Root(x, kc_root_list_[joint], pose);
            }



        }

        /*
        template<typename _Scalar>
        std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes()
        {
            return kc_nodes_;
        }*/

        template<typename _Scalar>
        const std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes() const
        {
            return kc_nodes_;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicNodes(const std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kin_nodes)
        {
            CXY_ASSERT(nullptr == kin_nodes );
            kc_nodes_ = kin_nodes;
            return;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicRootList(const std::vector<int> &list)
        {
            CXY_ASSERT( 0 == list.size() );

            kc_root_list_ = list;
            return;

        }


    }
}


template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<double>;
