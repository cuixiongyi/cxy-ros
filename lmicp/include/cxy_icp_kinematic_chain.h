#pragma once
#include "cxy_transform.h"
#include "cxy_debug.h"
#include "cxy_icp_kinematic_node.h"
#include "common/cxy_common.h"

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        class cxy_icp_kinematic_chain
        {

        private:

            std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kc_nodes_;
            std::vector<int> kc_root_list_;


        public:
            cxy_icp_kinematic_chain();

            pcl::PointCloud<pcl::PointXYZ>::Ptr getTransCloud();

            //std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& getKinematicChainNodes();
            const std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& getKinematicChainNodes() const;
            void setKinematicNodes( std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kin_nodes);
            void setKinematicRootList( std::vector<int>& list);

            pcl::PointCloud<pcl::PointXYZ>::Ptr getFullModelCloud(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x);
            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x, const int& joint);
            void getPose2Root(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x, const int& joint, cxy_transform::Pose<_Scalar>& pose);

            inline const int size() {CXY_ASSERT( kc_root_list_.size() == kc_nodes_->size());
                                    return kc_root_list_.size();};
        };


    }
}