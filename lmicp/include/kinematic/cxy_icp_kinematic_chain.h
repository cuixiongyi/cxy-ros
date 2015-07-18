#pragma once
#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "kinematic/cxy_icp_kinematic_node.h"
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

            inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr getModelCloud(int joint)
            {   CXY_ASSERT(joint >= 0 && joint < kc_nodes_->size());
                return (*kc_nodes_)[joint].modelCloud_;
                };

            //std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& getKinematicChainNodes();
            const std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& getKinematicChainNodes() const;
            void setKinematicNodes( std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kin_nodes);
            void setKinematicRootList( std::vector<int>& list);


            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                             Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent );

            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                             Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose);

            pcl::PointCloud<pcl::PointXYZ>::Ptr getFullModelCloud_World( Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x);
            void getKinematicPose2World( Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                      , const int& joint
                                      , cxy_transform::Pose<_Scalar>& pose
                                      , cxy_transform::Pose<_Scalar>& pose_parent);

            inline const int size() {CXY_ASSERT( kc_root_list_.size() == kc_nodes_->size());
                                    return kc_root_list_.size();};
        };


    }
}