#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>


// hack.hpp should not appear at last
#include "common/hack.hpp"
#include "utility/cxy_transform.h"
#include "utility/cxy_sync.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_config.h"

#include "kinematic/cxy_icp_kinematic_joint.h"
#include "kinematic/cxy_icp_kinematic_point.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        class cxy_icp_kinematic_chain
        {

            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixXX;
        public:
            cxy_icp_kinematic_chain(const std::shared_ptr<const cxy_config>&);
            ~cxy_icp_kinematic_chain();

            void constructKinematicChain();


            void setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);


            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                             const MatrixX1& x
                                             , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent );

            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                            const MatrixX1& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose);

            pcl::PointCloud<pcl::PointXYZ>::Ptr getFullModelCloud_World( const MatrixX1& x);
            void getKinematicPose2World( const MatrixX1& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent);

            inline const int size() { return config_->joint_number_; }


        private:

            std::vector<std::shared_ptr<cxy_icp_kinematic_joint<_Scalar>>> joints_;
            cxy_sync jointTime;

            const std::shared_ptr<const cxy_config>& config_;
            std::shared_ptr<std::vector<cxy_icp_kinematic_point>> points_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;
            bool hasSetDataCloud_;
            cxy_sync dataTime;
        };


    }
}