#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <mutex>

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
            cxy_icp_kinematic_chain(const cxy_config* const);
            ~cxy_icp_kinematic_chain();

            void updateJoints();

            void updateModelPoints();

            void setJointPara(const MatrixX1&);

            void constructKinematicChain();


            void setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr getFullModelCloud_World();

            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                             const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent );

            pcl::PointCloud<pcl::PointXYZ>::Ptr getOneModelCloud_World(
                                          const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose);


            void getKinematicPose2World( const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent);

            inline const int size() { return config_->joint_number_; }

            void getResidual(MatrixX1&);

            void getJacobian(MatrixXX&);

        private:

            std::mutex kinematic_chain_lock;
            std::vector<std::shared_ptr<cxy_icp_kinematic_joint<_Scalar>>> joints_;
            cxy_sync jointTime;
            MatrixX1 x_;

            const cxy_config* const config_;
            std::shared_ptr<std::vector<cxy_icp_kinematic_point*>> points_;

            std::vector<int> pointJointIdx_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;
            bool hasSetDataCloud_;
            cxy_sync dataTime;


            /// this is the list to synchronize multithreading, do not access directly
            /// 0 means not up-to-date
            /// 1 means
            std::vector<Update_Status> joint_sync_list;
            void syc_resetSyncList();
            bool syc_tryUpdateJointList(const int &);
            bool syc_isJointUptoDate(const int &);
            bool syc_setJointUptoDate(const int &, const cxy_transform::Pose<_Scalar>&);
        };


    }
}