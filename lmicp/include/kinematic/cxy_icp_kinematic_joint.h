#pragma once

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "utility/cxy_transform.h"
#include "common/cxy_debug.h"
#include "common/cxy_common.h"
#include "common/cxy_joint_info.h"
#include "common/cxy_config.h"
//#include "common/hack.hpp"
namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        class cxy_icp_kinematic_chain;

		template<typename _Scalar>
        class cxy_icp_kinematic_joint 
        {
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixXX;


		public:


			
			public:
            cxy_icp_kinematic_joint(const cxy_config* const, const int&, const cxy_icp_kinematic_chain<_Scalar>*);
            ~cxy_icp_kinematic_joint();
                void init();
            const cxy_joint_info joint_info_;
            void setChildList(const std::vector<const cxy_icp_kinematic_joint*>&);
            void setParent(const cxy_icp_kinematic_joint* );
				//cxy_transform::Pose& getPose() {return pose_;}
				//const cxy_transform::Pose& getPose() const {return pose_;}
		private:
			const cxy_config* const config_;
            const cxy_icp_kinematic_chain<_Scalar>* kc_ptr_;
            const int joint_idx_;
            _Scalar* theta_;
            const int DoF_;

            cxy_transform::Pose<_Scalar> pose_;
            const cxy_transform::Pose<_Scalar> originPose_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;

            const cxy_icp_kinematic_joint* pParent_;
            std::vector<const cxy_icp_kinematic_joint*> pChildList_;

        /// inline function
        public:
            inline const int&                   getParent() const {return joint_info_.joint_parent;}
            inline const cxy_transform::Axis&         getJointType() const {return joint_info_.jointType;}
            inline std::string&                 getModelFileName() {return joint_info_.model_filename;}
            inline const int&                   numDoF() {return joint_info_.DoF;}
            inline void setPose(const cxy_transform::Pose<_Scalar>& pose) {pose_ = pose;}
            inline void setPose(cxy_transform::Pose<_Scalar>&& pose) {pose_ = std::move(pose);}
            inline const cxy_transform::Pose<_Scalar>&  getPose() const { return pose_;}
            inline const cxy_transform::Pose<_Scalar>&  getOriginPose() { return originPose_;}
            inline const pcl::PointCloud<pcl::PointXYZ>::Ptr&  getModelCloud() { return modelCloud_;}
            inline void setTheta(const _Scalar* p) { std::memcpy(theta_, p, sizeof(_Scalar)*DoF_);};
            inline const _Scalar* getTheta() { return theta_;};

            /*
             * query about what's relation between joint a and b
             */


        };
		

	}
}