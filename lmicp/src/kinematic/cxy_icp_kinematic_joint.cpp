
#include "kinematic/cxy_icp_kinematic_joint.h"

#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{


        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::cxy_icp_kinematic_joint(const cxy_config* const config_ptr
                                                                  , const int& joint_idx
                                                                  , const cxy_icp_kinematic_chain<_Scalar>* kc_ptr)
        : joint_info_(config_->joint_config_[joint_idx])
            , config_(config_ptr)
            , kc_ptr_(kc_ptr)
            , joint_idx_(joint_idx)
            , DoF_(joint_info_.DoF)
            , originPose_(joint_info_.t[0], joint_info_.t[1], joint_info_.t[2]
                          , joint_info_.r[0], joint_info_.r[1], joint_info_.r[2])
        {
            theta_ = new _Scalar [DoF_];
        }

        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::~cxy_icp_kinematic_joint()
        {
            delete[] theta_;
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>::init()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>::setChildList
                    (const std::vector<const cxy_icp_kinematic_joint*>& childList)
        {
            pChildList_ = childList;
            return;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>:: setParent(const cxy_icp_kinematic_joint* parent)
        {
            pParent_ = parent;
            return;
        }



/*
        template<typename _Scalar>
        cxy_transform::Pose& cxy_icp_kinematic_joint<_Scalar>::getPose()
        {
            return pose_;
        }

        template<typename _Scalar>
        const cxy_transform::Pose& cxy_icp_kinematic_joint<_Scalar>::getPose() const
        {
            return pose_;
        }
*/





    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_joint<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_joint<double>;

