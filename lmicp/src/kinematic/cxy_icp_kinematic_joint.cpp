
#include "kinematic/cxy_icp_kinematic_joint.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{

        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::cxy_icp_kinematic_joint(const std::shared_ptr<const cxy_config>& config_ptr, const int& joint_idx)
        : joint_info_(config_->joint_config_[joint_idx])
        {
            config_ = config_ptr;

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>::init()
        {

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

