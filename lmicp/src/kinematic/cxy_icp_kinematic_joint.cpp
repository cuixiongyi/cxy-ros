
#include "kinematic/cxy_icp_kinematic_joint.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{
		template<typename _Scalar>
		cxy_icp_kinematic_joint<_Scalar>::cxy_icp_kinematic_joint()
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

