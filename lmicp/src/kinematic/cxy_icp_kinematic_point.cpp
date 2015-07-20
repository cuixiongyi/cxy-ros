
#include "kinematic/cxy_icp_kinematic_point.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        cxy_icp_kinematic_point<_Scalar>::cxy_icp_kinematic_point(const std::shared_ptr<const cxy_config>& config_ptr, const int& joint_idx)
                : config_(config_ptr)
                , joint_idx_(joint_idx)
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::init()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointResidual()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointJacobian()
        {

        }



    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<double>;

