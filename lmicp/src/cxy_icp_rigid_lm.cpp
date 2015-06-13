#include "../include/cxy_icp_rigid_lm.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar>
        cxy_icp_rigid_lm<_Scalar>::cxy_icp_rigid_lm() : cxy_icp_rigid<_Scalar>()
        {
            ;

        }

        template<typename _Scalar>
        _Scalar cxy_icp_rigid_lm<_Scalar>::icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
        {

            Eigen::LevenbergMarquardt <cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm(*this->func_);

            return lm.lmder1(x);

        }



    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_rigid_lm<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_rigid_lm<double>;
