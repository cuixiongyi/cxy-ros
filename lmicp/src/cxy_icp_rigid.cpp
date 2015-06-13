#include "cxy_icp_rigid.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        template<typename _Scalar>
        cxy_icp_rigid<_Scalar>::cxy_icp_rigid() : cxy_icp<_Scalar>()
            {


            }
        template<typename _Scalar>
        int cxy_icp_rigid<_Scalar>::icp_prepare_cost_function()
        {
            cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>* tmp = new cxy_icp_rigid_func<_Scalar>(this->modelCloud_, this->dataCloud_, this->kdtreeptr_);
            this->func_ = tmp;
        }

    }
}