#pragma once

#include "cxy_icp.h"
#include "cxy_transform.h"
#include "cxy_icp_rigid_func.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar>
        class cxy_icp_rigid : public cxy_icp<_Scalar>
        {

        public:
            cxy_icp_rigid();
            virtual int icp_prepare_cost_function();



        protected:


        };
    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_rigid<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_rigid<double>;
