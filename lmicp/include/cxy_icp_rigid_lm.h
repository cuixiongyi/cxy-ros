#pragma once

#include "cxy_icp_rigid.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar>
        class cxy_icp_rigid_lm : public cxy_icp_rigid<_Scalar>
        {


        protected:



        public:
            virtual int icp_minimization();
            cxy_icp_rigid_lm();



        };
    }
}