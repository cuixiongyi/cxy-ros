#pragma once

#include "cxy_icp_rigid.h"
#include <unsupported/Eigen/NonLinearOptimization>

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar>
        class cxy_icp_rigid_lm : public cxy_icp_rigid<_Scalar>
        {


        protected:



        public:
            _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);

            cxy_icp_rigid_lm();



        };
    }
}
