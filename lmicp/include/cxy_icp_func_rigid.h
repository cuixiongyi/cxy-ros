#pragma once

//#include "vnl/vnl_cost_function.h"
#include "cxy_icp_func.h"
#include "vector"
namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_icp_func_rigid : public cxy_icp_func
        {
            float risidual(const dataType &list,  const paraVectorType &para);
            paraVectorType residual_derivative(const dataType &list,  const paraVectorType &para);

        };
    }
}