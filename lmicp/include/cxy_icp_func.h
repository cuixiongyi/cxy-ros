#pragma once

//#include "vnl/vnl_cost_function.h"
#include "vector"
namespace cxy {
    namespace cxy_lmicp_lib {

        typedef std::vector<unsigned int> dataVectorType
        typedef unsigned int dataType
        typedef std::vector<float> paraVectorType
        typedef float paraType
        class cxy_icp_func
        {
            virtual float risidual(const dataType &list,  const paraVectorType &para);
            virtual paraVectorType residual_derivative(const dataType &list,  const paraVectorType &para);

        };
    }
}