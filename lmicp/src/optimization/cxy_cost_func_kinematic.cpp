#include "optimization/cxy_cost_func_kinematic.h"

#include "kinematic/cxy_icp_kinematic.h"
#include "Eigen/Core"
#include <iostream>
//#include "main.h"

namespace  cxy
{


namespace cxy_optimization
{
    template<typename _Scalar>
    cxy_cost_func_kinematic<_Scalar>::cxy_cost_func_kinematic
        (int const & paraSize, int const & dataSize, cxy_kinematic::cxy_icp_kinematic<_Scalar>* const& kinematic)
        : Cxy_Cost_Func_Abstract<_Scalar, Eigen::Dynamic, Eigen::Dynamic>(paraSize, dataSize)
        , kinematic_(kinematic)
        {

        }

    template<typename _Scalar>
    _Scalar cxy_cost_func_kinematic<_Scalar>::operator()
            (MatrixX1 & x, MatrixX1& res) const
    {
        kinematic_->computeResidual(x, res);
        std::cout<<"x == "<<x<<std::endl;
        return 0;
    }

    template<typename _Scalar>
    _Scalar cxy_cost_func_kinematic<_Scalar>::df
            (MatrixX1 & x, MatrixXX& jac) const
    {
        kinematic_->computeJacobian(x, jac);
        return 0;
    }


}
}

template class cxy::cxy_optimization::cxy_cost_func_kinematic<float>;
template class cxy::cxy_optimization::cxy_cost_func_kinematic<double>;
