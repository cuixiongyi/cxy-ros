#include "tracker/cxy_tracker.h"


namespace cxy
{

    template<typename _Scalar>
    cxy_tracker<_Scalar>::cxy_tracker(const cxy_config *const config)
    : config_(config)
    , kinematic_(config)
    , x_(cxy_config::joint_DoFs)
    {
        x_.setZero();
    }

    template<typename _Scalar>
    cxy_tracker<_Scalar>::~cxy_tracker()
    {

    }


    template<typename _Scalar>
    _Scalar cxy_tracker<_Scalar>::runOptimization()
    {
        kinematic_.constructModelPoints(x_);
        std::size_t const& pointSize = kinematic_.getModelPointSize();
        cxy_optimization::cxy_cost_func_kinematic<_Scalar> func(
                cxy_config::joint_DoFs
                , pointSize
                , &kinematic_);
        Eigen::LevenbergMarquardt <cxy_optimization::cxy_cost_func_kinematic<_Scalar>, _Scalar > lm2(func);
        _Scalar res = lm2.lmder1(x_);

        return res;
    }




}

template class cxy::cxy_tracker<float>;
template class cxy::cxy_tracker<double>;
