#include "tracker/cxy_tracker.h"


namespace cxy
{

    template<typename _Scalar>
    cxy_tracker<_Scalar>::cxy_tracker(const cxy_config *const config)
    : config_(config)
    , kinematic_(config)
    {

    }

    template<typename _Scalar>
    cxy_tracker<_Scalar>::~cxy_tracker()
    {

    }


    template<typename _Scalar>
    void cxy_tracker<_Scalar>::runOptimization()
    {
        kinematic_.
        cxy_optimization::cxy_cost_func_kinematic<_Scalar> func();
        Eigen::LevenbergMarquardt <cxy_optimization::cxy_cost_func_kinematic<_Scalar>, _Scalar > lm2(func);
        _Scalar res = lm2.lmder1(x_);

        return;
    }




}

template class cxy::cxy_tracker<float>;
template class cxy::cxy_tracker<double>;
